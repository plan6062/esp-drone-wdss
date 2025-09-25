/*
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "cJSON.h"

#include "led.h"
#include "motors.h"
#include "mpu6050.h"
#include "i2cdev.h"

// 비행 제어 시스템 헤더
#include "../components/core/crazyflie/stabilizer_types.h"
#include "../components/core/crazyflie/power_distribution.h"

// WiFi 설정 - config.h 파일에서 불러오기
#include "config.h"
#define TCP_SERVER_IP SERVER_IP
#define TCP_SERVER_PORT 8080  // GCS TCP 서버 포트

// WiFi 연결 이벤트
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// 안전 모니터링을 위한 전역 변수
static I2C_Dev *i2c_sensor_dev;
static bool gyro_initialized = false;
static bool safety_system_active = true;
static uint32_t last_motor_command_time = 0;

// 비행 제어 시스템 전역 변수
static sensorData_t sensorData;
static state_t state;
static setpoint_t setpoint;
static control_t control;

// 안전 임계값 (실제 비행을 위한 보수적 설정)
#define MAX_ROTATION_RATE 2000   // 최대 회전 속도 (deg/s * 131 LSB/deg/s)
#define MAX_ACCELERATION 20000   // 최대 가속도 (1.2g * 16384 = ~19660)
#define MAX_MOTOR_RUNTIME_MS 30000   // 최대 모터 연속 작동 시간 (30초)
#define STABILITY_CHECK_INTERVAL 50  // 안정성 체크 간격 (ms)

// 센서 데이터를 sensorData_t 구조체로 변환하는 함수
void update_sensor_data(void)
{
    if (!gyro_initialized) {
        return;
    }

    int16_t ax, ay, az, gx, gy, gz;
    mpu6050GetMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // MPU6050 raw 데이터를 표준 단위로 변환
    // 가속도: ±2g 설정 시 16384 LSB/g
    // 자이로: ±2000°/s 설정 시 16.4 LSB/(°/s)
    sensorData.acc.x = ax / 16384.0f;
    sensorData.acc.y = ay / 16384.0f;
    sensorData.acc.z = az / 16384.0f;

    sensorData.gyro.x = gx / 16.4f;  // deg/s
    sensorData.gyro.y = gy / 16.4f;
    sensorData.gyro.z = gz / 16.4f;

    sensorData.interruptTimestamp = esp_timer_get_time();
}

// 간단한 자세 추정 (센서 융합)
void simple_state_estimation(void)
{
    // 가속도로부터 롤/피치 추정 (간단한 방법)
    float roll = atan2f(sensorData.acc.y, sensorData.acc.z) * 180.0f / M_PI;
    float pitch = atan2f(-sensorData.acc.x, sqrtf(sensorData.acc.y * sensorData.acc.y + sensorData.acc.z * sensorData.acc.z)) * 180.0f / M_PI;

    state.attitude.roll = roll;
    state.attitude.pitch = pitch;
    // 자이로에서 yaw rate를 적분해서 yaw 추정 (간단한 방법)
    static float yaw = 0.0f;
    static uint32_t lastTime = 0;
    uint32_t currentTime = esp_timer_get_time() / 1000;
    if (lastTime > 0) {
        float dt = (currentTime - lastTime) / 1000.0f;
        yaw += sensorData.gyro.z * dt;
    }
    state.attitude.yaw = yaw;
    lastTime = currentTime;

    state.attitude.timestamp = currentTime;
}

// 간단한 PID 제어기 (기본적인 안정화)
static float pid_roll_p = 2.0f;
static float pid_pitch_p = 2.0f;
static float pid_yaw_p = 2.0f;

void simple_attitude_controller(void)
{
    // 목표값과 현재값의 차이 계산
    float roll_error = setpoint.attitude.roll - state.attitude.roll;
    float pitch_error = setpoint.attitude.pitch - state.attitude.pitch;
    float yaw_error = setpoint.attitude.yaw - state.attitude.yaw;

    // 간단한 P 제어 (추후 PID로 확장 가능)
    control.roll = (int16_t)(roll_error * pid_roll_p);
    control.pitch = (int16_t)(pitch_error * pid_pitch_p);
    control.yaw = (int16_t)(yaw_error * pid_yaw_p);

    // Thrust는 외부에서 설정
    // control.thrust는 이미 set_motor_speed에서 설정됨
}

// 자이로 센서 초기화 함수
bool init_gyro_sensor(void)
{
    printf("MPU6050 자이로 센서 초기화 시작...\n");

    // I2C 센서 버스 사용 (I2C0_DEV는 센서용)
    i2c_sensor_dev = I2C0_DEV;

    // 원본과 동일하게 센서 안정화를 위해 충분히 대기 (2초)
    printf("센서 안정화를 위해 2초 대기 중...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));  // 원본과 동일한 2초 대기

    // 중요: I2C 버스를 먼저 초기화해야 함!
    printf("I2C 버스 초기화 중...\n");
    if (!i2cdevInit(i2c_sensor_dev)) {
        printf("에러: I2C 버스 초기화 실패!\n");
        gyro_initialized = false;
        return false;
    }
    printf("I2C 버스 초기화 완료\n");

    // MPU6050 초기화 - 원본과 동일한 순서로!
    printf("MPU6050 센서 초기화 중...\n");
    mpu6050Init(i2c_sensor_dev);  // I2C 포트와 주소 설정

    printf("센서 하드웨어 리셋 중...\n");
    mpu6050Reset();  // 센서 리셋
    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms 대기

    printf("SLEEP 모드 해제 중... (핵심!)\n");
    mpu6050SetSleepEnabled(false);  // SLEEP 모드 해제 ← 이것이 핵심!
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms 대기

    printf("클럭 소스 설정 중...\n");
    mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);  // X축 자이로를 클럭 소스로
    vTaskDelay(pdMS_TO_TICKS(100));  // 클럭 안정화 대기

    // 원본과 동일한 방식으로 연결 테스트
    printf("MPU6050 연결 테스트 중...\n");
    if (mpu6050TestConnection() == true) {
        printf("MPU6050 I2C 연결 [성공].\n");

        // 상세 센서 진단 시작
        printf("=== 상세 센서 진단 ===\n");

        // 센서 상태 레지스터 확인
        uint8_t pwr_mgmt_1, pwr_mgmt_2;
        i2cdevReadByte(i2c_sensor_dev, 0x68, 0x6B, &pwr_mgmt_1);  // PWR_MGMT_1
        i2cdevReadByte(i2c_sensor_dev, 0x68, 0x6C, &pwr_mgmt_2);  // PWR_MGMT_2
        printf("전원 관리 상태: PWR_MGMT_1=0x%02X, PWR_MGMT_2=0x%02X\n", pwr_mgmt_1, pwr_mgmt_2);

        if (pwr_mgmt_1 & 0x40) {
            printf("경고: 센서가 SLEEP 모드입니다!\n");
        }

        // 여러 번 센서 데이터 읽기 테스트 (센서가 업데이트되는지 확인)
        printf("센서 데이터 연속 테스트 (5회):\n");
        bool data_changing = false;
        int16_t prev_x = 0, prev_y = 0, prev_z = 0;

        for (int i = 0; i < 5; i++) {
            int16_t test_x, test_y, test_z;
            mpu6050GetRotation(&test_x, &test_y, &test_z);
            printf("  테스트 %d: gx=%d, gy=%d, gz=%d\n", i+1, test_x, test_y, test_z);

            if (i > 0 && (test_x != prev_x || test_y != prev_y || test_z != prev_z)) {
                data_changing = true;
            }
            prev_x = test_x; prev_y = test_y; prev_z = test_z;
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!data_changing && prev_x == 0 && prev_y == 0 && prev_z == 0) {
            printf("치명적 문제: 센서 데이터가 전혀 변하지 않고 모두 0입니다!\n");
            printf("센서가 SLEEP 모드이거나 하드웨어 문제일 가능성이 높습니다.\n");
            gyro_initialized = false;
            return false;
        } else if (!data_changing) {
            printf("경고: 센서 데이터가 변하지 않습니다. 센서가 고정된 값을 출력 중입니다.\n");
        } else {
            printf("좋음: 센서 데이터가 정상적으로 변화하고 있습니다.\n");
        }

        gyro_initialized = true;
        return true;
    } else {
        printf("MPU6050 I2C 연결 [실패].\n");
        printf("센서를 확인하고 전원을 껐다 켜주세요.\n");
        gyro_initialized = false;
        return false;
    }
}

// 실제 비행을 위한 통합 안전 모니터링 함수
bool check_flight_safety(void)
{
    if (!safety_system_active) {
        return true;
    }

    uint32_t current_time = esp_timer_get_time() / 1000; // ms 단위로 변환

    // 1. 자이로 센서 안정성 검사 (가장 중요!)
    if (gyro_initialized) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050GetMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // 회전 속도 안전성 검사
        if (abs(gx) > MAX_ROTATION_RATE || abs(gy) > MAX_ROTATION_RATE || abs(gz) > MAX_ROTATION_RATE) {
            printf("위험: 과도한 회전 감지! gx=%d, gy=%d, gz=%d (임계값: %d)\n",
                   gx, gy, gz, MAX_ROTATION_RATE);
            return false;
        }

        // 가속도 안전성 검사
        if (abs(ax) > MAX_ACCELERATION || abs(ay) > MAX_ACCELERATION || abs(az) > MAX_ACCELERATION) {
            printf("위험: 과도한 가속도 감지! ax=%d, ay=%d, az=%d (임계값: %d)\n",
                   ax, ay, az, MAX_ACCELERATION);
            return false;
        }

        // 주기적으로 센서 상태 출력 (5초마다)
        static uint32_t last_sensor_debug = 0;
        if (current_time - last_sensor_debug > 5000) {
            printf("센서 상태: 회전(gx=%d,gy=%d,gz=%d) 가속도(ax=%d,ay=%d,az=%d)\n",
                   gx, gy, gz, ax, ay, az);
            last_sensor_debug = current_time;
        }
    } else {
        printf("경고: 자이로 센서 없이 비행 중! 매우 위험합니다.\n");
    }

    // 2. 모터 연속 작동 시간 체크
    if (last_motor_command_time > 0) {
        uint32_t runtime = current_time - last_motor_command_time;
        if (runtime > MAX_MOTOR_RUNTIME_MS) {
            printf("안전: 모터 최대 작동 시간 초과 (%lu ms). 자동 정지합니다.\n", runtime);
            return false;
        }

        // 주기적으로 모터 작동 시간 출력 (10초마다)
        static uint32_t last_time_debug = 0;
        if (current_time - last_time_debug > 10000) {
            printf("비행 상태: 모터 작동 시간 %lu초 (최대 %d초)\n", runtime/1000, MAX_MOTOR_RUNTIME_MS/1000);
            last_time_debug = current_time;
        }
    }

    return true; // 안전 상태
}

// WiFi 이벤트 핸들러
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        printf("WiFi 스테이션 시작됨, 연결 시도...\n");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        printf("WiFi 연결 끊어짐 - 이유: %d, SSID: %s\n", disconnected->reason, disconnected->ssid);
        printf("재연결 시도...\n");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("WiFi 연결됨! IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// WiFi 초기화
void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,  // 더 관대한 인증
            .channel = 0,  // 자동 채널 선택
            .scan_method = WIFI_ALL_CHANNEL_SCAN,  // 모든 채널 스캔
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("WiFi 초기화 완료. %s 연결 중...\n", WIFI_SSID);
    printf("주의: ESP32-S2는 2.4GHz WiFi만 지원합니다!\n");
}

// TCP 서버 연결
int tcp_connect_to_server(void)
{
    struct sockaddr_in server_addr;
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (sock < 0) {
        printf("TCP 소켓 생성 실패\n");
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_SERVER_PORT);

    if (inet_pton(AF_INET, TCP_SERVER_IP, &server_addr.sin_addr) <= 0) {
        printf("서버 IP 주소 변환 실패\n");
        close(sock);
        return -1;
    }

    printf("TCP 서버에 연결 시도: %s:%d\n", TCP_SERVER_IP, TCP_SERVER_PORT);

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        printf("TCP 서버 연결 실패\n");
        close(sock);
        return -1;
    }

    printf("TCP 서버 연결 성공!\n");

    // 연결 확인 메시지 전송
    const char* hello_msg = "ESP32-S2 Drone connected\n";
    send(sock, hello_msg, strlen(hello_msg), 0);

    return sock;
}

// 새로운 통합 비행 제어 함수
void set_motor_speed_with_stabilization(uint8_t base_speed_percent)
{
    // 안전 모니터링 - 비행 중인 경우에만 체크
    if (base_speed_percent > 0 && safety_system_active) {
        if (!check_flight_safety()) {
            printf("비상: 안전성 검사 실패! 모터를 즉시 정지합니다.\n");
            base_speed_percent = 0;
            last_motor_command_time = 0; // 타이머 리셋

            // 비상 정지 후 LED로 경고 표시
            ledClearAll();
            ledSet(LED_RED, true);
        } else {
            // 모터 시작 시간 기록
            if (last_motor_command_time == 0) {
                last_motor_command_time = esp_timer_get_time() / 1000;
                printf("비행 시작: 모터 가동 시간 기록\n");
            }
        }
    } else if (base_speed_percent == 0) {
        // 모터 정지 시 타이머 리셋
        last_motor_command_time = 0;
    }

    // 실제 비행을 위한 안전 제한: 최대 85%
    if (base_speed_percent > 85) {
        printf("WARNING: Motor speed %d%% exceeded maximum 85%%, limiting to 85%%\n", base_speed_percent);
        base_speed_percent = 85;
    }

    // 안전한 최소 비행 속도 (60% 미만은 비행 불안정 위험)
    if (base_speed_percent > 0 && base_speed_percent < 60) {
        printf("WARNING: Motor speed %d%% below safe flight threshold (60%%), adjusting to 65%%\n", base_speed_percent);
        base_speed_percent = 65;
    }

    if (base_speed_percent == 0) {
        // 모터 정지
        motorsSetRatio(0, 0);
        motorsSetRatio(1, 0);
        motorsSetRatio(2, 0);
        motorsSetRatio(3, 0);
        printf("Motors STOPPED\n");
        return;
    }

    // **새로운 비행 제어 시스템 적용**
    // 1. 센서 데이터 업데이트
    update_sensor_data();

    // 2. 상태 추정
    simple_state_estimation();

    // 3. 목표값 설정 (수평 유지)
    setpoint.attitude.roll = 0.0f;   // 목표 롤: 0도
    setpoint.attitude.pitch = 0.0f;  // 목표 피치: 0도
    setpoint.attitude.yaw = state.attitude.yaw; // 현재 yaw 유지

    // 4. PID 제어
    simple_attitude_controller();

    // 5. Power Distribution (원본 esp-drone 방식 사용!)
    // base thrust를 65535 스케일로 변환
    control.thrust = (base_speed_percent * 655.0f);

    // **원본 power_distribution 시스템 사용**
    powerDistribution(&control);

    // 디버그 출력 (5초마다)
    static uint32_t lastDebugTime = 0;
    uint32_t currentTime = esp_timer_get_time() / 1000;
    if (currentTime - lastDebugTime > 5000) {
        printf("STABILIZED FLIGHT: Base=%d%%, Roll=%.1f°(%.1f), Pitch=%.1f°(%.1f), Yaw=%.1f°\n",
               base_speed_percent,
               state.attitude.roll, setpoint.attitude.roll,
               state.attitude.pitch, setpoint.attitude.pitch,
               state.attitude.yaw);
        printf("Control: R=%d P=%d Y=%d T=%.0f\n",
               control.roll, control.pitch, control.yaw, control.thrust);
        lastDebugTime = currentTime;
    }
}

// 하위 호환성을 위한 래퍼 함수
void set_motor_speed(uint8_t speed_percent)
{
    set_motor_speed_with_stabilization(speed_percent);
}

// JSON 명령 파싱 및 실행
void process_json_command(const char* json_str)
{
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        printf("JSON 파싱 실패\n");
        return;
    }

    // motor_speed 추출
    cJSON *motor_speed = cJSON_GetObjectItem(json, "motor_speed");
    uint8_t speed = 0;
    if (cJSON_IsNumber(motor_speed)) {
        speed = (uint8_t)motor_speed->valueint;
        printf("모터 속도: %d%%\n", speed);
    }

    // rgb 추출
    cJSON *rgb = cJSON_GetObjectItem(json, "rgb");
    if (cJSON_IsObject(rgb)) {
        cJSON *r = cJSON_GetObjectItem(rgb, "r");
        cJSON *g = cJSON_GetObjectItem(rgb, "g");
        cJSON *b = cJSON_GetObjectItem(rgb, "b");

        uint8_t red = cJSON_IsNumber(r) ? (uint8_t)r->valueint : 0;
        uint8_t green = cJSON_IsNumber(g) ? (uint8_t)g->valueint : 0;
        uint8_t blue = cJSON_IsNumber(b) ? (uint8_t)b->valueint : 0;

        printf("RGB: (%d,%d,%d)\n", red, green, blue);

        // RGB 값에 따라 LED 제어 (중복 방지 로직 포함)
        static int last_led_color = -1; // -1: none, 0: RED, 1: GREEN, 2: BLUE

        ledClearAll();

        // RGB 값을 크기 순으로 정렬하여 우선순위 결정
        int current_led_color;

        if (red == 0 && green == 0 && blue == 0) {
            // 모든 값이 0이면 기본으로 RED 표시
            current_led_color = 0;
        } else if (red >= green && red >= blue) {
            current_led_color = 0; // RED 우선순위
        } else if (green >= blue) {
            current_led_color = 1; // GREEN 우선순위
        } else {
            current_led_color = 2; // BLUE 우선순위
        }

        // 이전과 같은 색상이면 다음 우선순위 색상으로 변경
        if (current_led_color == last_led_color) {
            if (red > 0 && green > 0 && blue > 0) {
                // 세 값 모두 0이 아닌 경우에만 대체 색상 선택
                if (current_led_color == 0 && green >= blue) current_led_color = 1;      // RED → GREEN
                else if (current_led_color == 0) current_led_color = 2;                  // RED → BLUE
                else if (current_led_color == 1 && red >= blue) current_led_color = 0;   // GREEN → RED
                else if (current_led_color == 1) current_led_color = 2;                  // GREEN → BLUE
                else if (current_led_color == 2 && red >= green) current_led_color = 0;  // BLUE → RED
                else current_led_color = 1;                                              // BLUE → GREEN
            }
        }

        // LED 설정 및 출력
        switch(current_led_color) {
            case 0:
                ledSet(LED_RED, true);
                printf("LED: RED");
                break;
            case 1:
                ledSet(LED_GREEN, true);
                printf("LED: GREEN");
                break;
            case 2:
                ledSet(LED_BLUE, true);
                printf("LED: BLUE");
                break;
        }

        // 중복 방지 로깅
        if (current_led_color == last_led_color) {
            printf(" (same as last)\n");
        } else if (last_led_color != -1) {
            printf(" (changed from %s)\n",
                   last_led_color == 0 ? "RED" :
                   last_led_color == 1 ? "GREEN" : "BLUE");
        } else {
            printf("\n");
        }

        last_led_color = current_led_color;
    }

    // 모터 제어
    set_motor_speed(speed);

    cJSON_Delete(json);
}

void app_main()
{
    printf("=== ESP32-S2 Drone 0 Started! ===\n");

    // NVS 초기화 (WiFi용)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // LED 초기화
    printf("Initializing LED...\n");
    ledInit();
    printf("LED initialized!\n");

    // 모터 초기화
    printf("Initializing Motors...\n");
    motorsInit(motorMapDefaultBrushed);
    printf("Motors initialized!\n");

    // 실제 비행을 위한 자이로 센서 초기화 (중요!)
    printf("Initializing MPU6050 Gyro Sensor for Real Flight Safety...\n");
    if (init_gyro_sensor()) {
        printf("*** MPU6050 자이로 센서 초기화 완료 - 안전한 비행 준비됨! ***\n");
    } else {
        printf("*** 경고: MPU6050 센서 초기화 실패 - 비행 시 매우 위험! ***\n");
    }

    // Power Distribution 시스템 초기화
    printf("Initializing Power Distribution System...\n");
    powerDistributionInit();
    printf("Power Distribution System initialized!\n");

    // 비행 안전 시스템 활성화
    safety_system_active = true;
    printf("Advanced Flight Safety System with Gyro Monitoring ACTIVE!\n");

    // WiFi 초기화 및 연결
    printf("Initializing WiFi...\n");
    wifi_init_sta();

    // WiFi 연결 대기
    printf("Waiting for WiFi connection...\n");
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        false,
        true,
        portMAX_DELAY
    );

    if (bits & WIFI_CONNECTED_BIT) {
        printf("=== WiFi 연결 성공! ===\n");
    } else {
        printf("=== WiFi 연결 실패! 타임아웃 발생 ===\n");
        printf("=== 오프라인 테스트 모드로 진행 ===\n");

        // WiFi 정지
        esp_wifi_stop();
    }

    // WiFi 연결 확인
    if (!(bits & WIFI_CONNECTED_BIT)) {
        printf("WiFi가 연결되지 않았으므로 TCP 연결을 시도하지 않습니다.\n");
        printf("LED와 모터 테스트 모드로 진행합니다.\n");

        // 간단한 LED/모터 테스트 루프
        int test_count = 0;
        while (1) {
            printf("=== 오프라인 테스트 %d ===\n", test_count);

            switch(test_count % 4) {
                case 0:
                    printf("RED LED + 10%% Motors\n");
                    ledClearAll();
                    ledSet(LED_RED, true);
                    set_motor_speed(10);
                    break;
                case 1:
                    printf("GREEN LED + 20%% Motors\n");
                    ledClearAll();
                    ledSet(LED_GREEN, true);
                    set_motor_speed(20);
                    break;
                case 2:
                    printf("BLUE LED + 30%% Motors\n");
                    ledClearAll();
                    ledSet(LED_BLUE, true);
                    set_motor_speed(30);
                    break;
                case 3:
                    printf("LED OFF + Motors STOP\n");
                    ledClearAll();
                    set_motor_speed(0);
                    break;
            }

            test_count++;
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    // TCP 연결 및 통신 루프
    char recv_buffer[1024];
    int reconnect_delay = 2000;

    while (1) {
        // TCP 서버에 연결
        int tcp_socket = tcp_connect_to_server();

        if (tcp_socket < 0) {
            printf("TCP 연결 실패, %d초 후 재시도...\n", reconnect_delay / 1000);
            vTaskDelay(pdMS_TO_TICKS(reconnect_delay));
            continue;
        }

        printf("=== TCP 연결됨! 명령 대기 중... ===\n");
        reconnect_delay = 2000; // 연결 성공 시 지연시간 초기화

        // TCP 데이터 수신 루프
        while (1) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(tcp_socket, &read_fds);

            struct timeval timeout;
            timeout.tv_sec = 10;  // 10초 타임아웃
            timeout.tv_usec = 0;

            int activity = select(tcp_socket + 1, &read_fds, NULL, NULL, &timeout);

            if (activity < 0) {
                printf("select() 오류\n");
                break;
            } else if (activity == 0) {
                // 타임아웃 - 연결 상태 확인용 ping 전송
                const char* ping = "ping\n";
                if (send(tcp_socket, ping, strlen(ping), 0) < 0) {
                    printf("Ping 전송 실패, 연결 끊어짐\n");
                    break;
                }
                printf("Ping sent to server\n");
                continue;
            }

            if (FD_ISSET(tcp_socket, &read_fds)) {
                int bytes_received = recv(tcp_socket, recv_buffer, sizeof(recv_buffer) - 1, 0);

                if (bytes_received <= 0) {
                    printf("TCP 연결 끊어짐 (bytes_received: %d)\n", bytes_received);
                    break;
                }

                recv_buffer[bytes_received] = '\0';

                // 여러 JSON 메시지가 한번에 올 수 있으므로 줄 단위로 처리
                char *line_start = recv_buffer;
                char *line_end;

                while ((line_end = strchr(line_start, '\n')) != NULL) {
                    *line_end = '\0';
                    if (strlen(line_start) > 0 && strcmp(line_start, "ping") != 0) {
                        printf("=== 명령 수신 ===\n%s\n", line_start);
                        process_json_command(line_start);
                        printf("=== 명령 실행 완료 ===\n");
                    }
                    line_start = line_end + 1;
                }
            }
        }

        // 연결 종료
        printf("TCP 연결 종료, 재연결 시도...\n");
        close(tcp_socket);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
