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

// WiFi 설정 - config.h 파일에서 불러오기
#include "config.h"
#define TCP_SERVER_IP SERVER_IP
#define TCP_SERVER_PORT 8080  // GCS TCP 서버 포트

// WiFi 연결 이벤트
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

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

void set_motor_speed(uint8_t speed_percent)
{
    // 안전을 위해 최대 15%로 제한
    if (speed_percent > 15) {
        speed_percent = 15;
    }

    uint16_t motor_ratio = (uint16_t)(speed_percent * 655);

    // 4개 모터 모두에 동일한 속도 적용
    for (int i = 0; i < 4; i++) {
        motorsSetRatio(i, motor_ratio);
    }

    printf("Motors set to %d%% (ratio: %d)\n", speed_percent, motor_ratio);
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
