#include "gcs_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "led.h"
#include "motors.h"
#include <string.h>

// config.h에서 WiFi 및 서버 설정 불러오기
#include "config.h"

static const char* TAG = "GCS_COMM";

#define GCS_TASK_STACK_SIZE 8192
#define GCS_TASK_PRIORITY 5
#define RGB_THRESHOLD 128

// WiFi 설정 - config.h에서 불러오기
#define TCP_SERVER_IP SERVER_IP
#define TCP_SERVER_PORT 8080

static bool gcs_initialized = false;
static TaskHandle_t gcs_task_handle = NULL;
static int tcp_socket = -1;

// WiFi 연결 이벤트 그룹
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// 함수 선언
static bool gcs_parse_command(const char* json_str, gcs_command_t* cmd);

// WiFi 이벤트 핸들러
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi 연결 끊어짐, 재연결 시도...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi 연결됨, IP:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// TCP 소켓 연결
static int tcp_connect_to_server(void)
{
    struct sockaddr_in server_addr;
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (sock < 0) {
        ESP_LOGE(TAG, "TCP 소켓 생성 실패");
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_SERVER_PORT);

    if (inet_pton(AF_INET, TCP_SERVER_IP, &server_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "서버 IP 주소 변환 실패");
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP 서버에 연결 시도: %s:%d", TCP_SERVER_IP, TCP_SERVER_PORT);

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "TCP 서버 연결 실패");
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "TCP 서버 연결 성공!");

    // 연결 확인 메시지 전송
    const char* hello_msg = "ESP32 connected\n";
    send(sock, hello_msg, strlen(hello_msg), 0);

    return sock;
}

// TCP 데이터 수신 및 처리
static void tcp_process_received_data(const char* data, int len)
{
    if (len <= 0) return;

    char json_buffer[512];
    int copy_len = len < sizeof(json_buffer) - 1 ? len : sizeof(json_buffer) - 1;
    memcpy(json_buffer, data, copy_len);
    json_buffer[copy_len] = '\0';

    ESP_LOGI(TAG, "TCP 데이터 수신: %s", json_buffer);

    gcs_command_t command;
    if (gcs_parse_command(json_buffer, &command)) {
        gcs_process_rgb(command.rgb);
        gcs_apply_motor_speed(command.motor_speed);
        ESP_LOGI(TAG, "명령 실행: motor=%d%%, rgb=(%d,%d,%d)",
                command.motor_speed, command.rgb.r, command.rgb.g, command.rgb.b);
    }
}

// WiFi Station 모드 초기화
static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi Station 모드 초기화 완료");
}

void gcs_process_rgb(gcs_rgb_t rgb)
{
    ledClearAll();

    uint8_t max_val = rgb.r;
    if (rgb.g > max_val) max_val = rgb.g;
    if (rgb.b > max_val) max_val = rgb.b;

    if (max_val < RGB_THRESHOLD) {
        return;
    }

    if (rgb.r == max_val && rgb.r >= RGB_THRESHOLD) {
        ledSet(LED_RED, true);
    } else if (rgb.g == max_val && rgb.g >= RGB_THRESHOLD) {
        ledSet(LED_GREEN, true);
    } else if (rgb.b == max_val && rgb.b >= RGB_THRESHOLD) {
        ledSet(LED_BLUE, true);
    }

    ESP_LOGI(TAG, "RGB (%d,%d,%d) -> LED 적용", rgb.r, rgb.g, rgb.b);
}

void gcs_apply_motor_speed(uint8_t speed_percent)
{
    if (speed_percent > 100) {
        speed_percent = 100;
    }

    uint16_t motor_ratio = (uint16_t)(speed_percent * 655);

    for (int i = 0; i < 4; i++) {
        motorsSetRatio(i, motor_ratio);
    }

    ESP_LOGI(TAG, "모터 속도 설정: %d%% (ratio: %d)", speed_percent, motor_ratio);
}

static bool gcs_parse_command(const char* json_str, gcs_command_t* cmd)
{
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        ESP_LOGE(TAG, "JSON 파싱 실패");
        return false;
    }

    cJSON *motor_speed = cJSON_GetObjectItem(json, "motor_speed");
    if (cJSON_IsNumber(motor_speed)) {
        cmd->motor_speed = (uint8_t)motor_speed->valueint;
    } else {
        cmd->motor_speed = 0;
    }

    cJSON *rgb = cJSON_GetObjectItem(json, "rgb");
    if (cJSON_IsObject(rgb)) {
        cJSON *r = cJSON_GetObjectItem(rgb, "r");
        cJSON *g = cJSON_GetObjectItem(rgb, "g");
        cJSON *b = cJSON_GetObjectItem(rgb, "b");

        cmd->rgb.r = cJSON_IsNumber(r) ? (uint8_t)r->valueint : 0;
        cmd->rgb.g = cJSON_IsNumber(g) ? (uint8_t)g->valueint : 0;
        cmd->rgb.b = cJSON_IsNumber(b) ? (uint8_t)b->valueint : 0;
    } else {
        cmd->rgb.r = cmd->rgb.g = cmd->rgb.b = 0;
    }

    cJSON_Delete(json);
    return true;
}

static void gcs_comm_task(void *parameters)
{
    ESP_LOGI(TAG, "GCS 통신 태스크 시작");

    // WiFi 연결 대기
    ESP_LOGI(TAG, "WiFi 연결 대기 중...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    char recv_buffer[1024];
    int reconnect_delay = 1000; // 초기 재연결 지연시간 (ms)

    while (1) {
        // TCP 서버에 연결
        tcp_socket = tcp_connect_to_server();

        if (tcp_socket < 0) {
            ESP_LOGW(TAG, "TCP 연결 실패, %d초 후 재시도...", reconnect_delay / 1000);
            vTaskDelay(pdMS_TO_TICKS(reconnect_delay));
            reconnect_delay = reconnect_delay < 30000 ? reconnect_delay * 2 : 30000; // 최대 30초
            continue;
        }

        reconnect_delay = 1000; // 연결 성공 시 지연시간 초기화

        // TCP 데이터 수신 루프
        while (1) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(tcp_socket, &read_fds);

            struct timeval timeout;
            timeout.tv_sec = 5;  // 5초 타임아웃
            timeout.tv_usec = 0;

            int activity = select(tcp_socket + 1, &read_fds, NULL, NULL, &timeout);

            if (activity < 0) {
                ESP_LOGE(TAG, "select() 오류");
                break;
            } else if (activity == 0) {
                // 타임아웃 - 연결 상태 확인용 ping 전송
                const char* ping = "ping\n";
                if (send(tcp_socket, ping, strlen(ping), 0) < 0) {
                    ESP_LOGW(TAG, "Ping 전송 실패, 연결 끊어짐");
                    break;
                }
                continue;
            }

            if (FD_ISSET(tcp_socket, &read_fds)) {
                int bytes_received = recv(tcp_socket, recv_buffer, sizeof(recv_buffer) - 1, 0);

                if (bytes_received <= 0) {
                    ESP_LOGW(TAG, "TCP 연결 끊어짐 (bytes_received: %d)", bytes_received);
                    break;
                }

                recv_buffer[bytes_received] = '\0';

                // 여러 JSON 메시지가 한번에 올 수 있으므로 줄 단위로 처리
                char *line_start = recv_buffer;
                char *line_end;

                while ((line_end = strchr(line_start, '\n')) != NULL) {
                    *line_end = '\0';
                    if (strlen(line_start) > 0 && strcmp(line_start, "ping") != 0) {
                        tcp_process_received_data(line_start, strlen(line_start));
                    }
                    line_start = line_end + 1;
                }
            }
        }

        // 연결 종료
        ESP_LOGI(TAG, "TCP 연결 종료, 재연결 시도...");
        close(tcp_socket);
        tcp_socket = -1;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void gcs_comm_init(void)
{
    if (gcs_initialized) {
        ESP_LOGW(TAG, "GCS 통신이 이미 초기화됨");
        return;
    }

    // LED와 모터 초기화
    ledInit();
    motorsInit(motorMapDefaultBrushed);

    // WiFi Station 모드 초기화
    wifi_init_sta();

    gcs_initialized = true;
    ESP_LOGI(TAG, "GCS 통신 초기화 완료");
}

void gcs_comm_start(void)
{
    if (!gcs_initialized) {
        ESP_LOGE(TAG, "GCS 통신이 초기화되지 않음");
        return;
    }

    if (gcs_task_handle != NULL) {
        ESP_LOGW(TAG, "GCS 통신 태스크가 이미 실행 중");
        return;
    }

    BaseType_t result = xTaskCreate(
        gcs_comm_task,
        "gcs_comm_task",
        GCS_TASK_STACK_SIZE,
        NULL,
        GCS_TASK_PRIORITY,
        &gcs_task_handle
    );

    if (result == pdPASS) {
        ESP_LOGI(TAG, "GCS 통신 태스크 시작 성공");
    } else {
        ESP_LOGE(TAG, "GCS 통신 태스크 생성 실패");
    }
}