#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "mbedtls/md.h"
#include "mbedtls/base64.h"

/* AI 语音播报相关 */
#include "ai_config.h"
#include "spark_chat.h"
#include "baidu_tts.h"
#include "max98357a.h"
/* 显示屏 */
#include "st7735_display.h"

#define TAG "A39C_RX"

/* ====== WiFi STA 配置（连接路由器） ====== */
#define WIFI_STA_SSID      "OPPO"
#define WIFI_STA_PASS      "123456789"
#define WIFI_MAX_RETRY     10
#define WIFI_RECONNECT_INTERVAL_MS  5000  /* 断线重连间隔(毫秒) */

/* ====== OneNet MQTT 配置 ====== */
#define ONENET_PRODUCT_ID   "W9BY3SlO8w"
#define ONENET_DEVICE_NAME  "dachuang"
#define ONENET_DEVICE_KEY   "aDFIWDNUeFpLTTdUTzdCS1VzdWlyV3JLS3pnR2tkdVU="
#define ONENET_BROKER_URL   "mqtt://mqtts.heclouds.com"
#define ONENET_ET           "4102444800"  /* 2100-01-01, token 长期有效 */
#define ONENET_RES          "products/" ONENET_PRODUCT_ID "/devices/" ONENET_DEVICE_NAME
#define ONENET_TOPIC_POST   "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/post"

/* ====== 引脚配置（ESP32-S3 N16R8） ====== */
/* 注意: GPIO26-37 被 Octal Flash/PSRAM 占用，不可使用 */
#define A39C_UART_NUM      UART_NUM_2
#define A39C_TX_PIN        GPIO_NUM_17   /* ESP32-S3 TX -> A39C RXD */
#define A39C_RX_PIN        GPIO_NUM_16   /* ESP32-S3 RX <- A39C TXD */
#define A39C_BAUD          9600
#define A39C_MD0_PIN       GPIO_NUM_15   /* 模式选择引脚 MD0 */
#define A39C_MD1_PIN       GPIO_NUM_14   /* 模式选择引脚 MD1 */
#define A39C_AUX_PIN       GPIO_NUM_18   /* AUX 状态引脚 */
/*
 * A39C 模块模式脚（MD0/MD1）
 * 根据你提供的数据手册：
 * - MD0=0, MD1=0：配置状态
 * - MD0=1, MD1=0：一般工作状态（收发测试）
 * - MD0=1, MD1=1：休眠/低功耗
 */
#define A39C_MD0_LEVEL     1
#define A39C_MD1_LEVEL     0

/* ====== 帧格式 ====== */
#define FRAME_HEAD1        0xAA
#define FRAME_HEAD2        0x55
#define FRAME_PAYLOAD_LEN  0x19
#define FRAME_TOTAL_LEN    29

typedef struct {
    uint8_t seq;
    float lux;
    float env_temp_c;
    float env_humi_pct;
    float press_kpa;
    float soil_temp_c;
    float soil_humi_pct;
    float ph;
    uint16_t n;
    uint16_t p;
    uint16_t k;
} sensor_packet_t;

typedef enum {
    CONTROL_MODE_MANUAL = 0,
    CONTROL_MODE_SMART = 1,
} control_mode_t;

typedef struct {
    uint8_t fan_speed;    /* 0-100 */
    uint8_t pump_speed;   /* 0-100 */
    uint8_t servo_angle;  /* 0-180 */
    uint32_t update_time_s;
    char reason[96];
} control_state_t;

static uint32_t ok_count = 0;
static uint32_t fail_count = 0;
static uint32_t rx_bytes = 0;
static uint32_t aux_low_count = 0;

/* 前置声明：跨模块共享状态 */
static bool mqtt_connected = false;
static bool mqtt_pending_send = false;

/* 控制模式与执行器状态 */
static control_mode_t g_control_mode = CONTROL_MODE_MANUAL;
static control_state_t g_control_state = {
    .fan_speed = 0,
    .pump_speed = 0,
    .servo_angle = 90,
    .update_time_s = 0,
    .reason = "init",
};
static volatile bool g_smart_control_force_run = false;

/* 控制命令协议: A5 5A MODE FAN PUMP SERVO_L SERVO_H RSV XOR */
#define CONTROL_FRAME_HEAD1 0xA5
#define CONTROL_FRAME_HEAD2 0x5A
#define CONTROL_FRAME_LEN   9

static uint8_t clamp_u8(int val, int min_v, int max_v)
{
    if (val < min_v) return (uint8_t)min_v;
    if (val > max_v) return (uint8_t)max_v;
    return (uint8_t)val;
}

static void json_escape_text(const char *src, char *dst, size_t dst_size)
{
    if (!dst || dst_size == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }

    size_t di = 0;
    for (size_t si = 0; src[si] != '\0' && di + 1 < dst_size; si++) {
        char c = src[si];
        if (c == '"' || c == '\\' || c == '\n' || c == '\r' || c == '\t') {
            if (di + 2 >= dst_size) {
                break;
            }
            dst[di++] = '\\';
            dst[di++] = (c == '"' || c == '\\') ? c : ' ';
        } else {
            dst[di++] = c;
        }
    }
    dst[di] = '\0';
}

static void send_control_frame_to_stm32(control_mode_t mode, uint8_t fan, uint8_t pump, uint8_t servo)
{
    uint8_t frame[CONTROL_FRAME_LEN] = {0};
    frame[0] = CONTROL_FRAME_HEAD1;
    frame[1] = CONTROL_FRAME_HEAD2;
    frame[2] = (mode == CONTROL_MODE_SMART) ? 0x02 : 0x01;
    frame[3] = fan;
    frame[4] = pump;
    frame[5] = (uint8_t)(servo & 0xFF);
    frame[6] = (uint8_t)((servo >> 8) & 0xFF);
    frame[7] = 0x00;
    for (int i = 0; i < CONTROL_FRAME_LEN - 1; i++) {
        frame[8] ^= frame[i];
    }

    int written = uart_write_bytes(A39C_UART_NUM, frame, CONTROL_FRAME_LEN);
    ESP_LOGI(TAG, "下发控制帧 mode=%s fan=%u pump=%u servo=%u bytes=%d",
             mode == CONTROL_MODE_SMART ? "smart" : "manual",
             fan, pump, servo, written);
}

static void apply_control_and_send(control_mode_t mode, int fan_speed, int pump_speed,
                                   int servo_angle, const char *reason)
{
    g_control_state.fan_speed = clamp_u8(fan_speed, 0, 100);
    g_control_state.pump_speed = clamp_u8(pump_speed, 0, 100);
    g_control_state.servo_angle = clamp_u8(servo_angle, 0, 180);
    g_control_state.update_time_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (reason && reason[0]) {
        snprintf(g_control_state.reason, sizeof(g_control_state.reason), "%s", reason);
    }
    g_control_mode = mode;

    send_control_frame_to_stm32(g_control_mode,
                                g_control_state.fan_speed,
                                g_control_state.pump_speed,
                                g_control_state.servo_angle);
}

/* 最新一帧数据（供 HTTP 接口读取） */
static sensor_packet_t latest_pkt = {0};
static bool has_data = false;

/* ====== 历史数据环形缓冲区（趋势图表） ====== */
#define HISTORY_MAX 60

typedef struct {
    float env_temp;
    float env_humi;
    float soil_temp;
    float soil_humi;
    float lux;
    float ph;
    uint32_t timestamp_s;   /* 自启动以来的秒数 */
} history_point_t;

static history_point_t history_buf[HISTORY_MAX];
static int history_head = 0;   /* 下一个写入位置 */
static int history_count = 0;  /* 当前存储的数据点数 */

static void history_push(const sensor_packet_t *pkt)
{
    history_point_t *pt = &history_buf[history_head];
    pt->env_temp  = pkt->env_temp_c;
    pt->env_humi  = pkt->env_humi_pct;
    pt->soil_temp = pkt->soil_temp_c;
    pt->soil_humi = pkt->soil_humi_pct;
    pt->lux       = pkt->lux;
    pt->ph        = pkt->ph;
    pt->timestamp_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    history_head = (history_head + 1) % HISTORY_MAX;
    if (history_count < HISTORY_MAX) history_count++;
}

/* 嵌入的 HTML 文件 */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static void log_packet(const sensor_packet_t *pkt)
{
    ESP_LOGI(TAG, "解析成功: SEQ=%u", pkt->seq);
    ESP_LOGI(TAG, "  光照=%.2f lx, 环境温度=%.1f ℃, 环境湿度=%.1f %%",
             pkt->lux, pkt->env_temp_c, pkt->env_humi_pct);
    ESP_LOGI(TAG, "  气压=%.1f hPa, 土壤温度=%.1f ℃, 土壤湿度=%.1f %%",
             pkt->press_kpa, pkt->soil_temp_c, pkt->soil_humi_pct);
    ESP_LOGI(TAG, "  pH=%.2f, N=%u mg/kg, P=%u mg/kg, K=%u mg/kg",
             pkt->ph, pkt->n, pkt->p, pkt->k);
}

static inline uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline int16_t i16le(const uint8_t *p)
{
    return (int16_t)u16le(p);
}

static inline uint32_t u32le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static bool parse_frame(const uint8_t *f, sensor_packet_t *out)
{
    if (f[0] != FRAME_HEAD1 || f[1] != FRAME_HEAD2) {
        return false;
    }
    if (f[2] != FRAME_PAYLOAD_LEN) {
        return false;
    }

    uint8_t xor_val = 0;
    for (int i = 0; i < FRAME_TOTAL_LEN - 1; i++) {
        xor_val ^= f[i];
    }
    if (xor_val != f[FRAME_TOTAL_LEN - 1]) {
        return false;
    }

    out->seq           = f[3];
    out->lux           = (float)u32le(&f[4]) / 100.0f;
    out->env_temp_c    = (float)i16le(&f[8]) / 10.0f;
    out->env_humi_pct  = (float)u16le(&f[10]) / 10.0f;
    out->press_kpa     = (float)u32le(&f[12]) / 100.0f;  /* 实际单位 hPa */
    out->soil_temp_c   = (float)i16le(&f[16]) / 10.0f;
    out->soil_humi_pct = (float)u16le(&f[18]) / 10.0f;
    out->ph            = (float)u16le(&f[20]) / 100.0f;
    out->n             = u16le(&f[22]);
    out->p             = u16le(&f[24]);
    out->k             = u16le(&f[26]);
    return true;
}

/* ====== WiFi STA 初始化 ====== */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        mqtt_connected = false;  /* WiFi断了MQTT肯定也断了 */
        s_retry_num++;
        ESP_LOGW(TAG, "WiFi断开，第%d次重连...", s_retry_num);
        vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_INTERVAL_MS));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "获得IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "正在连接WiFi: %s ...", WIFI_STA_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi已连接: %s", WIFI_STA_SSID);
    } else {
        ESP_LOGW(TAG, "WiFi连接超时，系统继续启动，后台持续重连");
    }
}

/* ====== HTTP 服务器 ====== */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    size_t len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, len);
    return ESP_OK;
}

static esp_err_t api_data_handler(httpd_req_t *req)
{
    char buf[420];
    char reason_esc[192];
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    if (has_data) {
        snprintf(buf, sizeof(buf),
            "{\"seq\":%u,\"lux\":%.2f,\"env_temp\":%.1f,\"env_humi\":%.1f,"
            "\"press\":%.1f,\"soil_temp\":%.1f,\"soil_humi\":%.1f,"
            "\"ph\":%.2f,\"n\":%u,\"p\":%u,\"k\":%u,"
            "\"ok\":%lu,\"fail\":%lu,"
            "\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"reason\":\"%s\"}",
            latest_pkt.seq, latest_pkt.lux, latest_pkt.env_temp_c, latest_pkt.env_humi_pct,
            latest_pkt.press_kpa, latest_pkt.soil_temp_c, latest_pkt.soil_humi_pct,
            latest_pkt.ph, latest_pkt.n, latest_pkt.p, latest_pkt.k,
            (unsigned long)ok_count, (unsigned long)fail_count,
            g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
            g_control_state.fan_speed,
            g_control_state.pump_speed,
            g_control_state.servo_angle,
            reason_esc);
    } else {
        snprintf(buf, sizeof(buf),
            "{\"seq\":0,\"lux\":0,\"env_temp\":0,\"env_humi\":0,"
            "\"press\":0,\"soil_temp\":0,\"soil_humi\":0,"
            "\"ph\":0,\"n\":0,\"p\":0,\"k\":0,"
            "\"ok\":0,\"fail\":0,"
            "\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"reason\":\"%s\"}",
            g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
            g_control_state.fan_speed,
            g_control_state.pump_speed,
            g_control_state.servo_angle,
            reason_esc);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t api_control_get_handler(httpd_req_t *req)
{
    char buf[320];
    char reason_esc[192];
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    snprintf(buf, sizeof(buf),
             "{\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,"
             "\"updated\":%lu,\"reason\":\"%s\"}",
             g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
             g_control_state.fan_speed,
             g_control_state.pump_speed,
             g_control_state.servo_angle,
             (unsigned long)g_control_state.update_time_s,
             reason_esc);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t api_control_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content len");
        return ESP_FAIL;
    }

    char body[520] = {0};
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body read failed");
        return ESP_FAIL;
    }
    body[received] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "json parse failed");
        return ESP_FAIL;
    }

    const cJSON *mode = cJSON_GetObjectItem(root, "mode");
    if (mode && cJSON_IsString(mode) && mode->valuestring) {
        if (strcmp(mode->valuestring, "smart") == 0) {
            g_control_mode = CONTROL_MODE_SMART;
            snprintf(g_control_state.reason, sizeof(g_control_state.reason), "web set smart");
            g_smart_control_force_run = true;
        } else {
            g_control_mode = CONTROL_MODE_MANUAL;
            snprintf(g_control_state.reason, sizeof(g_control_state.reason), "web set manual");
        }
        g_control_state.update_time_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    }

    if (g_control_mode == CONTROL_MODE_MANUAL) {
        int fan = g_control_state.fan_speed;
        int pump = g_control_state.pump_speed;
        int servo = g_control_state.servo_angle;

        const cJSON *fan_j = cJSON_GetObjectItem(root, "fan");
        const cJSON *pump_j = cJSON_GetObjectItem(root, "pump");
        const cJSON *servo_j = cJSON_GetObjectItem(root, "servo");

        if (fan_j && cJSON_IsNumber(fan_j)) fan = fan_j->valueint;
        if (pump_j && cJSON_IsNumber(pump_j)) pump = pump_j->valueint;
        if (servo_j && cJSON_IsNumber(servo_j)) servo = servo_j->valueint;

        apply_control_and_send(CONTROL_MODE_MANUAL, fan, pump, servo, "manual web control");
    } else {
        send_control_frame_to_stm32(g_control_mode,
                                    g_control_state.fan_speed,
                                    g_control_state.pump_speed,
                                    g_control_state.servo_angle);
    }

    cJSON_Delete(root);

    char reason_esc[192];
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    char resp[320];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"reason\":\"%s\"}",
             g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
             g_control_state.fan_speed,
             g_control_state.pump_speed,
             g_control_state.servo_angle,
             reason_esc);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

static esp_err_t api_history_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    char line[128];

    /* 发送 JSON 数组开头 */
    httpd_resp_sendstr_chunk(req, "[");

    /* 从最旧到最新遍历环形缓冲区 */
    int start = (history_count < HISTORY_MAX) ? 0 : history_head;
    for (int i = 0; i < history_count; i++) {
        int idx = (start + i) % HISTORY_MAX;
        const history_point_t *pt = &history_buf[idx];
        int len = snprintf(line, sizeof(line),
            "%s{\"t\":%lu,\"et\":%.1f,\"eh\":%.1f,\"st\":%.1f,\"sh\":%.1f,\"lx\":%.1f,\"ph\":%.2f}",
            (i > 0) ? "," : "",
            (unsigned long)pt->timestamp_s,
            pt->env_temp, pt->env_humi,
            pt->soil_temp, pt->soil_humi,
            pt->lux, pt->ph);
        httpd_resp_send_chunk(req, line, len);
    }

    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_send_chunk(req, NULL, 0);  /* 结束 chunked 响应 */
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t api_uri = {
            .uri       = "/api/data",
            .method    = HTTP_GET,
            .handler   = api_data_handler,
        };
        httpd_register_uri_handler(server, &api_uri);

        httpd_uri_t history_uri = {
            .uri       = "/api/history",
            .method    = HTTP_GET,
            .handler   = api_history_handler,
        };
        httpd_register_uri_handler(server, &history_uri);

        httpd_uri_t control_get_uri = {
            .uri       = "/api/control",
            .method    = HTTP_GET,
            .handler   = api_control_get_handler,
        };
        httpd_register_uri_handler(server, &control_get_uri);

        httpd_uri_t control_post_uri = {
            .uri       = "/api/control",
            .method    = HTTP_POST,
            .handler   = api_control_post_handler,
        };
        httpd_register_uri_handler(server, &control_post_uri);

        ESP_LOGI(TAG, "HTTP 服务器已启动");
    }
    return server;
}

/* ====== OneNet MQTT ====== */
static esp_mqtt_client_handle_t mqtt_client = NULL;

/* URL 编码: 仅处理 Base64 中的特殊字符 +/= */
static void url_encode_b64(const char *src, char *dst, size_t dst_size)
{
    size_t si = 0, di = 0;
    while (src[si] && di + 3 < dst_size) {
        char c = src[si++];
        if (c == '+')      { dst[di++]='%'; dst[di++]='2'; dst[di++]='B'; }
        else if (c == '/') { dst[di++]='%'; dst[di++]='2'; dst[di++]='F'; }
        else if (c == '=') { dst[di++]='%'; dst[di++]='3'; dst[di++]='D'; }
        else               { dst[di++] = c; }
    }
    dst[di] = '\0';
}

/* 生成 OneNet MQTT 鉴权 token */
static int onenet_gen_token(char *out, size_t out_size)
{
    /* 1) Base64 解码 device_key */
    unsigned char key_raw[64];
    size_t key_len = 0;
    if (mbedtls_base64_decode(key_raw, sizeof(key_raw), &key_len,
                              (const unsigned char *)ONENET_DEVICE_KEY,
                              strlen(ONENET_DEVICE_KEY)) != 0) {
        ESP_LOGE(TAG, "Base64 解码 device_key 失败");
        return -1;
    }

    /* 2) 构造 StringToSign: et\nmethod\nres\nversion */
    char sts[256];
    snprintf(sts, sizeof(sts), "%s\n%s\n%s\n%s",
             ONENET_ET, "sha256", ONENET_RES, "2018-10-31");

    /* 3) HMAC-SHA256 */
    unsigned char hmac[32];
    if (mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                        key_raw, key_len,
                        (const unsigned char *)sts, strlen(sts),
                        hmac) != 0) {
        ESP_LOGE(TAG, "HMAC-SHA256 计算失败");
        return -1;
    }

    /* 4) Base64 编码 HMAC */
    char hmac_b64[64];
    size_t b64_len = 0;
    if (mbedtls_base64_encode((unsigned char *)hmac_b64, sizeof(hmac_b64),
                              &b64_len, hmac, 32) != 0) {
        ESP_LOGE(TAG, "Base64 编码 HMAC 失败");
        return -1;
    }
    hmac_b64[b64_len] = '\0';

    /* 5) URL 编码 sign 和 res */
    char sign_enc[128], res_enc[128];
    url_encode_b64(hmac_b64, sign_enc, sizeof(sign_enc));
    url_encode_b64(ONENET_RES, res_enc, sizeof(res_enc));

    /* 6) 组装 token */
    snprintf(out, out_size,
             "version=2018-10-31&res=%s&et=%s&method=sha256&sign=%s",
             res_enc, ONENET_ET, sign_enc);

    ESP_LOGI(TAG, "OneNet token 已生成 (len=%d)", (int)strlen(out));
    ESP_LOGI(TAG, "Token: %s", out);
    return 0;
}

/* OneNet 回复 topic */
#define ONENET_TOPIC_REPLY  ONENET_TOPIC_POST "/reply"

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT 已连接 OneNet");
        mqtt_connected = true;
        mqtt_pending_send = true;  /* 重连后立即补发 */
        /* 订阅回复 topic，查看平台是否接受数据 */
        esp_mqtt_client_subscribe(mqtt_client, ONENET_TOPIC_REPLY, 0);
        ESP_LOGI(TAG, "已订阅: %s", ONENET_TOPIC_REPLY);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT 连接断开");
        mqtt_connected = false;
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT 消息已送达 broker, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT 收到回复 topic=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "MQTT 回复内容=%.*s", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR: {
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "MQTT 传输层错误");
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "MQTT 连接被拒绝, code=%d", event->error_handle->connect_return_code);
        }
        break;
    }
    default:
        break;
    }
}

static void mqtt_publish_data(void)
{
    if (!mqtt_connected) return;

    static uint32_t msg_id = 0;
    msg_id++;

    char payload[512];
    if (has_data) {
        snprintf(payload, sizeof(payload),
            "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{"
            "\"lux\":{\"value\":%.2f},"
            "\"env_temp\":{\"value\":%.1f},"
            "\"env_humi\":{\"value\":%.1f},"
            "\"press\":{\"value\":%.1f},"
            "\"soil_temp\":{\"value\":%.1f},"
            "\"soil_humi\":{\"value\":%.1f},"
            "\"ph\":{\"value\":%.2f},"
            "\"n\":{\"value\":%u},"
            "\"p\":{\"value\":%u},"
            "\"k\":{\"value\":%u}}}",
            (unsigned long)msg_id,
            latest_pkt.lux, latest_pkt.env_temp_c, latest_pkt.env_humi_pct,
            latest_pkt.press_kpa, latest_pkt.soil_temp_c, latest_pkt.soil_humi_pct,
            latest_pkt.ph, latest_pkt.n, latest_pkt.p, latest_pkt.k);
    } else {
        /* 无 LoRa 数据时发送测试数据，验证云平台通路 */
        snprintf(payload, sizeof(payload),
            "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{"
            "\"lux\":{\"value\":1234.56},"
            "\"env_temp\":{\"value\":25.5},"
            "\"env_humi\":{\"value\":60.0},"
            "\"press\":{\"value\":101.3},"
            "\"soil_temp\":{\"value\":22.0},"
            "\"soil_humi\":{\"value\":45.0},"
            "\"ph\":{\"value\":6.80},"
            "\"n\":{\"value\":28},"
            "\"p\":{\"value\":15},"
            "\"k\":{\"value\":30}}}",
            (unsigned long)msg_id);
        ESP_LOGW(TAG, "无 LoRa 数据，发送测试数据");
    }

    ESP_LOGI(TAG, "MQTT Topic: %s", ONENET_TOPIC_POST);
    ESP_LOGI(TAG, "MQTT Payload: %s", payload);
    int ret = esp_mqtt_client_publish(mqtt_client, ONENET_TOPIC_POST, payload, 0, 1, 0);
    ESP_LOGI(TAG, "MQTT 上报数据 id=%lu, ret=%d", (unsigned long)msg_id, ret);
}

static void mqtt_pub_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));  /* 启动后等5秒(确保MQTT已连接) */
    while (1) {
        if (mqtt_pending_send && mqtt_connected) {
            ESP_LOGI(TAG, "MQTT 重连补发数据");
            mqtt_publish_data();
            mqtt_pending_send = false;
        }
        mqtt_publish_data();
        vTaskDelay(pdMS_TO_TICKS(10000));  /* 每10秒上报一次 */
    }
}

static void mqtt_init(void)
{
    static char token[512];
    if (onenet_gen_token(token, sizeof(token)) != 0) {
        ESP_LOGE(TAG, "OneNet token 生成失败，MQTT 不启动");
        return;
    }

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri  = ONENET_BROKER_URL,
        .broker.address.port = 1883,
        .credentials.client_id = ONENET_DEVICE_NAME,
        .credentials.username  = ONENET_PRODUCT_ID,
        .credentials.authentication.password = token,
    };

    mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "MQTT 客户端已启动，正在连接 OneNet...");
    xTaskCreate(mqtt_pub_task, "mqtt_pub", 4096, NULL, 5, NULL);
}

/* ====== 硬件初始化 ====== */
static void a39c_mode_init(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(A39C_MD0_PIN));
    ESP_ERROR_CHECK(gpio_reset_pin(A39C_MD1_PIN));

    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << A39C_MD0_PIN) | (1ULL << A39C_MD1_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    ESP_ERROR_CHECK(gpio_set_level(A39C_MD0_PIN, A39C_MD0_LEVEL));
    ESP_ERROR_CHECK(gpio_set_level(A39C_MD1_PIN, A39C_MD1_LEVEL));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "模式脚设置: 目标 MD0=%d MD1=%d, 读回 MD0=%d MD1=%d",
             A39C_MD0_LEVEL,
             A39C_MD1_LEVEL,
             gpio_get_level(A39C_MD0_PIN),
             gpio_get_level(A39C_MD1_PIN));

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << A39C_AUX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));
}

static void uart_init_a39c(void)
{
    const uart_config_t cfg = {
        .baud_rate = A39C_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(A39C_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(A39C_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(A39C_UART_NUM, A39C_TX_PIN, A39C_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART init done: %d 8N1", A39C_BAUD);
}

static void check_alerts(const sensor_packet_t *pkt, char *alert_buf, size_t buf_size);

/* ====== LoRa 接收任务 ====== */
static void lora_rx_task(void *arg)
{
    uint8_t buf[FRAME_TOTAL_LEN];
    int state = 0;   /* 0: 找AA 1: 找55 2: 收剩余 */
    int idx = 0;
    TickType_t last_report_tick = xTaskGetTickCount();
    uint32_t last_rx_bytes = 0;
    uint32_t last_aux_low_count = 0;

    while (1) {
        if (gpio_get_level(A39C_AUX_PIN) == 0) {
            aux_low_count++;
        }

        uint8_t b;
        int n = uart_read_bytes(A39C_UART_NUM, &b, 1, pdMS_TO_TICKS(100));
        if (n <= 0) {
            TickType_t now_tick = xTaskGetTickCount();
            if ((now_tick - last_report_tick) >= pdMS_TO_TICKS(5000)) {
                uint32_t delta_rx = rx_bytes - last_rx_bytes;
                uint32_t delta_aux_low = aux_low_count - last_aux_low_count;
                ESP_LOGI(TAG, "5秒统计: RX字节=%lu, OK=%lu, FAIL=%lu",
                         (unsigned long)rx_bytes,
                         (unsigned long)ok_count,
                         (unsigned long)fail_count);
                ESP_LOGI(TAG, "5秒增量: RX新增=%lu, AUX低电平采样=%lu, 当前AUX=%d",
                         (unsigned long)delta_rx,
                         (unsigned long)delta_aux_low,
                         gpio_get_level(A39C_AUX_PIN));

                if (delta_rx == 0 && delta_aux_low == 0) {
                    ESP_LOGW(TAG, "未见AUX活动，接收模块可能没有收到空中数据，请检查信道/地址/空口速率");
                } else if (delta_rx == 0 && delta_aux_low > 0) {
                    ESP_LOGW(TAG, "AUX有活动但UART无字节，请检查A39C TXD->ESP32-S3 GPIO%d 以及模块串口参数", A39C_RX_PIN);
                }
                last_report_tick = now_tick;
                last_rx_bytes = rx_bytes;
                last_aux_low_count = aux_low_count;
            }
            continue;
        }
        rx_bytes += (uint32_t)n;

        if (state == 0) {
            if (b == FRAME_HEAD1) {
                buf[0] = b;
                state = 1;
            }
        } else if (state == 1) {
            if (b == FRAME_HEAD2) {
                buf[1] = b;
                idx = 2;
                state = 2;
            } else if (b == FRAME_HEAD1) {
                buf[0] = b;
                state = 1;
            } else {
                state = 0;
            }
        } else {
            buf[idx++] = b;
            if (idx >= FRAME_TOTAL_LEN) {
                sensor_packet_t pkt;
                if (parse_frame(buf, &pkt)) {
                    ok_count++;
                    log_packet(&pkt);
                    /* 更新最新数据供网页读取 */
                    latest_pkt = pkt;
                    has_data = true;
                    history_push(&pkt);
                    if (g_control_mode == CONTROL_MODE_SMART) {
                        g_smart_control_force_run = true;
                    }
                    /* 实时刷新显示屏 */
                    {
                        display_sensor_data_t dd = {
                            .lux          = pkt.lux,
                            .env_temp_c   = pkt.env_temp_c,
                            .env_humi_pct = pkt.env_humi_pct,
                            .press_kpa    = pkt.press_kpa,
                            .soil_temp_c  = pkt.soil_temp_c,
                            .soil_humi_pct = pkt.soil_humi_pct,
                            .ph = pkt.ph,
                            .n = pkt.n, .p = pkt.p, .k = pkt.k,
                        };
                        char alert[64] = "";
                        check_alerts(&pkt, alert, sizeof(alert));
                        lcd_update(&dd, true, PLANT_SPECIES_EN,
                                   mqtt_connected, mqtt_connected, alert);
                    }
                } else {
                    fail_count++;
                    ESP_LOGW(TAG, "Frame invalid. OK=%lu FAIL=%lu 原始帧=",
                             (unsigned long)ok_count, (unsigned long)fail_count);
                    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, FRAME_TOTAL_LEN, ESP_LOG_WARN);
                }
                state = 0;
                idx = 0;
            }
        }
    }
}

/* ====== AI 语音播报 ====== */
static max98357a_handle_t *g_audio_handle = NULL;
static spark_chat_client_t g_spark = {0};
static spark_chat_client_t g_smart_ctrl = {0};
static baidu_tts_handle_t  g_tts = {0};

/* ====== 异常告警阈值（通用基础值，AI会根据品种联网搜索后细化判断） ====== */
#define ALERT_TEMP_HIGH      40.0f   /* 高温上限 ℃ */
#define ALERT_TEMP_LOW        2.0f   /* 低温下限 ℃ */
#define ALERT_HUMI_HIGH      95.0f   /* 环境湿度过高 % */
#define ALERT_HUMI_LOW       20.0f   /* 环境湿度过低 % */
#define ALERT_SOIL_HUMI_LOW  15.0f   /* 土壤过干 % */
#define ALERT_SOIL_HUMI_HIGH 90.0f   /* 土壤过湿 % */
#define ALERT_SOIL_TEMP_HIGH 40.0f   /* 土壤温度过高 ℃ */
#define ALERT_PH_LOW          3.5f   /* pH过酸 */
#define ALERT_PH_HIGH         9.0f   /* pH过碱 */
#define ALERT_LUX_HIGH    120000.0f  /* 光照过强 lux */

#define NORMAL_REPORT_INTERVAL_S  300   /* 正常时5分钟播报一次 */
#define ALERT_REPORT_INTERVAL_S    60   /* 异常时1分钟播报一次 */
#define SMART_CONTROL_INTERVAL_S   30    /* 智能控制评估周期 */

static bool parse_control_json_from_text(const char *text, int *fan, int *pump, int *servo, char *reason, size_t reason_size)
{
    if (!text || !fan || !pump || !servo) {
        return false;
    }

    const char *start = strchr(text, '{');
    const char *end = strrchr(text, '}');
    if (!start || !end || end <= start) {
        return false;
    }

    size_t len = (size_t)(end - start + 1);
    if (len >= 512) {
        len = 511;
    }
    char json_buf[512];
    memcpy(json_buf, start, len);
    json_buf[len] = '\0';

    cJSON *root = cJSON_Parse(json_buf);
    if (!root) {
        return false;
    }

    const cJSON *fan_j = cJSON_GetObjectItem(root, "fan");
    const cJSON *pump_j = cJSON_GetObjectItem(root, "pump");
    const cJSON *servo_j = cJSON_GetObjectItem(root, "servo");
    const cJSON *reason_j = cJSON_GetObjectItem(root, "reason");

    bool ok = fan_j && cJSON_IsNumber(fan_j)
              && pump_j && cJSON_IsNumber(pump_j)
              && servo_j && cJSON_IsNumber(servo_j);

    if (ok) {
        *fan = fan_j->valueint;
        *pump = pump_j->valueint;
        *servo = servo_j->valueint;
        if (reason && reason_size > 0) {
            if (reason_j && cJSON_IsString(reason_j) && reason_j->valuestring) {
                snprintf(reason, reason_size, "%s", reason_j->valuestring);
            } else {
                snprintf(reason, reason_size, "smart adjust");
            }
        }
    }

    cJSON_Delete(root);
    return ok;
}

static void build_smart_control_prompt(char *buf, size_t size)
{
    snprintf(buf, size,
        "你是%s温室控制器。根据当前数据直接给出执行器控制量。"
        "输出必须是纯JSON，且只有这些字段："
        "{\"fan\":0-100,\"pump\":0-100,\"servo\":0-180,\"reason\":\"<=20字\"}。"
        "不要输出markdown和多余文本。"
        "当前数据: 光照%.0f lx, 环境温度%.1f℃, 环境湿度%.0f%%, 土壤温度%.1f℃, 土壤湿度%.0f%%, pH%.2f, N%u, P%u, K%u。"
        "控制逻辑示例: 高温或强光可增大风扇并调整舵机遮阳; 土壤偏干提高水泵。",
        PLANT_SPECIES,
        latest_pkt.lux, latest_pkt.env_temp_c, latest_pkt.env_humi_pct,
        latest_pkt.soil_temp_c, latest_pkt.soil_humi_pct, latest_pkt.ph,
        latest_pkt.n, latest_pkt.p, latest_pkt.k);
}

/* 检测数据是否异常，返回告警描述(空字符串=正常) */
static void check_alerts(const sensor_packet_t *pkt, char *alert_buf, size_t buf_size)
{
    alert_buf[0] = '\0';
    int offset = 0;

    if (pkt->env_temp_c > ALERT_TEMP_HIGH) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "环境温度过高%.1f℃！", pkt->env_temp_c);
    }
    if (pkt->env_temp_c < ALERT_TEMP_LOW) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "环境温度过低%.1f℃！", pkt->env_temp_c);
    }
    if (pkt->env_humi_pct > ALERT_HUMI_HIGH) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "环境湿度过高%.0f%%！", pkt->env_humi_pct);
    }
    if (pkt->env_humi_pct < ALERT_HUMI_LOW) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "环境湿度过低%.0f%%！", pkt->env_humi_pct);
    }
    if (pkt->soil_humi_pct < ALERT_SOIL_HUMI_LOW) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "土壤湿度仅%.0f%%需浇水！", pkt->soil_humi_pct);
    }
    if (pkt->soil_humi_pct > ALERT_SOIL_HUMI_HIGH) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "土壤湿度%.0f%%过高！", pkt->soil_humi_pct);
    }
    if (pkt->soil_temp_c > ALERT_SOIL_TEMP_HIGH) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "土壤温度过高%.1f℃！", pkt->soil_temp_c);
    }
    if (pkt->ph < ALERT_PH_LOW) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "土壤pH%.2f偏酸！", pkt->ph);
    }
    if (pkt->ph > ALERT_PH_HIGH) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "土壤pH%.2f偏碱！", pkt->ph);
    }
    if (pkt->lux > ALERT_LUX_HIGH) {
        offset += snprintf(alert_buf + offset, buf_size - offset,
            "光照过强%.0flux！", pkt->lux);
    }
}

static void ai_audio_init(void)
{
    /* 初始化 MAX98357A 音频输出 */
    ESP_LOGI(TAG, "初始化音频输出 MAX98357A...");
    max98357a_config_t audio_cfg = max98357a_get_default_config();
    audio_cfg.i2s_port     = I2S_NUM_0;
    audio_cfg.bclk_pin     = AUDIO_BCLK_PIN;
    audio_cfg.lrclk_pin    = AUDIO_LRCLK_PIN;
    audio_cfg.din_pin      = AUDIO_DIN_PIN;
    audio_cfg.sd_mode_pin  = AUDIO_SD_MODE_PIN;
    audio_cfg.sample_rate  = 16000;
    audio_cfg.bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT;
    audio_cfg.gain         = MAX98357A_GAIN_9DB;
    audio_cfg.channel      = MAX98357A_CHANNEL_LEFT;

    esp_err_t ret = max98357a_init(&audio_cfg, &g_audio_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX98357A 初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    /* 初始化豆包大模型 */
    ESP_LOGI(TAG, "初始化 AI 大模型...");
    spark_chat_config_t spark_cfg = {
        .api_key  = AI_API_KEY,
        .url      = AI_URL,
        .user_id  = NULL,
        .model    = AI_MODEL,
        .timeout_ms = 30000,
        .stream   = true,
        .enable_web_search = true,
        .search_mode = "auto",
    };
    spark_chat_init(&g_spark, &spark_cfg);

    spark_chat_config_t ctrl_cfg = spark_cfg;
    ctrl_cfg.enable_web_search = false;
    spark_chat_init(&g_smart_ctrl, &ctrl_cfg);
    spark_chat_add_message(&g_smart_ctrl, "system",
        "你是温室执行器控制助手。根据实时环境数据输出JSON控制量。"
        "禁止解释，禁止多余文本，只能输出JSON。"
        "fan和pump范围0-100，servo范围0-180。");

    /* 动态生成 system prompt：根据 PLANT_SPECIES 让 AI 联网搜索适生条件 */
    char sys_prompt[512];
    snprintf(sys_prompt, sizeof(sys_prompt),
        "你是%s种植智慧农业助手。监测对象是%s。"
        "请联网搜索%s的最佳种植条件，"
        "根据搜索结果直接分析传感器数据。"
        "如有异常给出针对%s的建议。"
        "回答极简，不超过50字，不要客套。",
        PLANT_SPECIES, PLANT_SPECIES, PLANT_SPECIES, PLANT_SPECIES);
    spark_chat_add_message(&g_spark, "system", sys_prompt);

    /* 初始化百度 TTS */
    ESP_LOGI(TAG, "初始化百度 TTS...");
    baidu_tts_config_t tts_cfg = {
        .api_key    = BAIDU_API_KEY,
        .secret_key = BAIDU_SECRET_KEY,
        .voice      = BAIDU_TTS_VOICE_FEMALE,
        .speed      = 5,
        .pitch      = 5,
        .volume     = 15,
        .timeout_ms = 30000,
    };
    baidu_tts_init(&g_tts, &tts_cfg, g_audio_handle);

    /* 放大器上电后一次性 enable，此后保持常开，避免频繁 SD_MODE 切换产生噪声 */
    max98357a_enable(g_audio_handle);

    ESP_LOGI(TAG, "AI 语音播报模块初始化完成");
}

/* 构建传感器数据摘要发送给 AI */
static void build_sensor_prompt(char *buf, size_t size, bool is_alert, const char *alerts)
{
    if (has_data) {
        int n = snprintf(buf, size,
            "数据：光照%.0f lx，气温%.1f℃，湿度%.0f%%，"
            "气压%.0f，土温%.1f℃，土湿%.0f%%，"
            "pH%.1f，N%u P%u K%u mg/kg。",
            latest_pkt.lux, latest_pkt.env_temp_c, latest_pkt.env_humi_pct,
            latest_pkt.press_kpa, latest_pkt.soil_temp_c, latest_pkt.soil_humi_pct,
            latest_pkt.ph, latest_pkt.n, latest_pkt.p, latest_pkt.k);
        if (is_alert && alerts[0]) {
            snprintf(buf + n, size - n,
                "【告警】%s 请分析并建议。", alerts);
        } else {
            snprintf(buf + n, size - n,
                "简评当前环境。");
        }
    } else {
        snprintf(buf, size, "暂无传感器数据，提醒检查设备。");
    }
}

/* AI 分析 + 语音播报任务 */
static void ai_report_task(void *arg)
{
    /* 等待系统完全启动 */
    vTaskDelay(pdMS_TO_TICKS(10000));

    /* 上电自我介绍 */
    ESP_LOGI(TAG, "===== 播放自我介绍 =====");
    {
        char intro[128];
        snprintf(intro, sizeof(intro),
            "%s智慧监测系统已启动，开始为您实时分析环境数据。",
            PLANT_SPECIES);
        esp_err_t intro_ret = ESP_FAIL;
        for (int i = 0; i < 3; i++) {
            intro_ret = baidu_tts_speak(&g_tts, intro);
            if (intro_ret == ESP_OK) {
                break;
            }
            ESP_LOGW(TAG, "开场播报失败，重试 %d/3: %s", i + 1, esp_err_to_name(intro_ret));
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        if (intro_ret != ESP_OK) {
            ESP_LOGE(TAG, "开场播报最终失败: %s", esp_err_to_name(intro_ret));
        }
    }

    /* 等待3秒再开始定时播报（缩短延迟，加速首次检测） */
    vTaskDelay(pdMS_TO_TICKS(3000));

    int normal_elapsed_s = ALERT_REPORT_INTERVAL_S;  /* 初始化为告警间隔，确保立即触发首次检测 */
    const int check_interval_s = 10;  /* 每10秒检测一次告警 */

    while (1) {

        if (!has_data) {
            /* 没有传感器数据则等待后重试 */
            vTaskDelay(pdMS_TO_TICKS(check_interval_s * 1000));
            normal_elapsed_s += check_interval_s;
            continue;
        }

        /* 检测异常 */
        char alert_buf[256];
        check_alerts(&latest_pkt, alert_buf, sizeof(alert_buf));
        bool is_alert = (alert_buf[0] != '\0');

        /* 决定是否播报：异常立即播报(最低间隔60s)，正常每5分钟播报 */
        bool should_report = false;
        if (is_alert && normal_elapsed_s >= ALERT_REPORT_INTERVAL_S) {
            should_report = true;
            ESP_LOGW(TAG, "检测到异常: %s", alert_buf);
        } else if (normal_elapsed_s >= NORMAL_REPORT_INTERVAL_S) {
            should_report = true;
        }

        if (!should_report) {
            vTaskDelay(pdMS_TO_TICKS(check_interval_s * 1000));
            normal_elapsed_s += check_interval_s;
            continue;
        }

        normal_elapsed_s = 0;  /* 重置计时 */

        ESP_LOGI(TAG, "===== AI 语音播报开始 (异常=%d) =====", is_alert);
        ESP_LOGI(TAG, "空闲堆内存: %lu bytes", (unsigned long)esp_get_free_heap_size());

        /* 1) 构建传感器数据提示词 */
        char prompt[512];
        build_sensor_prompt(prompt, sizeof(prompt), is_alert, alert_buf);
        ESP_LOGI(TAG, "AI 提示词: %s", prompt);

        /* 2) 发送给大模型 */
        spark_chat_add_message(&g_spark, "user", prompt);
        spark_chat_trim_history(&g_spark);

        bool ok = spark_chat_request(&g_spark);
        /* 每次请求后释放HTTP连接，避免复用过期连接导致下次收不到数据 */
        spark_chat_close_connection(&g_spark);

        if (ok) {
            const char *response = spark_chat_get_last_response(&g_spark);
            ESP_LOGI(TAG, "AI 回复: %s", response);

            /* 把 AI 回复加入对话历史 */
            spark_chat_add_message(&g_spark, "assistant", response);

            /* 3) TTS 语音播放 */
            if (strlen(response) > 0) {
                ESP_LOGI(TAG, "TTS 播放中...");
                esp_err_t ret = baidu_tts_speak(&g_tts, response);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "TTS 播放失败: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGW(TAG, "AI 回复为空，跳过TTS");
            }
        } else {
            ESP_LOGW(TAG, "AI 请求失败");
        }

        int next = is_alert ? ALERT_REPORT_INTERVAL_S : NORMAL_REPORT_INTERVAL_S;
        ESP_LOGI(TAG, "===== AI 播报结束，%d 秒后下次检查 =====", next);

        /* 延时放在循环末尾，首次能立即检测 */
        vTaskDelay(pdMS_TO_TICKS(check_interval_s * 1000));
        normal_elapsed_s += check_interval_s;
    }
}

/* 智能模式：根据大模型分析自动调节风扇/水泵/舵机 */
static void ai_smart_control_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(12000));
    uint32_t last_run_s = 0;

    while (1) {
        uint32_t now_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        bool interval_due = (now_s - last_run_s) >= SMART_CONTROL_INTERVAL_S;

        if (!has_data || g_control_mode != CONTROL_MODE_SMART) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!g_smart_control_force_run && !interval_due) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        g_smart_control_force_run = false;
        last_run_s = now_s;

        char prompt[640];
        build_smart_control_prompt(prompt, sizeof(prompt));

        spark_chat_add_message(&g_smart_ctrl, "user", prompt);
        spark_chat_trim_history(&g_smart_ctrl);

        bool ok = spark_chat_request(&g_smart_ctrl);
        spark_chat_close_connection(&g_smart_ctrl);

        if (!ok) {
            ESP_LOGW(TAG, "智能控制AI请求失败");
            vTaskDelay(pdMS_TO_TICKS(SMART_CONTROL_INTERVAL_S * 1000));
            continue;
        }

        const char *resp = spark_chat_get_last_response(&g_smart_ctrl);
        int fan = 0, pump = 0, servo = 90;
        char reason[96] = "smart adjust";

        if (parse_control_json_from_text(resp, &fan, &pump, &servo, reason, sizeof(reason))) {
            apply_control_and_send(CONTROL_MODE_SMART, fan, pump, servo, reason);
            ESP_LOGI(TAG, "智能控制生效 fan=%d pump=%d servo=%d reason=%s", fan, pump, servo, reason);
        } else {
            ESP_LOGW(TAG, "智能控制解析失败: %s", resp ? resp : "(null)");
        }
    }
}

void app_main(void)
{
    /* NVS 初始化（WiFi 需要） */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 先初始化 LoRa 接收硬件 */
    a39c_mode_init();
    uart_init_a39c();

    /* 启动 WiFi STA（连接路由器） */
    wifi_init_sta();

    /* 启动 HTTP 服务器 */
    start_webserver();

    /* 启动 MQTT 连接 OneNet */
    mqtt_init();

    ESP_LOGI(TAG, "A39C RX start, MD0=%d MD1=%d AUX=%d",
             gpio_get_level(A39C_MD0_PIN),
             gpio_get_level(A39C_MD1_PIN),
             gpio_get_level(A39C_AUX_PIN));

    /* 初始化 ST7735S 显示屏（须在 LoRa 任务之前，否则 lcd_update 使用未初始化的 SPI） */
    if (lcd_init() == ESP_OK) {
        lcd_update(NULL, false, PLANT_SPECIES_EN, false, false, "");
    }

    /* 在独立任务中运行 LoRa 接收循环 */
    ESP_LOGI(TAG, "空闲堆内存: %lu bytes", (unsigned long)esp_get_free_heap_size());
    xTaskCreate(lora_rx_task, "lora_rx", 8192, NULL, 10, NULL);

    /* 初始化 AI 语音播报并启动任务 */
    ai_audio_init();
    xTaskCreate(ai_report_task, "ai_report", 16384, NULL, 3, NULL);
    xTaskCreate(ai_smart_control_task, "ai_smart_ctl", 16384, NULL, 3, NULL);

    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  智慧农业监测系统已启动");
    ESP_LOGI(TAG, "  LoRa + OneNet + AI语音播报");
    ESP_LOGI(TAG, "============================================");
}
