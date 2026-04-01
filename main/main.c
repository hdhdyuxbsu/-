#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
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
#include "esp_heap_caps.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "mbedtls/md.h"
#include "mbedtls/base64.h"
#include "mbedtls/platform.h"

/* AI 语音播报相关 */
#include "ai_config.h"
#include "doubao_chat.h"
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
#define WIFI_MAX_TX_POWER_QDBM 52         /* 13dBm, 降低峰值电流 */

/* ====== OneNet MQTT 配置 ====== */
#define ONENET_PRODUCT_ID   "W9BY3SlO8w"
#define ONENET_DEVICE_NAME  "dachuang"
#define ONENET_DEVICE_KEY   "aDFIWDNUeFpLTTdUTzdCS1VzdWlyV3JLS3pnR2tkdVU="
#define ONENET_BROKER_URL   "mqtt://mqtts.heclouds.com"
#define ONENET_ET           "4102444800"  /* 2100-01-01, token 长期有效 */
#define ONENET_RES          "products/" ONENET_PRODUCT_ID "/devices/" ONENET_DEVICE_NAME
#define ONENET_TOPIC_POST   "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/post"
#define ONENET_TOPIC_SET    "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/set"  /* 远程控制 */

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
#define A39C_DIAG_READ_CONFIG_ON_BOOT 1
#define AUDIO_SELF_TEST_ON_BOOT 0

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
    uint8_t light_level;  /* 0-100 补光灯亮度 */
    uint32_t update_time_s;
    char reason[96];
} control_state_t;

static uint32_t ok_count = 0;
static uint32_t fail_count = 0;
static uint32_t rx_bytes = 0;
static uint32_t aux_low_count = 0;
static uint32_t rx_pin_low_count = 0;

/* 前置声明：跨模块共享状态 */
static bool mqtt_connected = false;
static bool mqtt_pending_send = false;

/* 控制模式与执行器状态 */
static control_mode_t g_control_mode = CONTROL_MODE_SMART;
static control_state_t g_control_state = {
    .fan_speed = 0,
    .pump_speed = 0,
    .servo_angle = 90,
    .light_level = 0,
    .update_time_s = 0,
    .reason = "init",
};
static volatile bool g_smart_control_force_run = false;
static char g_last_alert[256] = "";
static char g_last_ai_reply[256] = "";
static volatile bool g_auto_stop_sent = false;
static char g_monitor_plant[64] = PLANT_SPECIES_EN;

/*
 * 控制命令协议 V2 (ESP32 -> STM32)
 * Byte0  : 0xA5
 * Byte1  : 0x5A
 * Byte2  : ver      (0x01)
 * Byte3  : cmd      (0x10=set control)
 * Byte4  : seq      (递增序号)
 * Byte5  : mode     (0x01 manual, 0x02 smart)
 * Byte6  : fan      (0-100)
 * Byte7  : pump     (0-100)
 * Byte8  : servo    (0-180)
 * Byte9  : light    (0-100, 补光灯亮度)
 * Byte10 : XOR checksum of Byte0..Byte9
 */
/*
 * 控制应答协议 V2 (STM32 -> ESP32)
 * Byte0  : 0x5A
 * Byte1  : 0xA5
 * Byte2  : ver      (0x01)
 * Byte3  : cmd      (0x90=ack)
 * Byte4  : seq      (回显序号)
 * Byte5  : status   (bit0=fAN_ON, bit1=PUMP_ON, bit7=APPLIED_OK)
 * Byte6  : fan_actual
 * Byte7  : pump_actual
 * Byte8  : servo_actual
 * Byte9  : err_code
 * Byte10 : XOR checksum of Byte0..Byte9
 */
#define CONTROL_FRAME_HEAD1 0xA5
#define CONTROL_FRAME_HEAD2 0x5A
#define CONTROL_ACK_HEAD1   0x5A
#define CONTROL_ACK_HEAD2   0xA5
#define CONTROL_FRAME_VER   0x01
#define CONTROL_CMD_SET     0x10
#define CONTROL_CMD_ACK     0x90
#define CONTROL_FRAME_LEN   11
#define CONTROL_RETRY_INTERVAL_S 10
#define SMART_FAN_STEP_MAX  10
#define SMART_PUMP_STEP_MAX 15

typedef struct {
    bool pending;
    uint8_t seq;
    uint8_t frame[CONTROL_FRAME_LEN];
    int64_t last_send_us;
    uint32_t retry_count;
} control_tx_state_t;

typedef struct {
    bool valid;
    uint8_t seq;
    bool applied_ok;
    bool fan_on;
    bool pump_on;
    uint8_t fan_actual;
    uint8_t pump_actual;
    uint8_t servo_actual;
    uint8_t err_code;
    uint32_t recv_time_s;
} control_ack_state_t;

static control_tx_state_t g_ctrl_tx = {0};
static control_ack_state_t g_ctrl_ack = {0};
static uint8_t g_ctrl_seq = 0;

static uint8_t clamp_u8(int val, int min_v, int max_v)
{
    if (val < min_v) return (uint8_t)min_v;
    if (val > max_v) return (uint8_t)max_v;
    return (uint8_t)val;
}

static uint8_t limit_step_u8(uint8_t current, uint8_t target, uint8_t max_step)
{
    if (target > current) {
        uint16_t next = (uint16_t)current + max_step;
        return (next < target) ? (uint8_t)next : target;
    }
    if (target < current) {
        int next = (int)current - (int)max_step;
        return (next > target) ? (uint8_t)next : target;
    }
    return target;
}

static void trim_text_inplace(char *s)
{
    if (!s) {
        return;
    }

    size_t len = strlen(s);
    size_t start = 0;
    while (start < len && (s[start] == ' ' || s[start] == '\t' || s[start] == '\r' || s[start] == '\n')) {
        start++;
    }

    size_t end = len;
    while (end > start && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r' || s[end - 1] == '\n')) {
        end--;
    }

    if (start > 0) {
        memmove(s, s + start, end - start);
    }
    s[end - start] = '\0';
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

static uint8_t calc_xor(const uint8_t *buf, size_t len)
{
    uint8_t x = 0;
    for (size_t i = 0; i < len; i++) {
        x ^= buf[i];
    }
    return x;
}

static void send_control_frame_raw(const uint8_t *frame, const char *tag)
{
    int written = uart_write_bytes(A39C_UART_NUM, frame, CONTROL_FRAME_LEN);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame, CONTROL_FRAME_LEN, ESP_LOG_INFO);
    ESP_LOGI(TAG, "%s 下发控制帧 bytes=%d", tag, written);
}

static void send_control_frame_to_stm32(control_mode_t mode, uint8_t fan, uint8_t pump, uint8_t servo, uint8_t light)
{
    uint8_t frame[CONTROL_FRAME_LEN] = {0};
    frame[0] = CONTROL_FRAME_HEAD1;
    frame[1] = CONTROL_FRAME_HEAD2;
    frame[2] = CONTROL_FRAME_VER;
    frame[3] = CONTROL_CMD_SET;
    frame[4] = ++g_ctrl_seq;
    frame[5] = (mode == CONTROL_MODE_SMART) ? 0x02 : 0x01;
    frame[6] = fan;
    frame[7] = pump;
    frame[8] = servo;
    frame[9] = light;  /* 补光灯亮度 0-100 */
    frame[10] = calc_xor(frame, CONTROL_FRAME_LEN - 1);

    memcpy(g_ctrl_tx.frame, frame, CONTROL_FRAME_LEN);
    g_ctrl_tx.pending = true;
    g_ctrl_tx.seq = frame[4];
    g_ctrl_tx.retry_count = 0;
    g_ctrl_tx.last_send_us = esp_timer_get_time();

    send_control_frame_raw(frame, "CTRL");
    ESP_LOGI(TAG, "下发控制帧V2 seq=%u mode=%s fan=%u pump=%u servo=%u light=%u，等待STM32 ACK",
             g_ctrl_tx.seq,
             mode == CONTROL_MODE_SMART ? "smart" : "manual",
             fan, pump, servo, light);
}

static bool parse_control_ack_frame(const uint8_t *frame)
{
    if (!frame) {
        return false;
    }
    if (frame[0] != CONTROL_ACK_HEAD1 || frame[1] != CONTROL_ACK_HEAD2) {
        return false;
    }
    if (frame[2] != CONTROL_FRAME_VER || frame[3] != CONTROL_CMD_ACK) {
        return false;
    }
    uint8_t x = calc_xor(frame, CONTROL_FRAME_LEN - 1);
    if (x != frame[CONTROL_FRAME_LEN - 1]) {
        ESP_LOGW(TAG, "控制ACK校验失败");
        return false;
    }

    uint8_t seq = frame[4];
    uint8_t status = frame[5];
    g_ctrl_ack.valid = true;
    g_ctrl_ack.seq = seq;
    g_ctrl_ack.applied_ok = ((status & 0x80) != 0);
    g_ctrl_ack.fan_on = ((status & 0x01) != 0);
    g_ctrl_ack.pump_on = ((status & 0x02) != 0);
    g_ctrl_ack.fan_actual = frame[6];
    g_ctrl_ack.pump_actual = frame[7];
    g_ctrl_ack.servo_actual = frame[8];
    g_ctrl_ack.err_code = frame[9];
    g_ctrl_ack.recv_time_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    ESP_LOGI(TAG, "收到STM32 ACK seq=%u ok=%d fan_on=%d pump_on=%d fan=%u pump=%u servo=%u err=%u",
             seq,
             g_ctrl_ack.applied_ok,
             g_ctrl_ack.fan_on,
             g_ctrl_ack.pump_on,
             g_ctrl_ack.fan_actual,
             g_ctrl_ack.pump_actual,
             g_ctrl_ack.servo_actual,
             g_ctrl_ack.err_code);

    if (g_ctrl_tx.pending && seq == g_ctrl_tx.seq) {
        g_ctrl_tx.pending = false;
        ESP_LOGI(TAG, "控制指令 seq=%u 已确认，停止重发", seq);
    }
    return true;
}

static void apply_control_and_send(control_mode_t mode, int fan_speed, int pump_speed,
                                   int servo_angle, int light_level, const char *reason)
{
    uint8_t new_fan = clamp_u8(fan_speed, 0, 100);
    uint8_t new_pump = clamp_u8(pump_speed, 0, 100);
    uint8_t new_servo = clamp_u8(servo_angle, 0, 180);
    uint8_t new_light = clamp_u8(light_level, 0, 100);

    if (mode == CONTROL_MODE_SMART) {
        uint8_t old_fan = g_control_state.fan_speed;
        uint8_t old_pump = g_control_state.pump_speed;

        new_fan = limit_step_u8(old_fan, new_fan, SMART_FAN_STEP_MAX);
        new_pump = limit_step_u8(old_pump, new_pump, SMART_PUMP_STEP_MAX);

        if (new_fan != (uint8_t)fan_speed) {
            ESP_LOGI(TAG, "风扇斜坡限制: raw=%d limited=%u prev=%u max_step=%d",
                     fan_speed, new_fan, old_fan, SMART_FAN_STEP_MAX);
        }
        if (new_pump != (uint8_t)pump_speed) {
            ESP_LOGI(TAG, "泵速斜坡限制: raw=%d limited=%u prev=%u max_step=%d",
                     pump_speed, new_pump, old_pump, SMART_PUMP_STEP_MAX);
        }
    }

    g_control_state.fan_speed = new_fan;
    g_control_state.pump_speed = new_pump;
    g_control_state.servo_angle = new_servo;
    g_control_state.light_level = new_light;
    g_control_state.update_time_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (reason && reason[0]) {
        snprintf(g_control_state.reason, sizeof(g_control_state.reason), "%s", reason);
    }
    g_control_mode = mode;

    if (!(g_control_state.fan_speed == 0 &&
          g_control_state.pump_speed == 0 &&
          g_control_state.servo_angle == 90 &&
          g_control_state.light_level == 0)) {
        g_auto_stop_sent = false;
    }

    send_control_frame_to_stm32(g_control_mode,
                                g_control_state.fan_speed,
                                g_control_state.pump_speed,
                                g_control_state.servo_angle,
                                g_control_state.light_level);
}

static void control_retry_task(void *arg)
{
    while (1) {
        if (g_ctrl_tx.pending) {
            int64_t now_us = esp_timer_get_time();
            int64_t elapsed_us = now_us - g_ctrl_tx.last_send_us;
            if (elapsed_us >= (int64_t)CONTROL_RETRY_INTERVAL_S * 1000000LL) {
                send_control_frame_raw(g_ctrl_tx.frame, "CTRL-RETRY");
                g_ctrl_tx.last_send_us = now_us;
                g_ctrl_tx.retry_count++;
                ESP_LOGW(TAG, "未收到STM32 ACK，10秒重发控制帧 seq=%u retry=%lu",
                         g_ctrl_tx.seq,
                         (unsigned long)g_ctrl_tx.retry_count);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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

    // 降低WiFi发射功率，减少上电阶段电流尖峰触发Brownout
    esp_err_t txp_ret = esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    if (txp_ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi最大发射功率已设置为 %.1f dBm", WIFI_MAX_TX_POWER_QDBM / 4.0f);
    } else {
        ESP_LOGW(TAG, "设置WiFi发射功率失败: %s", esp_err_to_name(txp_ret));
    }

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
static esp_err_t api_plant_post_handler(httpd_req_t *req);

static esp_err_t root_get_handler(httpd_req_t *req)
{
    size_t len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, len);
    return ESP_OK;
}

static void set_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t api_options_handler(httpd_req_t *req)
{
    set_cors_headers(req);
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void *tls_calloc_prefer_psram(size_t n, size_t size)
{
    if (n == 0 || size == 0) {
        return NULL;
    }
    if (size > SIZE_MAX / n) {
        return NULL;
    }

    void *ptr = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) {
        ptr = heap_caps_calloc(n, size, MALLOC_CAP_8BIT);
    }
    return ptr;
}

static void tls_free_prefer_psram(void *ptr)
{
    heap_caps_free(ptr);
}

static void init_mbedtls_allocator(void)
{
    int ret = mbedtls_platform_set_calloc_free(tls_calloc_prefer_psram, tls_free_prefer_psram);
    if (ret == 0) {
        ESP_LOGI(TAG, "mbedTLS allocator set to prefer PSRAM");
    } else {
        ESP_LOGW(TAG, "mbedTLS allocator setup failed: -0x%04X", -ret);
    }
    ESP_LOGI(TAG, "Heap snapshot: total=%lu internal=%lu psram=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static esp_err_t api_data_handler(httpd_req_t *req)
{
    char buf[1280];
    char plant_esc[128];
    char reason_esc[256];
    char alert_esc[320];
    char ai_reply_esc[320];
    json_escape_text(g_monitor_plant, plant_esc, sizeof(plant_esc));
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    json_escape_text(g_last_alert, alert_esc, sizeof(alert_esc));
    json_escape_text(g_last_ai_reply, ai_reply_esc, sizeof(ai_reply_esc));
    if (has_data) {
    snprintf(buf, sizeof(buf),
            "{\"seq\":%u,\"lux\":%.2f,\"env_temp\":%.1f,\"env_humi\":%.1f,"
            "\"press\":%.1f,\"soil_temp\":%.1f,\"soil_humi\":%.1f,"
            "\"ph\":%.2f,\"n\":%u,\"p\":%u,\"k\":%u,"
            "\"ok\":%lu,\"fail\":%lu,"
            "\"plant\":\"%s\",\"updated\":%lu,"
            "\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,"
            "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\"}",
            latest_pkt.seq, latest_pkt.lux, latest_pkt.env_temp_c, latest_pkt.env_humi_pct,
            latest_pkt.press_kpa, latest_pkt.soil_temp_c, latest_pkt.soil_humi_pct,
            latest_pkt.ph, latest_pkt.n, latest_pkt.p, latest_pkt.k,
            (unsigned long)ok_count, (unsigned long)fail_count,
            plant_esc, (unsigned long)g_control_state.update_time_s,
            g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
            g_control_state.fan_speed,
            g_control_state.pump_speed,
            g_control_state.servo_angle,
            g_control_state.light_level,
            reason_esc,
            alert_esc,
            ai_reply_esc);
    } else {
        snprintf(buf, sizeof(buf),
            "{\"seq\":0,\"lux\":0,\"env_temp\":0,\"env_humi\":0,"
            "\"press\":0,\"soil_temp\":0,\"soil_humi\":0,"
            "\"ph\":0,\"n\":0,\"p\":0,\"k\":0,"
            "\"ok\":0,\"fail\":0,"
            "\"plant\":\"%s\",\"updated\":%lu,"
            "\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,"
            "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\"}",
            plant_esc, (unsigned long)g_control_state.update_time_s,
            g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
            g_control_state.fan_speed,
            g_control_state.pump_speed,
            g_control_state.servo_angle,
            g_control_state.light_level,
            reason_esc,
            alert_esc,
            ai_reply_esc);
    }
    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t api_control_get_handler(httpd_req_t *req)
{
    char buf[1024];
    char plant_esc[128];
    char reason_esc[256];
    char alert_esc[320];
    char ai_reply_esc[320];
    json_escape_text(g_monitor_plant, plant_esc, sizeof(plant_esc));
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    json_escape_text(g_last_alert, alert_esc, sizeof(alert_esc));
    json_escape_text(g_last_ai_reply, ai_reply_esc, sizeof(ai_reply_esc));
    snprintf(buf, sizeof(buf),
             "{\"plant\":\"%s\",\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,"
             "\"updated\":%lu,\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\"}",
             plant_esc,
             g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
             g_control_state.fan_speed,
             g_control_state.pump_speed,
             g_control_state.servo_angle,
             g_control_state.light_level,
             (unsigned long)g_control_state.update_time_s,
             reason_esc,
             alert_esc,
             ai_reply_esc);
    set_cors_headers(req);
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

    control_mode_t prev_mode = g_control_mode;
    bool mode_changed = false;

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
        mode_changed = (g_control_mode != prev_mode);
    }

    if (mode_changed) {
        // 切换模式时强制先下发一次停止指令
        apply_control_and_send(g_control_mode, 0, 0, 90, 0, "mode switch stop");
        if (g_control_mode == CONTROL_MODE_SMART) {
            g_smart_control_force_run = true;
        }
    } else if (g_control_mode == CONTROL_MODE_MANUAL) {
        int fan = g_control_state.fan_speed;
        int pump = g_control_state.pump_speed;
        int servo = g_control_state.servo_angle;
        int light = g_control_state.light_level;

        const cJSON *fan_j = cJSON_GetObjectItem(root, "fan");
        const cJSON *pump_j = cJSON_GetObjectItem(root, "pump");
        const cJSON *servo_j = cJSON_GetObjectItem(root, "servo");
        const cJSON *light_j = cJSON_GetObjectItem(root, "light");

        if (fan_j && cJSON_IsNumber(fan_j)) fan = fan_j->valueint;
        if (pump_j && cJSON_IsNumber(pump_j)) pump = pump_j->valueint;
        if (servo_j && cJSON_IsNumber(servo_j)) servo = servo_j->valueint;
        if (light_j && cJSON_IsNumber(light_j)) light = light_j->valueint;

        apply_control_and_send(CONTROL_MODE_MANUAL, fan, pump, servo, light, "manual web control");
    } else {
        send_control_frame_to_stm32(g_control_mode,
                                    g_control_state.fan_speed,
                                    g_control_state.pump_speed,
                                    g_control_state.servo_angle,
                                    g_control_state.light_level);
    }

    cJSON_Delete(root);

    char plant_esc[128];
    char reason_esc[256];
    char alert_esc[320];
    char ai_reply_esc[320];
    json_escape_text(g_monitor_plant, plant_esc, sizeof(plant_esc));
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    json_escape_text(g_last_alert, alert_esc, sizeof(alert_esc));
    json_escape_text(g_last_ai_reply, ai_reply_esc, sizeof(ai_reply_esc));
    char resp[1024];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,"
             "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\"}",
             g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
             g_control_state.fan_speed,
             g_control_state.pump_speed,
             g_control_state.servo_angle,
             g_control_state.light_level,
             reason_esc,
             alert_esc,
             ai_reply_esc);
    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

static esp_err_t api_history_handler(httpd_req_t *req)
{
    set_cors_headers(req);
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
    config.stack_size = 8192;
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

        httpd_uri_t api_options_uri = {
            .uri       = "/api/data",
            .method    = HTTP_OPTIONS,
            .handler   = api_options_handler,
        };
        httpd_register_uri_handler(server, &api_options_uri);

        httpd_uri_t history_uri = {
            .uri       = "/api/history",
            .method    = HTTP_GET,
            .handler   = api_history_handler,
        };
        httpd_register_uri_handler(server, &history_uri);

        httpd_uri_t history_options_uri = {
            .uri       = "/api/history",
            .method    = HTTP_OPTIONS,
            .handler   = api_options_handler,
        };
        httpd_register_uri_handler(server, &history_options_uri);

        httpd_uri_t control_get_uri = {
            .uri       = "/api/control",
            .method    = HTTP_GET,
            .handler   = api_control_get_handler,
        };
        httpd_register_uri_handler(server, &control_get_uri);

        httpd_uri_t control_options_uri = {
            .uri       = "/api/control",
            .method    = HTTP_OPTIONS,
            .handler   = api_options_handler,
        };
        httpd_register_uri_handler(server, &control_options_uri);

        httpd_uri_t control_post_uri = {
            .uri       = "/api/control",
            .method    = HTTP_POST,
            .handler   = api_control_post_handler,
        };
        httpd_register_uri_handler(server, &control_post_uri);

        httpd_uri_t plant_options_uri = {
            .uri       = "/api/plant",
            .method    = HTTP_OPTIONS,
            .handler   = api_options_handler,
        };
        httpd_register_uri_handler(server, &plant_options_uri);

        httpd_uri_t plant_post_uri = {
            .uri       = "/api/plant",
            .method    = HTTP_POST,
            .handler   = api_plant_post_handler,
        };
        httpd_register_uri_handler(server, &plant_post_uri);

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
#define ONENET_TOPIC_SET_REPLY  ONENET_TOPIC_SET "/reply"  /* 控制指令回复 */

/* 处理远程控制指令 */
static void handle_remote_control(const char *data, int len)
{
    if (data == NULL || len <= 0) return;
    
    /* 解析 OneNET 下发格式: {"id":"xxx","version":"1.0","params":{"fan":{"value":50},...}} */
    cJSON *root = cJSON_ParseWithLength(data, len);
    if (root == NULL) {
        ESP_LOGW(TAG, "远程控制: JSON 解析失败");
        return;
    }
    
    cJSON *params = cJSON_GetObjectItem(root, "params");
    if (params == NULL) {
        cJSON_Delete(root);
        return;
    }
    
    int fan = -1, pump = -1, servo = -1, light = -1;
    const char *mode_str = NULL;
    
    /* 解析控制参数 */
    cJSON *fan_obj = cJSON_GetObjectItem(params, "fan");
    if (fan_obj) {
        cJSON *val = cJSON_GetObjectItem(fan_obj, "value");
        if (val && cJSON_IsNumber(val)) fan = val->valueint;
    }
    
    cJSON *pump_obj = cJSON_GetObjectItem(params, "pump");
    if (pump_obj) {
        cJSON *val = cJSON_GetObjectItem(pump_obj, "value");
        if (val && cJSON_IsNumber(val)) pump = val->valueint;
    }
    
    cJSON *servo_obj = cJSON_GetObjectItem(params, "servo");
    if (servo_obj) {
        cJSON *val = cJSON_GetObjectItem(servo_obj, "value");
        if (val && cJSON_IsNumber(val)) servo = val->valueint;
    }
    
    cJSON *light_obj = cJSON_GetObjectItem(params, "light");
    if (light_obj) {
        cJSON *val = cJSON_GetObjectItem(light_obj, "value");
        if (val && cJSON_IsNumber(val)) light = val->valueint;
    }
    
    cJSON *mode_obj = cJSON_GetObjectItem(params, "mode");
    if (mode_obj) {
        cJSON *val = cJSON_GetObjectItem(mode_obj, "value");
        if (val && cJSON_IsString(val)) mode_str = val->valuestring;
    }
    
    /* 切换模式 */
    if (mode_str != NULL) {
        if (strcmp(mode_str, "smart") == 0) {
            g_control_mode = CONTROL_MODE_SMART;
            ESP_LOGI(TAG, "远程切换: 智能模式");
        } else if (strcmp(mode_str, "manual") == 0) {
            g_control_mode = CONTROL_MODE_MANUAL;
            ESP_LOGI(TAG, "远程切换: 手动模式");
        }
    }
    
    /* 手动模式下执行控制 */
    if (g_control_mode == CONTROL_MODE_MANUAL && (fan >= 0 || pump >= 0 || servo >= 0 || light >= 0)) {
        if (fan < 0) fan = g_control_state.fan_speed;
        if (pump < 0) pump = g_control_state.pump_speed;
        if (servo < 0) servo = g_control_state.servo_angle;
        if (light < 0) light = g_control_state.light_level;
        
        apply_control_and_send(CONTROL_MODE_MANUAL, fan, pump, servo, light, "remote mqtt");
        ESP_LOGI(TAG, "远程控制: fan=%d pump=%d servo=%d light=%d", fan, pump, servo, light);
    }
    
    cJSON_Delete(root);
}

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT 已连接 OneNet");
        mqtt_connected = true;
        mqtt_pending_send = true;  /* 重连后立即补发 */
        /* 订阅回复 topic */
        esp_mqtt_client_subscribe(mqtt_client, ONENET_TOPIC_REPLY, 0);
        ESP_LOGI(TAG, "已订阅: %s", ONENET_TOPIC_REPLY);
        /* 订阅远程控制 topic */
        esp_mqtt_client_subscribe(mqtt_client, ONENET_TOPIC_SET, 0);
        ESP_LOGI(TAG, "已订阅: %s", ONENET_TOPIC_SET);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT 连接断开");
        mqtt_connected = false;
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT 消息已送达 broker, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT 收到消息 topic=%.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "MQTT 消息内容=%.*s", event->data_len, event->data);
        /* 处理远程控制指令 */
        if (event->topic_len > 0 && strstr(event->topic, "/property/set") != NULL) {
            handle_remote_control(event->data, event->data_len);
        }
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
static bool a39c_wait_aux_ready(uint32_t timeout_ms)
{
    TickType_t start_tick = xTaskGetTickCount();
    while (gpio_get_level(A39C_AUX_PIN) == 0 &&
           (xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout_ms)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return gpio_get_level(A39C_AUX_PIN) == 1;
}

static void a39c_set_mode_levels(int md0, int md1)
{
    ESP_ERROR_CHECK(gpio_set_level(A39C_MD0_PIN, md0));
    ESP_ERROR_CHECK(gpio_set_level(A39C_MD1_PIN, md1));
    vTaskDelay(pdMS_TO_TICKS(10));
}

static int a39c_read_response(uint8_t *buf, size_t buf_size,
                              uint32_t first_byte_timeout_ms,
                              uint32_t idle_timeout_ms)
{
    int total = 0;
    int n = uart_read_bytes(A39C_UART_NUM, buf, (uint32_t)buf_size,
                            pdMS_TO_TICKS(first_byte_timeout_ms));
    if (n <= 0) {
        return 0;
    }
    total += n;

    while ((size_t)total < buf_size) {
        n = uart_read_bytes(A39C_UART_NUM, buf + total, (uint32_t)(buf_size - total),
                            pdMS_TO_TICKS(idle_timeout_ms));
        if (n <= 0) {
            break;
        }
        total += n;
    }
    return total;
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static void a39c_diag_dump_config(void)
{
#if A39C_DIAG_READ_CONFIG_ON_BOOT
    const uint8_t read_all_cmd[] = {0x00, 0x04, 0x1B};
    const uint8_t read_ver_cmd[] = {0x00, 0x00, 0x01};
    uint8_t resp[96];

    ESP_LOGI(TAG, "A39C diag: switch to config mode for register dump");
    a39c_set_mode_levels(0, 0);
    if (!a39c_wait_aux_ready(1000)) {
        ESP_LOGW(TAG, "A39C diag: AUX did not go high in config mode");
    }

    ESP_ERROR_CHECK(uart_flush_input(A39C_UART_NUM));
    uart_write_bytes(A39C_UART_NUM, read_all_cmd, sizeof(read_all_cmd));
    uart_wait_tx_done(A39C_UART_NUM, pdMS_TO_TICKS(200));

    int n = a39c_read_response(resp, sizeof(resp), 300, 80);
    ESP_LOGI(TAG, "A39C diag read-all response bytes=%d", n);
    if (n > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, resp, n, ESP_LOG_INFO);
    }
    if (n >= 30 && resp[0] == 0x00 && resp[1] == 0x04 && resp[2] == 0x1B) {
        uint32_t baud = be32(&resp[3]);
        uint8_t uart_param = resp[7];
        uint16_t reg06 = ((uint16_t)resp[8] << 8) | resp[9];
        uint16_t work_mode = ((uint16_t)resp[10] << 8) | resp[11];
        uint8_t local_group = resp[23];
        uint8_t local_addr = resp[24];
        uint8_t target_group = resp[25];
        uint8_t target_addr = resp[26];
        uint8_t channel = (uint8_t)((reg06 >> 5) & 0x7F);
        uint8_t power = (uint8_t)((reg06 >> 3) & 0x03);
        uint8_t air_rate = (uint8_t)(reg06 & 0x07);

        ESP_LOGI(TAG, "A39C cfg: baud=%lu uart_param=0x%02X reg06=0x%04X mode=0x%04X",
                 (unsigned long)baud, uart_param, reg06, work_mode);
        ESP_LOGI(TAG, "A39C cfg: channel=%u power_idx=%u air_rate_idx=%u local=%02X:%02X target=%02X:%02X",
                 channel, power, air_rate, local_group, local_addr, target_group, target_addr);
    } else {
        ESP_LOGW(TAG, "A39C diag: unexpected read-all response");
    }

    ESP_ERROR_CHECK(uart_flush_input(A39C_UART_NUM));
    uart_write_bytes(A39C_UART_NUM, read_ver_cmd, sizeof(read_ver_cmd));
    uart_wait_tx_done(A39C_UART_NUM, pdMS_TO_TICKS(200));
    n = a39c_read_response(resp, sizeof(resp), 300, 80);
    ESP_LOGI(TAG, "A39C diag version response bytes=%d", n);
    if (n > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, resp, n, ESP_LOG_INFO);
        if (n > 3) {
            char ver[64];
            int copy_len = n - 3;
            if (copy_len >= (int)sizeof(ver)) copy_len = (int)sizeof(ver) - 1;
            memcpy(ver, &resp[3], (size_t)copy_len);
            ver[copy_len] = '\0';
            ESP_LOGI(TAG, "A39C version: %s", ver);
        }
    }

    ESP_LOGI(TAG, "A39C diag: restore work mode");
    a39c_set_mode_levels(A39C_MD0_LEVEL, A39C_MD1_LEVEL);
    if (!a39c_wait_aux_ready(1000)) {
        ESP_LOGW(TAG, "A39C diag: AUX did not go high after restoring work mode");
    }
    ESP_ERROR_CHECK(uart_flush_input(A39C_UART_NUM));
#endif
}

static void a39c_mode_init(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(A39C_MD0_PIN));
    ESP_ERROR_CHECK(gpio_reset_pin(A39C_MD1_PIN));

    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << A39C_MD0_PIN) | (1ULL << A39C_MD1_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    a39c_set_mode_levels(A39C_MD0_LEVEL, A39C_MD1_LEVEL);

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

    (void)a39c_wait_aux_ready(1000);

    ESP_LOGI(TAG, "A39C ready check: AUX=%d after mode switch",
             gpio_get_level(A39C_AUX_PIN));
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
    ESP_ERROR_CHECK(uart_flush_input(A39C_UART_NUM));
    ESP_ERROR_CHECK(gpio_pullup_en(A39C_RX_PIN));
    ESP_LOGI(TAG, "UART init done: %d 8N1, TX=GPIO%d RX=GPIO%d RX_LEVEL=%d AUX=%d",
             A39C_BAUD,
             A39C_TX_PIN,
             A39C_RX_PIN,
             gpio_get_level(A39C_RX_PIN),
             gpio_get_level(A39C_AUX_PIN));
}

static void check_alerts(const sensor_packet_t *pkt, char *alert_buf, size_t buf_size);

/* ====== LoRa 接收任务 ====== */
static void lora_rx_task(void *arg)
{
    uint8_t buf[FRAME_TOTAL_LEN];
    int state = 0;   /* 0: 找AA 1: 找55 2: 收剩余 */
    int idx = 0;
    uint8_t ack_buf[CONTROL_FRAME_LEN];
    int ack_state = 0; /* 0:找5A 1:找A5 2:收剩余 */
    int ack_idx = 0;
    TickType_t last_report_tick = xTaskGetTickCount();
    uint32_t last_rx_bytes = 0;
    uint32_t last_aux_low_count = 0;
    uint32_t last_rx_pin_low_count = 0;

    while (1) {
        if (gpio_get_level(A39C_AUX_PIN) == 0) {
            aux_low_count++;
        }
        if (gpio_get_level(A39C_RX_PIN) == 0) {
            rx_pin_low_count++;
        }

        uint8_t b;
        int n = uart_read_bytes(A39C_UART_NUM, &b, 1, pdMS_TO_TICKS(100));
        if (n <= 0) {
            TickType_t now_tick = xTaskGetTickCount();
            if ((now_tick - last_report_tick) >= pdMS_TO_TICKS(5000)) {
                uint32_t delta_rx = rx_bytes - last_rx_bytes;
                uint32_t delta_aux_low = aux_low_count - last_aux_low_count;
                uint32_t delta_rx_pin_low = rx_pin_low_count - last_rx_pin_low_count;
                ESP_LOGI(TAG, "5秒统计: RX字节=%lu, OK=%lu, FAIL=%lu",
                         (unsigned long)rx_bytes,
                         (unsigned long)ok_count,
                         (unsigned long)fail_count);
                ESP_LOGI(TAG, "5秒增量: RX新增=%lu, AUX低电平采样=%lu, 当前AUX=%d",
                         (unsigned long)delta_rx,
                         (unsigned long)delta_aux_low,
                         gpio_get_level(A39C_AUX_PIN));
                ESP_LOGI(TAG, "RX delta detail: rx_low=%lu rx_level=%d aux=%d",
                         (unsigned long)delta_rx_pin_low,
                         gpio_get_level(A39C_RX_PIN),
                         gpio_get_level(A39C_AUX_PIN));

                if (delta_rx == 0 && delta_aux_low == 0 && delta_rx_pin_low == 0) {
                    ESP_LOGW(TAG, "未见AUX活动，接收模块可能没有收到空中数据，请检查信道/地址/空口速率");
                } else if (delta_rx == 0 && delta_aux_low > 0) {
                    ESP_LOGW(TAG, "AUX有活动但UART无字节，请检查A39C TXD->ESP32-S3 GPIO%d 以及模块串口参数", A39C_RX_PIN);
                }
                if (delta_rx == 0 && delta_aux_low == 0 && delta_rx_pin_low > 0) {
                    ESP_LOGW(TAG, "RX pin toggles but UART gets no bytes. Check baud/parity/voltage level/shared GND.");
                }
                last_report_tick = now_tick;
                last_rx_bytes = rx_bytes;
                last_aux_low_count = aux_low_count;
                last_rx_pin_low_count = rx_pin_low_count;
            }
            continue;
        }
        rx_bytes += (uint32_t)n;

        /* STM32 控制ACK解析状态机（与传感器帧并行） */
        if (ack_state == 0) {
            if (b == CONTROL_ACK_HEAD1) {
                ack_buf[0] = b;
                ack_state = 1;
            }
        } else if (ack_state == 1) {
            if (b == CONTROL_ACK_HEAD2) {
                ack_buf[1] = b;
                ack_idx = 2;
                ack_state = 2;
            } else if (b == CONTROL_ACK_HEAD1) {
                ack_buf[0] = b;
                ack_state = 1;
            } else {
                ack_state = 0;
            }
        } else {
            ack_buf[ack_idx++] = b;
            if (ack_idx >= CONTROL_FRAME_LEN) {
                (void)parse_control_ack_frame(ack_buf);
                ack_state = 0;
                ack_idx = 0;
            }
        }

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
                        snprintf(g_last_alert, sizeof(g_last_alert), "%s", alert);
                        lcd_update(&dd, true, g_monitor_plant,
                                   mqtt_connected, mqtt_connected, alert);

                        /* 智能模式下，若环境恢复正常，自动发送停机指令 */
                        if (g_control_mode == CONTROL_MODE_SMART) {
                            bool is_alert = (alert[0] != '\0');
                            if (is_alert) {
                                g_auto_stop_sent = false;
                            } else if (!g_auto_stop_sent &&
                                       (g_control_state.fan_speed > 0 ||
                                        g_control_state.pump_speed > 0 ||
                                        g_control_state.servo_angle != 90 ||
                                        g_control_state.light_level > 0)) {
                                apply_control_and_send(CONTROL_MODE_SMART, 0, 0, 90, 0,
                                                       "auto stop by normal data");
                                g_auto_stop_sent = true;
                                ESP_LOGI(TAG, "环境恢复正常，已下发自动停机指令");
                            }
                        }
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
static doubao_chat_client_t g_doubao = {0};
static doubao_chat_client_t g_smart_ctrl = {0};
static baidu_tts_handle_t  g_tts = {0};
static SemaphoreHandle_t g_ai_lock = NULL;
static bool g_ai_clients_ready = false;
static volatile bool g_tts_playing = false;
static volatile bool g_alert_control_hold = false;

static bool ai_lock_take(TickType_t timeout_ticks)
{
    if (g_ai_lock == NULL) {
        return false;
    }
    return xSemaphoreTake(g_ai_lock, timeout_ticks) == pdTRUE;
}

static void ai_lock_give(void)
{
    if (g_ai_lock) {
        xSemaphoreGive(g_ai_lock);
    }
}

static void reset_ai_prompts_locked(void)
{
    char ai_sys_prompt[512];
    char ctrl_sys_prompt[512];

    doubao_chat_clear_history(&g_doubao);
    doubao_chat_clear_history(&g_smart_ctrl);

    snprintf(ctrl_sys_prompt, sizeof(ctrl_sys_prompt),
        "温室控制助手。只输出JSON:{\"fan\":0-100,\"pump\":0-100,\"servo\":0-180,\"light\":0-100,\"reason\":\"...\"}。"
        "高温/强光增风扇,土壤干增水泵,土壤湿减水泵,低光照增补光灯。");
    doubao_chat_add_message(&g_smart_ctrl, "system", ctrl_sys_prompt);

    snprintf(ai_sys_prompt, sizeof(ai_sys_prompt),
        "你是%s种植智慧农业助手。监测对象是%s。"
        "请联网搜索%s的最佳种植条件，"
        "根据搜索结果直接分析传感器数据。"
        "如有异常给出针对%s的建议。"
        "回答极简，不超过50字，不要客套。",
        g_monitor_plant, g_monitor_plant, g_monitor_plant, g_monitor_plant);
    doubao_chat_add_message(&g_doubao, "system", ai_sys_prompt);
}

static esp_err_t api_plant_post_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 256) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content len");
        return ESP_FAIL;
    }

    char body[280] = {0};
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

    const cJSON *plant_j = cJSON_GetObjectItem(root, "plant");
    if (!(plant_j && cJSON_IsString(plant_j) && plant_j->valuestring)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "plant required");
        return ESP_FAIL;
    }

    char new_plant[sizeof(g_monitor_plant)] = {0};
    snprintf(new_plant, sizeof(new_plant), "%s", plant_j->valuestring);
    trim_text_inplace(new_plant);
    if (new_plant[0] == '\0') {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "plant empty");
        return ESP_FAIL;
    }

    bool changed = (strcmp(g_monitor_plant, new_plant) != 0);
    if (changed) {
        snprintf(g_monitor_plant, sizeof(g_monitor_plant), "%s", new_plant);

        if (g_ai_clients_ready && ai_lock_take(pdMS_TO_TICKS(3000))) {
            reset_ai_prompts_locked();
            ai_lock_give();
        }

        apply_control_and_send(g_control_mode, 0, 0, 90, 0, "plant switch stop");
        g_smart_control_force_run = true;
        ESP_LOGI(TAG, "监控对象已切换: %s，已清零并触发智能调节", g_monitor_plant);
    }

    cJSON_Delete(root);

    char plant_esc[128];
    char reason_esc[256];
    char alert_esc[320];
    char ai_reply_esc[320];
    json_escape_text(g_monitor_plant, plant_esc, sizeof(plant_esc));
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    json_escape_text(g_last_alert, alert_esc, sizeof(alert_esc));
    json_escape_text(g_last_ai_reply, ai_reply_esc, sizeof(ai_reply_esc));
    char resp[1024];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"changed\":%s,\"plant\":\"%s\",\"mode\":\"%s\","
             "\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,\"updated\":%lu,"
             "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\"}",
             changed ? "true" : "false",
             plant_esc,
             g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
             g_control_state.fan_speed,
             g_control_state.pump_speed,
             g_control_state.servo_angle,
             g_control_state.light_level,
             (unsigned long)g_control_state.update_time_s,
             reason_esc,
             alert_esc,
             ai_reply_esc);
    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

#if AUDIO_SELF_TEST_ON_BOOT
static void audio_self_test_beep(void)
{
    if (g_audio_handle == NULL) {
        ESP_LOGW(TAG, "音频自检跳过: g_audio_handle 为空");
        return;
    }

    const int sample_rate = 16000;
    const int duration_ms = 180;
    const int freq_hz = 1000;
    const int total_samples = (sample_rate * duration_ms) / 1000;
    const int chunk_samples = 256;
    int16_t pcm[chunk_samples * 2];
    int phase = 0;

    ESP_LOGI(TAG, "开始音频自检蜂鸣");
    for (int written_samples = 0; written_samples < total_samples; ) {
        int cur = total_samples - written_samples;
        if (cur > chunk_samples) {
            cur = chunk_samples;
        }

        for (int i = 0; i < cur; i++) {
            int period = sample_rate / freq_hz;
            int16_t v = (phase < (period / 2)) ? 6000 : -6000;
            pcm[i * 2] = v;
            pcm[i * 2 + 1] = v;
            phase++;
            if (phase >= period) {
                phase = 0;
            }
        }

        size_t bytes = (size_t)cur * 2U * sizeof(int16_t);
        size_t out = 0;
        esp_err_t ret = max98357a_write(g_audio_handle, pcm, bytes, &out, 1000);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "音频自检写入失败: %s", esp_err_to_name(ret));
            return;
        }
        written_samples += cur;
    }
    ESP_LOGI(TAG, "音频自检蜂鸣完成");
}
#endif

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

static bool parse_control_json_from_text(const char *text, int *fan, int *pump, int *servo, int *light, char *reason, size_t reason_size)
{
    if (!text || !fan || !pump || !servo || !light) {
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
    const cJSON *light_j = cJSON_GetObjectItem(root, "light");
    const cJSON *reason_j = cJSON_GetObjectItem(root, "reason");

    bool ok = fan_j && cJSON_IsNumber(fan_j)
              && pump_j && cJSON_IsNumber(pump_j)
              && servo_j && cJSON_IsNumber(servo_j);

    if (ok) {
        *fan = fan_j->valueint;
        *pump = pump_j->valueint;
        *servo = servo_j->valueint;
        *light = (light_j && cJSON_IsNumber(light_j)) ? light_j->valueint : 0;
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
    // 极简 prompt，减少 token 数以避免 API 500 错误
    snprintf(buf, size,
    "传感器:光%.0flx,气温%.1f℃,湿%.0f%%,土温%.1f℃,土湿%.0f%%,pH%.1f,N%u P%u K%u。"
    "执行器:fan=%u,pump=%u,servo=%u,light=%u。告警:%s。"
    "输出JSON:{\"fan\":0-100,\"pump\":0-100,\"servo\":0-180,\"light\":0-100,\"reason\":\"...\"}",
        latest_pkt.lux, latest_pkt.env_temp_c, latest_pkt.env_humi_pct,
        latest_pkt.soil_temp_c, latest_pkt.soil_humi_pct, latest_pkt.ph,
        latest_pkt.n, latest_pkt.p, latest_pkt.k,
        g_control_state.fan_speed, g_control_state.pump_speed, g_control_state.servo_angle,
        g_control_state.light_level,
        g_last_alert[0] ? g_last_alert : "无");
}

static void build_fallback_control(const sensor_packet_t *pkt,
                                   int *fan, int *pump, int *servo, int *light,
                                   char *reason, size_t reason_size)
{
    int f = 20;
    int p = 0;
    int s = 90;
    int l = 0;

    if (pkt->env_temp_c >= 32.0f || pkt->lux >= 60000.0f) {
        f = 80;
        s = 130;
    } else if (pkt->env_temp_c >= 28.0f || pkt->lux >= 30000.0f) {
        f = 55;
        s = 110;
    }

    if (pkt->soil_humi_pct <= 15.0f) {
        p = 85;
    } else if (pkt->soil_humi_pct <= 30.0f) {
        p = 45;
    } else if (pkt->soil_humi_pct >= 80.0f) {
        p = 0;
    }

    /* 光照不足时自动补光 */
    if (pkt->lux < 500.0f) {
        l = 80;
    } else if (pkt->lux < 2000.0f) {
        l = 50;
    } else if (pkt->lux < 5000.0f) {
        l = 20;
    }

    *fan = f;
    *pump = p;
    *servo = s;
    *light = l;
    if (reason && reason_size > 0) {
        snprintf(reason, reason_size, "fallback by sensor");
    }
}

static void enforce_sensor_safety_control(const sensor_packet_t *pkt,
                                          int *fan, int *pump, int *servo, int *light,
                                          char *reason, size_t reason_size)
{
    if (!pkt || !fan || !pump || !servo || !light) {
        return;
    }

    // 安全硬规则：土壤极干必须开泵，土壤过湿必须停泵，防止AI误判。
    if (pkt->soil_humi_pct <= 15.0f) {
        if (*pump < 60) {
            *pump = 60;
        }
        if (reason && reason_size > 0) {
            snprintf(reason, reason_size, "safety: dry soil pump>=60");
        }
    } else if (pkt->soil_humi_pct >= 80.0f) {
        if (*pump != 0) {
            *pump = 0;
        }
        if (reason && reason_size > 0) {
            snprintf(reason, reason_size, "safety: wet soil pump=0");
        }
    }

    if (*fan < 0) *fan = 0;
    if (*fan > 100) *fan = 100;
    if (*pump < 0) *pump = 0;
    if (*pump > 100) *pump = 100;
    if (*servo < 0) *servo = 0;
    if (*servo > 180) *servo = 180;
    if (*light < 0) *light = 0;
    if (*light > 100) *light = 100;
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
    doubao_chat_config_t doubao_cfg = {
        .api_key  = AI_API_KEY,
        .url      = AI_URL,
        .user_id  = NULL,
        .model    = AI_MODEL,
        .timeout_ms = 30000,
        .stream   = false,  // doubao-seed-1-6-lite 不支持流式
        .enable_web_search = false,
        .search_mode = NULL,
    };
    doubao_chat_init(&g_doubao, &doubao_cfg);

    doubao_chat_config_t ctrl_cfg = doubao_cfg;
    ctrl_cfg.enable_web_search = false;
    doubao_chat_init(&g_smart_ctrl, &ctrl_cfg);
    if (ai_lock_take(pdMS_TO_TICKS(3000))) {
        reset_ai_prompts_locked();
        ai_lock_give();
    }
    g_ai_clients_ready = true;

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
    ret = baidu_tts_init(&g_tts, &tts_cfg, g_audio_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "百度TTS初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    /* 启动阶段避免立即拉高功放和播放自检音，降低峰值电流防止Brownout。
     * 首次TTS播放时会通过max98357a_write自动使能功放。 */
#if AUDIO_SELF_TEST_ON_BOOT
    ret = max98357a_enable(g_audio_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX98357A 使能失败: %s", esp_err_to_name(ret));
        return;
    }
    audio_self_test_beep();
#endif

    ESP_LOGI(TAG, "AI 语音播报模块初始化完成");
}

/* 构建传感器数据摘要发送给 AI */
static void build_sensor_prompt(char *buf, size_t size, bool is_alert, const char *alerts)
{
    if (has_data) {
        int n = snprintf(buf, size,
            "监测对象：%s。数据：光照%.0f lx，气温%.1f℃，湿度%.0f%%，"
            "气压%.0f，土温%.1f℃，土湿%.0f%%，"
            "pH%.1f，N%u P%u K%u mg/kg。",
            g_monitor_plant,
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
        char intro[384];
        snprintf(intro, sizeof(intro),
            "%s智慧监测系统已启动，开始为您实时分析环境数据。",
            g_monitor_plant);
        esp_err_t intro_ret = ESP_FAIL;
        for (int i = 0; i < 3; i++) {
            g_tts_playing = true;
            intro_ret = baidu_tts_speak(&g_tts, intro);
            g_tts_playing = false;
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
        snprintf(g_last_alert, sizeof(g_last_alert), "%s", alert_buf);
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

        if (is_alert) {
            // 异常流程中先暂停智能调节，待告警语音播报完成后再放行
            g_alert_control_hold = true;
        }

        normal_elapsed_s = 0;  /* 重置计时 */

        ESP_LOGI(TAG, "===== AI 语音播报开始 (异常=%d) =====", is_alert);
        ESP_LOGI(TAG, "空闲堆内存: %lu bytes", (unsigned long)esp_get_free_heap_size());

        /* 1) 构建传感器数据提示词 */
        char prompt[512];
        build_sensor_prompt(prompt, sizeof(prompt), is_alert, alert_buf);
        ESP_LOGI(TAG, "AI 提示词: %s", prompt);

        /* 2) 发送给大模型 */
        bool ok = false;
        char response_local[512] = {0};
        if (g_ai_clients_ready && ai_lock_take(pdMS_TO_TICKS(30000))) {
            doubao_chat_add_message(&g_doubao, "user", prompt);
            doubao_chat_trim_history(&g_doubao);

            ok = doubao_chat_request(&g_doubao);
            /* 每次请求后释放HTTP连接，避免复用过期连接导致下次收不到数据 */
            doubao_chat_close_connection(&g_doubao);

            if (ok) {
                const char *response = doubao_chat_get_last_response(&g_doubao);
                snprintf(response_local, sizeof(response_local), "%s", response ? response : "");
                doubao_chat_add_message(&g_doubao, "assistant", response_local);
            }
            ai_lock_give();
        }

        if (ok) {
            ESP_LOGI(TAG, "AI 回复: %s", response_local);
            snprintf(g_last_ai_reply, sizeof(g_last_ai_reply), "%.255s", response_local);

            /* 3) TTS 语音播放 */
            if (strlen(response_local) > 0) {
                ESP_LOGI(TAG, "TTS 播放中...");
                g_tts_playing = true;
                esp_err_t ret = baidu_tts_speak(&g_tts, response_local);
                g_tts_playing = false;
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "TTS 播放失败: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGW(TAG, "AI 回复为空，跳过TTS");
            }

            if (is_alert && g_control_mode == CONTROL_MODE_SMART) {
                // 异常场景：必须在语音播报结束后再触发智能调节
                g_smart_control_force_run = true;
                ESP_LOGI(TAG, "异常播报已完成，开始触发智能调节");
            }
        } else {
            ESP_LOGW(TAG, "AI 请求失败");
            snprintf(g_last_ai_reply, sizeof(g_last_ai_reply), "%s", "");
        }

        if (is_alert) {
            g_alert_control_hold = false;
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
    uint32_t consecutive_ai_fail = 0;

    while (1) {
        uint32_t now_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        bool interval_due = (now_s - last_run_s) >= SMART_CONTROL_INTERVAL_S;

        if (!has_data || g_control_mode != CONTROL_MODE_SMART) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (g_tts_playing || g_alert_control_hold) {
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

        bool ok = false;
        const int max_retry = 3;
        char resp_local[512] = {0};
        if (g_ai_clients_ready && ai_lock_take(pdMS_TO_TICKS(30000))) {
            doubao_chat_add_message(&g_smart_ctrl, "user", prompt);
            doubao_chat_trim_history(&g_smart_ctrl);

            for (int i = 0; i < max_retry; i++) {
                ok = doubao_chat_request(&g_smart_ctrl);
                doubao_chat_close_connection(&g_smart_ctrl);
                if (ok) {
                    const char *resp = doubao_chat_get_last_response(&g_smart_ctrl);
                    snprintf(resp_local, sizeof(resp_local), "%s", resp ? resp : "");
                    break;
                }
                ESP_LOGW(TAG, "智能控制AI请求失败，重试 %d/%d", i + 1, max_retry);
                if (i < max_retry - 1) {
                    vTaskDelay(pdMS_TO_TICKS((i + 1) * 1000));
                }
            }
            ai_lock_give();
        }

        if (!ok) {
            consecutive_ai_fail++;
            ESP_LOGW(TAG, "智能控制AI请求失败，连续失败=%lu", (unsigned long)consecutive_ai_fail);

            // 本轮AI请求失败即启用本地降级控制，避免土壤极干时迟迟不开泵。
            int fan = 0, pump = 0, servo = 90, light = 0;
            char reason[96] = "fallback by sensor";
            build_fallback_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            enforce_sensor_safety_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            apply_control_and_send(CONTROL_MODE_SMART, fan, pump, servo, light, reason);
            ESP_LOGW(TAG, "AI请求失败，启用本地降级控制 fan=%d pump=%d servo=%d light=%d", fan, pump, servo, light);

            // 失败后短暂退避，避免长时间阻塞导致异常播报后的控制响应变慢
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        consecutive_ai_fail = 0;

        int fan = 0, pump = 0, servo = 90, light = 0;
        char reason[96] = "smart adjust";

        if (parse_control_json_from_text(resp_local, &fan, &pump, &servo, &light, reason, sizeof(reason))) {
            enforce_sensor_safety_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            apply_control_and_send(CONTROL_MODE_SMART, fan, pump, servo, light, reason);
            ESP_LOGI(TAG, "智能控制生效 fan=%d pump=%d servo=%d light=%d reason=%s", fan, pump, servo, light, reason);
        } else {
            ESP_LOGW(TAG, "智能控制解析失败: %s", resp_local[0] ? resp_local : "(null)");
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

    init_mbedtls_allocator();

    g_ai_lock = xSemaphoreCreateMutex();
    if (!g_ai_lock) {
        ESP_LOGE(TAG, "创建AI互斥锁失败");
    }

    /* 先初始化 LoRa 接收硬件 */
    a39c_mode_init();
    uart_init_a39c();
    a39c_diag_dump_config();

    /* 上电默认进入智能模式，并先下发一次全关指令，确保执行器安全状态 */
    g_control_mode = CONTROL_MODE_SMART;
    apply_control_and_send(CONTROL_MODE_SMART, 0, 0, 90, 0, "boot safe stop");
    g_smart_control_force_run = true;
    ESP_LOGI(TAG, "上电默认智能模式，已下发执行器全关指令");

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
        lcd_update(NULL, false, g_monitor_plant, false, false, "");
    }

    /* 在独立任务中运行 LoRa 接收循环 */
    ESP_LOGI(TAG, "空闲堆内存: %lu bytes", (unsigned long)esp_get_free_heap_size());
    xTaskCreate(lora_rx_task, "lora_rx", 8192, NULL, 10, NULL);
    xTaskCreate(control_retry_task, "ctrl_retry", 4096, NULL, 8, NULL);

    /* 初始化 AI 语音播报并启动任务 */
    ai_audio_init();
    xTaskCreate(ai_report_task, "ai_report", 16384, NULL, 3, NULL);
    xTaskCreate(ai_smart_control_task, "ai_smart_ctl", 16384, NULL, 3, NULL);

    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  智慧农业监测系统已启动");
    ESP_LOGI(TAG, "  LoRa + OneNet + AI语音播报");
    ESP_LOGI(TAG, "============================================");
}
