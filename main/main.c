#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
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
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "mbedtls/md.h"
#include "mbedtls/base64.h"
#include "mbedtls/platform.h"

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
#define WIFI_MAX_TX_POWER_QDBM 52         /* 13dBm, 降低峰值电流 */

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
    uint8_t light_level;  /* 0-100 */
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
    .reason = "初始化",
};
static volatile bool g_smart_control_force_run = false;
static char g_last_alert[512] = "";      /* 增大以容纳完整中文异常描述 */
static char g_last_ai_reply[256] = "";
static volatile bool g_auto_stop_sent = false;
static char g_monitor_plant[64] = PLANT_SPECIES;

typedef struct {
    bool valid;
    char city[24];
    char weather[24];
    char wind_direction[24];
    char wind_power[16];
    float temperature_c;
    float humidity_pct;
    char report_time[32];
    uint32_t update_time_s;
} weather_state_t;

static weather_state_t g_weather = {
    .valid = false,
    .city = "Nanning",
};
static SemaphoreHandle_t g_weather_lock = NULL;
static uint32_t g_last_sensor_sample_time_s = 0;
static uint32_t g_last_cloud_report_time_s = 0;

static const char *monitor_target_for_lcd(void)
{
    static char label[16];
    size_t len = strlen(g_monitor_plant);
    bool ascii_only = true;

    for (size_t i = 0; i < len; ++i) {
        unsigned char ch = (unsigned char)g_monitor_plant[i];
        if (ch < 0x20 || ch > 0x7e) {
            ascii_only = false;
            break;
        }
    }

    if (ascii_only && len > 0) {
        snprintf(label, sizeof(label), "%.15s", g_monitor_plant);
        return label;
    }

    if (strcmp(g_monitor_plant, PLANT_SPECIES) == 0) {
        return PLANT_SPECIES_EN;
    }

    return "Monitor";
}

typedef struct {
    bool valid;
    char plant[64];
    float press_min;
    float press_max;
    float env_temp_min;
    float env_temp_max;
    float env_humi_min;
    float env_humi_max;
    float soil_temp_min;
    float soil_temp_max;
    float soil_humi_min;
    float soil_humi_max;
    float ph_min;
    float ph_max;
    float n_min;
    float n_max;
    float p_min;
    float p_max;
    float k_min;
    float k_max;
    float lux_min;
    float lux_max;
    char summary[160];
    char source[24];
    uint32_t update_time_s;
} plant_profile_t;

static plant_profile_t g_plant_profile = {0};
static SemaphoreHandle_t g_plant_profile_lock = NULL;
static volatile bool g_plant_profile_refresh_requested = true;

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
 * Byte9  : light    (0-100)
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
 * Byte9  : light_actual
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
#define SMART_FAN_STEP_MAX  20
#define SMART_PUMP_STEP_MAX 25
#define SMART_CONTROL_MIN_TRIGGER_GAP_S 5
#define SMART_CONTROL_FAIL_BACKOFF_S 3

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
    uint8_t light_actual;
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
    uart_write_bytes(A39C_UART_NUM, frame, CONTROL_FRAME_LEN);
    // 静默发送，不输出日志
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
    frame[9] = light;
    frame[10] = calc_xor(frame, CONTROL_FRAME_LEN - 1);

    memcpy(g_ctrl_tx.frame, frame, CONTROL_FRAME_LEN);
    g_ctrl_tx.pending = true;
    g_ctrl_tx.seq = frame[4];
    g_ctrl_tx.retry_count = 0;
    g_ctrl_tx.last_send_us = esp_timer_get_time();

    send_control_frame_raw(frame, "CTRL");
    // 静默处理，不输出控制帧详情
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
    g_ctrl_ack.light_actual = frame[9];
    g_ctrl_ack.recv_time_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    // 静默处理ACK，仅在调试时输出
    if (g_ctrl_tx.pending && seq == g_ctrl_tx.seq) {
        g_ctrl_tx.pending = false;
    }
    return true;
}

static void apply_control_internal(control_mode_t mode, int fan_speed, int pump_speed,
                                   int servo_angle, int light_level, const char *reason,
                                   bool apply_smart_ramp)
{
    uint8_t new_fan = clamp_u8(fan_speed, 0, 100);
    uint8_t new_pump = clamp_u8(pump_speed, 0, 100);
    uint8_t new_servo = clamp_u8(servo_angle, 0, 180);
    uint8_t new_light = clamp_u8(light_level, 0, 100);

    if (mode == CONTROL_MODE_SMART && apply_smart_ramp) {
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

static void apply_control_and_send(control_mode_t mode, int fan_speed, int pump_speed,
                                   int servo_angle, int light_level, const char *reason)
{
    // 智能模式关闭平滑控制，执行器直接按目标值生效。
    apply_control_internal(mode, fan_speed, pump_speed, servo_angle, light_level, reason, false);
}

static void apply_control_and_send_immediate(control_mode_t mode, int fan_speed, int pump_speed,
                                             int servo_angle, int light_level, const char *reason)
{
    apply_control_internal(mode, fan_speed, pump_speed, servo_angle, light_level, reason, false);
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
                // 静默重发，不输出日志
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

static void weather_state_store(const weather_state_t *src)
{
    if (!src) {
        return;
    }

    if (g_weather_lock && xSemaphoreTake(g_weather_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_weather = *src;
        xSemaphoreGive(g_weather_lock);
    } else {
        g_weather = *src;
    }
}

static void weather_state_load(weather_state_t *dst)
{
    if (!dst) {
        return;
    }

    if (g_weather_lock && xSemaphoreTake(g_weather_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        *dst = g_weather;
        xSemaphoreGive(g_weather_lock);
    } else {
        *dst = g_weather;
    }
}

typedef struct {
    weather_state_t weather;
    uint32_t weather_age_s;
    bool weather_stale;
} weather_runtime_info_t;

typedef struct {
    plant_profile_t profile;
    bool ready;
    bool matches_current;
    uint32_t age_s;
} plant_profile_runtime_info_t;

static uint32_t monotonic_time_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

static void get_weather_runtime_info(weather_runtime_info_t *out)
{
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));
    weather_state_load(&out->weather);

    out->weather_age_s = AMAP_WEATHER_STALE_THRESHOLD_S + 1U;
    out->weather_stale = true;

    if (out->weather.valid && out->weather.update_time_s > 0) {
        uint32_t now_s = monotonic_time_s();
        if (now_s >= out->weather.update_time_s) {
            out->weather_age_s = now_s - out->weather.update_time_s;
        } else {
            out->weather_age_s = 0;
        }
        out->weather_stale = (out->weather_age_s > AMAP_WEATHER_STALE_THRESHOLD_S);
    }
}

static void plant_profile_store(const plant_profile_t *src)
{
    if (!src) {
        return;
    }

    if (g_plant_profile_lock && xSemaphoreTake(g_plant_profile_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_plant_profile = *src;
        xSemaphoreGive(g_plant_profile_lock);
    } else {
        g_plant_profile = *src;
    }
}

static void plant_profile_load(plant_profile_t *dst)
{
    if (!dst) {
        return;
    }

    if (g_plant_profile_lock && xSemaphoreTake(g_plant_profile_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        *dst = g_plant_profile;
        xSemaphoreGive(g_plant_profile_lock);
    } else {
        *dst = g_plant_profile;
    }
}

static float clamp_profile_value(float value, float min_v, float max_v)
{
    if (value < min_v) return min_v;
    if (value > max_v) return max_v;
    return value;
}

static void fill_default_plant_profile(plant_profile_t *profile, const char *plant_name, const char *source)
{
    if (!profile) {
        return;
    }

    memset(profile, 0, sizeof(*profile));
    profile->valid = true;
    snprintf(profile->plant, sizeof(profile->plant), "%s", plant_name ? plant_name : "");
    profile->press_min = 95.0f;
    profile->press_max = 105.0f;
    profile->env_temp_min = 18.0f;
    profile->env_temp_max = 30.0f;
    profile->env_humi_min = 45.0f;
    profile->env_humi_max = 80.0f;
    profile->soil_temp_min = 18.0f;
    profile->soil_temp_max = 32.0f;
    profile->soil_humi_min = 35.0f;
    profile->soil_humi_max = 70.0f;
    profile->ph_min = 5.5f;
    profile->ph_max = 7.5f;
    profile->n_min = 3.0f;
    profile->n_max = 8.0f;
    profile->p_min = 1.0f;
    profile->p_max = 4.0f;
    profile->k_min = 2.0f;
    profile->k_max = 6.0f;
    profile->lux_min = 8000.0f;
    profile->lux_max = 35000.0f;
    snprintf(profile->summary, sizeof(profile->summary), "generic greenhouse profile");
    snprintf(profile->source, sizeof(profile->source), "%s", source ? source : "default");
    profile->update_time_s = monotonic_time_s();
}

static void request_plant_profile_refresh(const char *plant_name)
{
    plant_profile_t profile = {0};
    fill_default_plant_profile(&profile, plant_name ? plant_name : g_monitor_plant, "pending");
    profile.valid = false;
    profile.summary[0] = '\0';
    plant_profile_store(&profile);
    g_plant_profile_refresh_requested = true;
}

static void get_plant_profile_runtime_info(plant_profile_runtime_info_t *out)
{
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));
    plant_profile_load(&out->profile);
    out->ready = out->profile.valid;
    out->matches_current = (strcmp(out->profile.plant, g_monitor_plant) == 0);

    if (out->profile.update_time_s > 0) {
        uint32_t now_s = monotonic_time_s();
        out->age_s = (now_s >= out->profile.update_time_s) ? (now_s - out->profile.update_time_s) : 0;
    }
}

static void json_copy_string_field(const cJSON *obj, const char *key, char *dst, size_t dst_size)
{
    if (!dst || dst_size == 0) {
        return;
    }
    dst[0] = '\0';

    if (!obj || !key) {
        return;
    }

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(item) && item->valuestring) {
        snprintf(dst, dst_size, "%s", item->valuestring);
    }
}

static const char *json_get_first_string_value(const cJSON *obj,
                                               const char *key1,
                                               const char *key2,
                                               const char *key3,
                                               const char *key4)
{
    const char *keys[] = { key1, key2, key3, key4 };
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        if (!keys[i] || !keys[i][0]) {
            continue;
        }
        const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, keys[i]);
        if (cJSON_IsString(item) && item->valuestring && item->valuestring[0]) {
            return item->valuestring;
        }
    }
    return NULL;
}

static bool json_read_float_field(const cJSON *obj, const char *key, float *out)
{
    if (!obj || !key || !out) {
        return false;
    }

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(item)) {
        *out = (float)item->valuedouble;
        return true;
    }

    if (cJSON_IsString(item) && item->valuestring && item->valuestring[0]) {
        char *endptr = NULL;
        float val = strtof(item->valuestring, &endptr);
        if (endptr != item->valuestring) {
            *out = val;
            return true;
        }
    }

    return false;
}

static bool parse_plant_profile_json_from_text(const char *text, plant_profile_t *out, const char *fallback_plant)
{
    if (!text || !out) {
        return false;
    }

    const char *start = strchr(text, '{');
    const char *end = strrchr(text, '}');
    if (!start || !end || end <= start) {
        return false;
    }

    size_t len = (size_t)(end - start + 1);
    if (len >= 768) {
        len = 767;
    }

    char json_buf[768];
    memcpy(json_buf, start, len);
    json_buf[len] = '\0';

    cJSON *root = cJSON_Parse(json_buf);
    if (!root) {
        return false;
    }

    plant_profile_t profile = {0};
    profile.valid = true;
    profile.update_time_s = monotonic_time_s();

    const cJSON *plant_j = cJSON_GetObjectItem(root, "plant");
    if (cJSON_IsString(plant_j) && plant_j->valuestring) {
        snprintf(profile.plant, sizeof(profile.plant), "%s", plant_j->valuestring);
    } else {
        snprintf(profile.plant, sizeof(profile.plant), "%s", fallback_plant ? fallback_plant : g_monitor_plant);
    }

    json_read_float_field(root, "press_min", &profile.press_min);
    json_read_float_field(root, "press_max", &profile.press_max);
    json_read_float_field(root, "env_temp_min", &profile.env_temp_min);
    json_read_float_field(root, "env_temp_max", &profile.env_temp_max);
    json_read_float_field(root, "env_humi_min", &profile.env_humi_min);
    json_read_float_field(root, "env_humi_max", &profile.env_humi_max);
    json_read_float_field(root, "soil_temp_min", &profile.soil_temp_min);
    json_read_float_field(root, "soil_temp_max", &profile.soil_temp_max);
    json_read_float_field(root, "soil_humi_min", &profile.soil_humi_min);
    json_read_float_field(root, "soil_humi_max", &profile.soil_humi_max);
    json_read_float_field(root, "ph_min", &profile.ph_min);
    json_read_float_field(root, "ph_max", &profile.ph_max);
    json_read_float_field(root, "n_min", &profile.n_min);
    json_read_float_field(root, "n_max", &profile.n_max);
    json_read_float_field(root, "p_min", &profile.p_min);
    json_read_float_field(root, "p_max", &profile.p_max);
    json_read_float_field(root, "k_min", &profile.k_min);
    json_read_float_field(root, "k_max", &profile.k_max);
    json_read_float_field(root, "lux_min", &profile.lux_min);
    json_read_float_field(root, "lux_max", &profile.lux_max);
    json_copy_string_field(root, "summary", profile.summary, sizeof(profile.summary));

    if (profile.press_max <= profile.press_min ||
        profile.env_temp_max <= profile.env_temp_min ||
        profile.env_humi_max <= profile.env_humi_min ||
        profile.soil_temp_max <= profile.soil_temp_min ||
        profile.soil_humi_max <= profile.soil_humi_min ||
        profile.ph_max <= profile.ph_min ||
        profile.n_max <= profile.n_min ||
        profile.p_max <= profile.p_min ||
        profile.k_max <= profile.k_min ||
        profile.lux_max <= profile.lux_min) {
        cJSON_Delete(root);
        return false;
    }

    profile.press_min = clamp_profile_value(profile.press_min, 85.0f, 110.0f);
    profile.press_max = clamp_profile_value(profile.press_max, 88.0f, 115.0f);
    profile.env_temp_min = clamp_profile_value(profile.env_temp_min, 0.0f, 45.0f);
    profile.env_temp_max = clamp_profile_value(profile.env_temp_max, 5.0f, 50.0f);
    profile.env_humi_min = clamp_profile_value(profile.env_humi_min, 10.0f, 95.0f);
    profile.env_humi_max = clamp_profile_value(profile.env_humi_max, 20.0f, 100.0f);
    profile.soil_temp_min = clamp_profile_value(profile.soil_temp_min, 0.0f, 45.0f);
    profile.soil_temp_max = clamp_profile_value(profile.soil_temp_max, 5.0f, 50.0f);
    profile.soil_humi_min = clamp_profile_value(profile.soil_humi_min, 5.0f, 95.0f);
    profile.soil_humi_max = clamp_profile_value(profile.soil_humi_max, 10.0f, 100.0f);
    profile.ph_min = clamp_profile_value(profile.ph_min, 3.0f, 9.0f);
    profile.ph_max = clamp_profile_value(profile.ph_max, 4.0f, 10.0f);
    profile.n_min = clamp_profile_value(profile.n_min, 0.0f, 100.0f);
    profile.n_max = clamp_profile_value(profile.n_max, 0.1f, 200.0f);
    profile.p_min = clamp_profile_value(profile.p_min, 0.0f, 100.0f);
    profile.p_max = clamp_profile_value(profile.p_max, 0.1f, 200.0f);
    profile.k_min = clamp_profile_value(profile.k_min, 0.0f, 100.0f);
    profile.k_max = clamp_profile_value(profile.k_max, 0.1f, 200.0f);
    profile.lux_min = clamp_profile_value(profile.lux_min, 0.0f, 120000.0f);
    profile.lux_max = clamp_profile_value(profile.lux_max, 100.0f, 150000.0f);
    snprintf(profile.source, sizeof(profile.source), "%s", "web_search");

    if (profile.summary[0] == '\0') {
        snprintf(profile.summary, sizeof(profile.summary), "profile from web search");
    }

    *out = profile;
    cJSON_Delete(root);
    return true;
}

static bool parse_amap_weather_json(const char *json_text, weather_state_t *out)
{
    if (!json_text || !out) {
        return false;
    }

    cJSON *root = cJSON_Parse(json_text);
    if (!root) {
        return false;
    }

    bool ok = false;
    weather_state_t parsed = {0};

    const cJSON *status_j = cJSON_GetObjectItemCaseSensitive(root, "status");
    const cJSON *lives_j = cJSON_GetObjectItemCaseSensitive(root, "lives");
    const cJSON *live_j = cJSON_IsArray(lives_j) ? cJSON_GetArrayItem(lives_j, 0) : NULL;

    if (cJSON_IsString(status_j) && status_j->valuestring && strcmp(status_j->valuestring, "1") == 0 && cJSON_IsObject(live_j)) {
        parsed.valid = true;
        snprintf(parsed.city, sizeof(parsed.city), "Nanning");
        json_copy_string_field(live_j, "city", parsed.city, sizeof(parsed.city));
        if (parsed.city[0] == '\0') {
            snprintf(parsed.city, sizeof(parsed.city), "Nanning");
        }

        json_copy_string_field(live_j, "weather", parsed.weather, sizeof(parsed.weather));
        json_copy_string_field(live_j, "winddirection", parsed.wind_direction, sizeof(parsed.wind_direction));
        json_copy_string_field(live_j, "windpower", parsed.wind_power, sizeof(parsed.wind_power));
        json_copy_string_field(live_j, "reporttime", parsed.report_time, sizeof(parsed.report_time));

        if (!json_read_float_field(live_j, "temperature_float", &parsed.temperature_c)) {
            json_read_float_field(live_j, "temperature", &parsed.temperature_c);
        }
        if (!json_read_float_field(live_j, "humidity_float", &parsed.humidity_pct)) {
            json_read_float_field(live_j, "humidity", &parsed.humidity_pct);
        }

        parsed.update_time_s = monotonic_time_s();
        *out = parsed;
        ok = true;
    }

    cJSON_Delete(root);
    return ok;
}

static esp_err_t fetch_amap_weather_once(weather_state_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    char url[256];
    snprintf(url, sizeof(url),
             "https://restapi.amap.com/v3/weather/weatherInfo?key=%s&city=%s&extensions=base&output=JSON",
             AMAP_WEATHER_API_KEY,
             AMAP_WEATHER_CITY_ADCODE);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 12000,
        .buffer_size = 1536,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "高德天气请求打开失败: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    (void)esp_http_client_fetch_headers(client);

    char resp[2048];
    int total = 0;
    while (total < (int)sizeof(resp) - 1) {
        int r = esp_http_client_read(client, resp + total, (int)sizeof(resp) - 1 - total);
        if (r < 0) {
            err = ESP_FAIL;
            break;
        }
        if (r == 0) {
            break;
        }
        total += r;
    }
    resp[total] = '\0';

    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "高德天气读取失败");
        return err;
    }

    if (status_code != 200) {
        ESP_LOGW(TAG, "高德天气HTTP状态异常: %d body=%.180s", status_code, resp);
        return ESP_FAIL;
    }

    if (!parse_amap_weather_json(resp, out)) {
        ESP_LOGW(TAG, "高德天气解析失败: %.180s", resp);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void build_weather_prompt_text(char *buf, size_t size)
{
    if (!buf || size == 0) {
        return;
    }

    weather_runtime_info_t weather_rt = {0};
    get_weather_runtime_info(&weather_rt);
    const weather_state_t *snapshot = &weather_rt.weather;

    if (!snapshot->valid) {
        snprintf(buf, size,
                 "南宁天气暂无，weather_age_s=%lu，weather_stale=1，正在获取",
                 (unsigned long)weather_rt.weather_age_s);
        return;
    }

    const char *city = snapshot->city[0] ? snapshot->city : "Nanning";
    const char *weather = snapshot->weather[0] ? snapshot->weather : "unknown";
    const char *wind_dir = snapshot->wind_direction[0] ? snapshot->wind_direction : "unknown";
    const char *wind_power = snapshot->wind_power[0] ? snapshot->wind_power : "unknown";
    const char *report_time = snapshot->report_time[0] ? snapshot->report_time : "-";

    snprintf(buf, size,
             "%s当前%s，气温%.1f℃，湿度%.0f%%，%s风%s，观测时间%.31s，weather_age_s=%lu，weather_stale=%d",
             city,
             weather,
             snapshot->temperature_c,
             snapshot->humidity_pct,
             wind_dir,
             wind_power,
             report_time,
             (unsigned long)weather_rt.weather_age_s,
             weather_rt.weather_stale ? 1 : 0);
}

static void weather_fetch_task(void *arg)
{
    (void)arg;
    TickType_t normal_delay = pdMS_TO_TICKS((uint32_t)AMAP_WEATHER_REFRESH_INTERVAL_S * 1000U);
    const TickType_t retry_delay = pdMS_TO_TICKS(60000);

    if (normal_delay == 0) {
        normal_delay = pdMS_TO_TICKS(300000);
    }

    ESP_LOGI(TAG, "天气任务启动，城市adcode=%s", AMAP_WEATHER_CITY_ADCODE);

    while (1) {
        if (s_wifi_event_group) {
            EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
            if ((bits & WIFI_CONNECTED_BIT) == 0) {
                ESP_LOGI(TAG, "天气任务等待WiFi连接...");
                xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
            }
        }

        weather_state_t latest = {0};
        esp_err_t err = fetch_amap_weather_once(&latest);
        if (err == ESP_OK) {
            weather_state_store(&latest);
            ESP_LOGI(TAG, "天气更新成功: %s %s %.1f℃ 湿度%.0f%%",
                     latest.city,
                     latest.weather,
                     latest.temperature_c,
                     latest.humidity_pct);
            vTaskDelay(normal_delay);
        } else {
            ESP_LOGW(TAG, "天气更新失败，60秒后重试");
            vTaskDelay(retry_delay);
        }
    }
}

static esp_err_t api_data_handler(httpd_req_t *req)
{
    char buf[3584];
    char plant_esc[128];
    char reason_esc[256];
    char alert_esc[320];
    char ai_reply_esc[320];
    char profile_source_esc[64];
    char profile_summary_esc[256];
    weather_runtime_info_t weather_rt = {0};
    plant_profile_runtime_info_t profile_rt = {0};
    char weather_obs_esc[80];

    get_weather_runtime_info(&weather_rt);
    get_plant_profile_runtime_info(&profile_rt);
    json_escape_text(weather_rt.weather.report_time[0] ? weather_rt.weather.report_time : "-",
                     weather_obs_esc,
                     sizeof(weather_obs_esc));

    json_escape_text(g_monitor_plant, plant_esc, sizeof(plant_esc));
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    json_escape_text(g_last_alert, alert_esc, sizeof(alert_esc));
    json_escape_text(g_last_ai_reply, ai_reply_esc, sizeof(ai_reply_esc));
    json_escape_text(profile_rt.profile.source, profile_source_esc, sizeof(profile_source_esc));
    json_escape_text(profile_rt.profile.summary, profile_summary_esc, sizeof(profile_summary_esc));

    if (has_data) {
        snprintf(buf, sizeof(buf),
            "{\"seq\":%u,\"lux\":%.2f,\"env_temp\":%.1f,\"env_humi\":%.1f,"
            "\"press\":%.1f,\"soil_temp\":%.1f,\"soil_humi\":%.1f,"
            "\"ph\":%.2f,\"n\":%u,\"p\":%u,\"k\":%u,"
            "\"ok\":%lu,\"fail\":%lu,"
            "\"plant\":\"%s\",\"updated\":%lu,"
            "\"monitor_target\":\"%s\","
            "\"sample_time_s\":%lu,\"weather_fetch_time_s\":%lu,\"weather_observe_time\":\"%.63s\","
            "\"weather_age_s\":%lu,\"weather_stale\":%s,\"cloud_report_time_s\":%lu,"
            "\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,"
            "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\","
            "\"profile_ready\":%s,\"profile_matches_current\":%s,\"profile_age_s\":%lu,"
            "\"profile_source\":\"%.48s\",\"profile_summary\":\"%.220s\","
            "\"profile_press_min\":%.1f,\"profile_press_max\":%.1f,"
            "\"profile_temp_min\":%.1f,\"profile_temp_max\":%.1f,"
            "\"profile_air_humi_min\":%.1f,\"profile_air_humi_max\":%.1f,"
            "\"profile_soil_temp_min\":%.1f,\"profile_soil_temp_max\":%.1f,"
            "\"profile_soil_humi_min\":%.1f,\"profile_soil_humi_max\":%.1f,"
            "\"profile_ph_min\":%.1f,\"profile_ph_max\":%.1f,"
            "\"profile_n_min\":%.1f,\"profile_n_max\":%.1f,"
            "\"profile_p_min\":%.1f,\"profile_p_max\":%.1f,"
            "\"profile_k_min\":%.1f,\"profile_k_max\":%.1f,"
            "\"profile_lux_min\":%.0f,\"profile_lux_max\":%.0f}",
            latest_pkt.seq, latest_pkt.lux, latest_pkt.env_temp_c, latest_pkt.env_humi_pct,
            latest_pkt.press_kpa, latest_pkt.soil_temp_c, latest_pkt.soil_humi_pct,
            latest_pkt.ph, latest_pkt.n, latest_pkt.p, latest_pkt.k,
            (unsigned long)ok_count, (unsigned long)fail_count,
            plant_esc, (unsigned long)g_control_state.update_time_s,
            plant_esc,
            (unsigned long)g_last_sensor_sample_time_s,
            (unsigned long)weather_rt.weather.update_time_s,
            weather_obs_esc,
            (unsigned long)weather_rt.weather_age_s,
            weather_rt.weather_stale ? "true" : "false",
            (unsigned long)g_last_cloud_report_time_s,
            g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
            g_control_state.fan_speed,
            g_control_state.pump_speed,
            g_control_state.servo_angle,
            g_control_state.light_level,
            reason_esc,
            alert_esc,
            ai_reply_esc,
            profile_rt.ready ? "true" : "false",
            profile_rt.matches_current ? "true" : "false",
            (unsigned long)profile_rt.age_s,
            profile_source_esc,
            profile_summary_esc,
            profile_rt.profile.press_min,
            profile_rt.profile.press_max,
            profile_rt.profile.env_temp_min,
            profile_rt.profile.env_temp_max,
            profile_rt.profile.env_humi_min,
            profile_rt.profile.env_humi_max,
            profile_rt.profile.soil_temp_min,
            profile_rt.profile.soil_temp_max,
            profile_rt.profile.soil_humi_min,
            profile_rt.profile.soil_humi_max,
            profile_rt.profile.ph_min,
            profile_rt.profile.ph_max,
            profile_rt.profile.n_min,
            profile_rt.profile.n_max,
            profile_rt.profile.p_min,
            profile_rt.profile.p_max,
            profile_rt.profile.k_min,
            profile_rt.profile.k_max,
            profile_rt.profile.lux_min,
            profile_rt.profile.lux_max);
    } else {
        snprintf(buf, sizeof(buf),
            "{\"seq\":0,\"lux\":0,\"env_temp\":0,\"env_humi\":0,"
            "\"press\":0,\"soil_temp\":0,\"soil_humi\":0,"
            "\"ph\":0,\"n\":0,\"p\":0,\"k\":0,"
            "\"ok\":0,\"fail\":0,"
            "\"plant\":\"%s\",\"updated\":%lu,"
            "\"monitor_target\":\"%s\","
            "\"sample_time_s\":%lu,\"weather_fetch_time_s\":%lu,\"weather_observe_time\":\"%.63s\","
            "\"weather_age_s\":%lu,\"weather_stale\":%s,\"cloud_report_time_s\":%lu,"
            "\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,"
            "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\","
            "\"profile_ready\":%s,\"profile_matches_current\":%s,\"profile_age_s\":%lu,"
            "\"profile_source\":\"%.48s\",\"profile_summary\":\"%.220s\","
            "\"profile_press_min\":%.1f,\"profile_press_max\":%.1f,"
            "\"profile_temp_min\":%.1f,\"profile_temp_max\":%.1f,"
            "\"profile_air_humi_min\":%.1f,\"profile_air_humi_max\":%.1f,"
            "\"profile_soil_temp_min\":%.1f,\"profile_soil_temp_max\":%.1f,"
            "\"profile_soil_humi_min\":%.1f,\"profile_soil_humi_max\":%.1f,"
            "\"profile_ph_min\":%.1f,\"profile_ph_max\":%.1f,"
            "\"profile_n_min\":%.1f,\"profile_n_max\":%.1f,"
            "\"profile_p_min\":%.1f,\"profile_p_max\":%.1f,"
            "\"profile_k_min\":%.1f,\"profile_k_max\":%.1f,"
            "\"profile_lux_min\":%.0f,\"profile_lux_max\":%.0f}",
            plant_esc, (unsigned long)g_control_state.update_time_s,
            plant_esc,
            (unsigned long)g_last_sensor_sample_time_s,
            (unsigned long)weather_rt.weather.update_time_s,
            weather_obs_esc,
            (unsigned long)weather_rt.weather_age_s,
            weather_rt.weather_stale ? "true" : "false",
            (unsigned long)g_last_cloud_report_time_s,
            g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
            g_control_state.fan_speed,
            g_control_state.pump_speed,
            g_control_state.servo_angle,
            g_control_state.light_level,
            reason_esc,
            alert_esc,
            ai_reply_esc,
            profile_rt.ready ? "true" : "false",
            profile_rt.matches_current ? "true" : "false",
            (unsigned long)profile_rt.age_s,
            profile_source_esc,
            profile_summary_esc,
            profile_rt.profile.press_min,
            profile_rt.profile.press_max,
            profile_rt.profile.env_temp_min,
            profile_rt.profile.env_temp_max,
            profile_rt.profile.env_humi_min,
            profile_rt.profile.env_humi_max,
            profile_rt.profile.soil_temp_min,
            profile_rt.profile.soil_temp_max,
            profile_rt.profile.soil_humi_min,
            profile_rt.profile.soil_humi_max,
            profile_rt.profile.ph_min,
            profile_rt.profile.ph_max,
            profile_rt.profile.n_min,
            profile_rt.profile.n_max,
            profile_rt.profile.p_min,
            profile_rt.profile.p_max,
            profile_rt.profile.k_min,
            profile_rt.profile.k_max,
            profile_rt.profile.lux_min,
            profile_rt.profile.lux_max);
    }
    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t api_control_get_handler(httpd_req_t *req)
{
    char buf[2048];
    char plant_esc[128];
    char reason_esc[256];
    char alert_esc[320];
    char ai_reply_esc[320];
    char profile_source_esc[64];
    char profile_summary_esc[256];
    weather_runtime_info_t weather_rt = {0};
    plant_profile_runtime_info_t profile_rt = {0};
    char weather_obs_esc[80];

    get_weather_runtime_info(&weather_rt);
    get_plant_profile_runtime_info(&profile_rt);
    json_escape_text(weather_rt.weather.report_time[0] ? weather_rt.weather.report_time : "-",
                     weather_obs_esc,
                     sizeof(weather_obs_esc));

    json_escape_text(g_monitor_plant, plant_esc, sizeof(plant_esc));
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    json_escape_text(g_last_alert, alert_esc, sizeof(alert_esc));
    json_escape_text(g_last_ai_reply, ai_reply_esc, sizeof(ai_reply_esc));
    json_escape_text(profile_rt.profile.source, profile_source_esc, sizeof(profile_source_esc));
    json_escape_text(profile_rt.profile.summary, profile_summary_esc, sizeof(profile_summary_esc));
    snprintf(buf, sizeof(buf),
             "{\"plant\":\"%s\",\"monitor_target\":\"%s\",\"mode\":\"%s\",\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,"
             "\"updated\":%lu,\"sample_time_s\":%lu,\"weather_fetch_time_s\":%lu,\"weather_observe_time\":\"%.63s\","
             "\"weather_age_s\":%lu,\"weather_stale\":%s,\"cloud_report_time_s\":%lu,"
             "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\","
             "\"profile_ready\":%s,\"profile_matches_current\":%s,\"profile_age_s\":%lu,"
             "\"profile_source\":\"%.48s\",\"profile_summary\":\"%.220s\"}",
             plant_esc,
             plant_esc,
             g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
             g_control_state.fan_speed,
             g_control_state.pump_speed,
             g_control_state.servo_angle,
             g_control_state.light_level,
             (unsigned long)g_control_state.update_time_s,
             (unsigned long)g_last_sensor_sample_time_s,
             (unsigned long)weather_rt.weather.update_time_s,
             weather_obs_esc,
             (unsigned long)weather_rt.weather_age_s,
             weather_rt.weather_stale ? "true" : "false",
             (unsigned long)g_last_cloud_report_time_s,
             reason_esc,
             alert_esc,
             ai_reply_esc,
             profile_rt.ready ? "true" : "false",
             profile_rt.matches_current ? "true" : "false",
             (unsigned long)profile_rt.age_s,
             profile_source_esc,
             profile_summary_esc);
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
            snprintf(g_control_state.reason, sizeof(g_control_state.reason), "网页切换智能模式");
            g_smart_control_force_run = true;
        } else {
            g_control_mode = CONTROL_MODE_MANUAL;
            snprintf(g_control_state.reason, sizeof(g_control_state.reason), "网页切换手动模式");
        }
        g_control_state.update_time_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        mode_changed = (g_control_mode != prev_mode);
    }

    if (mode_changed) {
        // 切换模式时强制先下发一次停止指令
        apply_control_and_send(g_control_mode, 0, 0, 0, 0, "mode switch stop");
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
#define MQTT_PUB_TASK_STACK_SIZE   8192
#define MQTT_PAYLOAD_BUF_SIZE      1792
static char g_mqtt_payload_buf[MQTT_PAYLOAD_BUF_SIZE];

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
        // 静默处理，不输出日志
        break;
    case MQTT_EVENT_DATA:
        // 静默处理，不输出日志
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

    int fan_report = g_ctrl_ack.valid ? g_ctrl_ack.fan_actual : g_control_state.fan_speed;
    int pump_report = g_ctrl_ack.valid ? g_ctrl_ack.pump_actual : g_control_state.pump_speed;
    int servo_report = g_ctrl_ack.valid ? g_ctrl_ack.servo_actual : g_control_state.servo_angle;
    int light_report = g_ctrl_ack.valid ? g_ctrl_ack.light_actual : g_control_state.light_level;
    g_last_cloud_report_time_s = monotonic_time_s();

    char *payload = g_mqtt_payload_buf;
    int written = snprintf(payload, MQTT_PAYLOAD_BUF_SIZE,
        "{\"id\":\"%lu\",\"version\":\"1.0\",\"params\":{"
        "\"lux\":{\"value\":%.2f},"
        "\"env_temp\":{\"value\":%.1f},"
        "\"env_humi\":{\"value\":%.1f},"
        "\"press\":{\"value\":%.1f},"
        "\"soil_temp\":{\"value\":%.1f},"
        "\"soil_humi\":{\"value\":%.1f},"
        "\"ph\":{\"value\":%.2f},"
        "\"n\":{\"value\":%.1f},"
        "\"p\":{\"value\":%u},"
        "\"k\":{\"value\":%.1f},"
        "\"fan\":{\"value\":%d},"
        "\"pump\":{\"value\":%d},"
        "\"servo\":{\"value\":%d},"
        "\"light\":{\"value\":%d}}}",
        (unsigned long)msg_id,
        latest_pkt.lux,
        latest_pkt.env_temp_c,
        latest_pkt.env_humi_pct,
        latest_pkt.press_kpa,
        latest_pkt.soil_temp_c,
        latest_pkt.soil_humi_pct,
        latest_pkt.ph,
        (float)latest_pkt.n,
        latest_pkt.p,
        (float)latest_pkt.k,
        fan_report,
        pump_report,
        servo_report,
        light_report);
    if (written < 0 || written >= MQTT_PAYLOAD_BUF_SIZE) {
        ESP_LOGE(TAG, "MQTT payload 生成失败或被截断, len=%d", written);
        return;
    }

    if (!has_data) {
        ESP_LOGW(TAG, "尚未收到 LoRa 实时帧，OneNet 传感器值为当前缓存");
    }

    // 静默上报，不输出详细日志
    esp_mqtt_client_publish(mqtt_client, ONENET_TOPIC_POST, payload, 0, 1, 0);
}

static void mqtt_pub_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));  /* 启动后等5秒(确保MQTT已连接) */
    while (1) {
        if (mqtt_pending_send && mqtt_connected) {
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
    xTaskCreate(mqtt_pub_task, "mqtt_pub", MQTT_PUB_TASK_STACK_SIZE, NULL, 5, NULL);
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
static void emergency_fast_control_from_sensor(const sensor_packet_t *pkt);
static bool sensor_change_requires_fast_control(const sensor_packet_t *prev_pkt,
                                                const sensor_packet_t *cur_pkt,
                                                bool had_prev_data,
                                                bool prev_alert,
                                                bool cur_alert);

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
                // 仅在无数据时输出警告，正常运行时不输出统计
                if (delta_rx == 0 && delta_aux_low == 0 && delta_rx_pin_low == 0) {
                    ESP_LOGW(TAG, "未见AUX活动，接收模块可能没有收到空中数据");
                } else if (delta_rx == 0 && delta_aux_low > 0) {
                    ESP_LOGW(TAG, "AUX有活动但UART无字节，请检查接线");
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
                    sensor_packet_t prev_pkt = latest_pkt;
                    bool had_prev_data = has_data;
                    bool prev_alert = (g_last_alert[0] != '\0');
                    latest_pkt = pkt;
                    has_data = true;
                    g_last_sensor_sample_time_s = monotonic_time_s();
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
                        char alert[512] = "";
                        check_alerts(&pkt, alert, sizeof(alert));
                        snprintf(g_last_alert, sizeof(g_last_alert), "%s", alert);
                        lcd_update(&dd, true, monitor_target_for_lcd(),
                                   mqtt_connected, mqtt_connected, alert);

                        /* 智能模式下，若环境恢复正常，自动发送停机指令 */
                        if (g_control_mode == CONTROL_MODE_SMART) {
                            bool is_alert = (alert[0] != '\0');
                            emergency_fast_control_from_sensor(&pkt);
                            uint32_t now_s = monotonic_time_s();
                            static uint32_t s_last_fast_trigger_s = 0;

                            bool need_fast_control = sensor_change_requires_fast_control(&prev_pkt,
                                                                                         &pkt,
                                                                                         had_prev_data,
                                                                                         prev_alert,
                                                                                         is_alert);
                            if (need_fast_control &&
                                (now_s - s_last_fast_trigger_s) >= SMART_CONTROL_MIN_TRIGGER_GAP_S) {
                                g_smart_control_force_run = true;
                                s_last_fast_trigger_s = now_s;
                                ESP_LOGI(TAG, "传感器变化触发快速智能评估");
                            }

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
static spark_chat_client_t g_spark = {0};
static spark_chat_client_t g_smart_ctrl = {0};
static spark_chat_client_t g_profile_lookup = {0};
static baidu_tts_handle_t  g_tts = {0};
static SemaphoreHandle_t g_ai_lock = NULL;
static bool g_ai_clients_ready = false;
static bool g_tts_ready = false;
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

    spark_chat_clear_history(&g_spark);
    spark_chat_clear_history(&g_smart_ctrl);

    snprintf(ctrl_sys_prompt, sizeof(ctrl_sys_prompt),
        "你是%s温室执行器控制助手。根据实时环境和天气数据输出JSON控制量。"
        "禁止解释，禁止多余文本，只能输出JSON，reason字段必须用中文。"
        "fan和pump范围0-100，servo范围0-180。",
        g_monitor_plant);
    spark_chat_add_message(&g_smart_ctrl, "system", ctrl_sys_prompt);

    snprintf(ai_sys_prompt, sizeof(ai_sys_prompt),
        "你是%s种植智慧农业助手。监测对象是%s。"
        "请联网搜索%s的最佳种植条件，"
        "根据搜索结果直接分析传感器和天气数据。"
        "如有异常给出针对%s的建议。"
        "回答极简，不超过50字，不要客套。",
        g_monitor_plant, g_monitor_plant, g_monitor_plant, g_monitor_plant);
    spark_chat_add_message(&g_spark, "system", ai_sys_prompt);
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

    const char *plant_value = json_get_first_string_value(root,
                                                          "plant",
                                                          "monitor_target",
                                                          "target",
                                                          "name");
    if (!(plant_value && plant_value[0])) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "plant required");
        return ESP_FAIL;
    }

    char new_plant[sizeof(g_monitor_plant)] = {0};
    snprintf(new_plant, sizeof(new_plant), "%s", plant_value);
    trim_text_inplace(new_plant);
    if (new_plant[0] == '\0') {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "plant empty");
        return ESP_FAIL;
    }

    bool changed = (strcmp(g_monitor_plant, new_plant) != 0);
    if (changed) {
        snprintf(g_monitor_plant, sizeof(g_monitor_plant), "%s", new_plant);
        request_plant_profile_refresh(g_monitor_plant);

        if (g_ai_clients_ready && ai_lock_take(pdMS_TO_TICKS(3000))) {
            reset_ai_prompts_locked();
            ai_lock_give();
        }

        apply_control_and_send(g_control_mode, 0, 0, 0, 0, "plant switch stop");
        g_smart_control_force_run = true;
        ESP_LOGI(TAG, "监控对象已切换: %s，已清零并触发智能调节", g_monitor_plant);
    }

    cJSON_Delete(root);

    char plant_esc[128];
    char reason_esc[256];
    char alert_esc[320];
    char ai_reply_esc[320];
    char profile_source_esc[64];
    char profile_summary_esc[256];
    plant_profile_runtime_info_t profile_rt = {0};
    get_plant_profile_runtime_info(&profile_rt);
    json_escape_text(g_monitor_plant, plant_esc, sizeof(plant_esc));
    json_escape_text(g_control_state.reason, reason_esc, sizeof(reason_esc));
    json_escape_text(g_last_alert, alert_esc, sizeof(alert_esc));
    json_escape_text(g_last_ai_reply, ai_reply_esc, sizeof(ai_reply_esc));
    json_escape_text(profile_rt.profile.source, profile_source_esc, sizeof(profile_source_esc));
    json_escape_text(profile_rt.profile.summary, profile_summary_esc, sizeof(profile_summary_esc));
    char resp[1536];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"changed\":%s,\"plant\":\"%s\",\"monitor_target\":\"%s\",\"mode\":\"%s\","
             "\"fan\":%u,\"pump\":%u,\"servo\":%u,\"light\":%u,\"updated\":%lu,"
             "\"reason\":\"%.180s\",\"alert\":\"%.220s\",\"ai_reply\":\"%.220s\","
             "\"profile_ready\":%s,\"profile_matches_current\":%s,\"profile_age_s\":%lu,"
             "\"profile_source\":\"%.48s\",\"profile_summary\":\"%.220s\"}",
             changed ? "true" : "false",
             plant_esc,
             plant_esc,
             g_control_mode == CONTROL_MODE_SMART ? "smart" : "manual",
             g_control_state.fan_speed,
             g_control_state.pump_speed,
             g_control_state.servo_angle,
             g_control_state.light_level,
             (unsigned long)g_control_state.update_time_s,
             reason_esc,
             alert_esc,
             ai_reply_esc,
             profile_rt.ready ? "true" : "false",
             profile_rt.matches_current ? "true" : "false",
             (unsigned long)profile_rt.age_s,
             profile_source_esc,
             profile_summary_esc);
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
#define ALERT_SOIL_HUMI_CRITICAL 5.0f /* 土壤极干，立即兜底灌溉 % */
#define ALERT_SOIL_HUMI_HIGH 90.0f   /* 土壤过湿 % */
#define ALERT_SOIL_TEMP_HIGH 40.0f   /* 土壤温度过高 ℃ */
#define ALERT_PH_LOW          3.5f   /* pH过酸 */
#define ALERT_PH_HIGH         9.0f   /* pH过碱 */
#define ALERT_LUX_HIGH    120000.0f  /* 光照过强 lux */

#define NORMAL_REPORT_INTERVAL_S  300   /* 正常时5分钟播报一次 */
#define ALERT_REPORT_INTERVAL_S    60   /* 异常时1分钟播报一次 */
#define SMART_CONTROL_INTERVAL_S   10   /* 智能控制评估周期 */

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
        if (light_j && cJSON_IsNumber(light_j)) {
            *light = light_j->valueint;
        }
        if (reason && reason_size > 0) {
            if (reason_j && cJSON_IsString(reason_j) && reason_j->valuestring) {
                snprintf(reason, reason_size, "%s", reason_j->valuestring);
            } else {
                snprintf(reason, reason_size, "智能调节");
            }
        }
    }

    cJSON_Delete(root);
    return ok;
}

static void append_alert_text(char *buf, size_t size, const char *fmt, ...)
{
    if (!buf || size == 0 || !fmt) {
        return;
    }

    size_t used = strlen(buf);
    if (used >= size - 1) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf + used, size - used, fmt, args);
    va_end(args);
    
    /* 确保不截断在 UTF-8 字符中间 */
    size_t final_len = strlen(buf);
    while (final_len > 0 && (buf[final_len - 1] & 0xC0) == 0x80) {
        /* 最后一个字节是 UTF-8 continuation byte (10xxxxxx)，可能是截断的 */
        /* 向前查找这个字符的起始字节 */
        size_t start = final_len - 1;
        while (start > 0 && (buf[start] & 0xC0) == 0x80) {
            start--;
        }
        /* 检查这个 UTF-8 序列是否完整 */
        unsigned char lead = (unsigned char)buf[start];
        size_t expected_len = 0;
        if ((lead & 0x80) == 0) {
            expected_len = 1;  /* ASCII */
        } else if ((lead & 0xE0) == 0xC0) {
            expected_len = 2;  /* 2-byte UTF-8 */
        } else if ((lead & 0xF0) == 0xE0) {
            expected_len = 3;  /* 3-byte UTF-8 (中文) */
        } else if ((lead & 0xF8) == 0xF0) {
            expected_len = 4;  /* 4-byte UTF-8 */
        }
        if (expected_len > 0 && (final_len - start) == expected_len) {
            break;  /* 完整的 UTF-8 字符，无需截断 */
        }
        /* 不完整，移除这个字符 */
        buf[start] = '\0';
        final_len = start;
    }
}

#if 0
static void build_smart_control_prompt(char *buf, size_t size)
{
    const char *alert_text = g_last_alert[0] ? g_last_alert : "无";
    weather_runtime_info_t weather_rt = {0};
    get_weather_runtime_info(&weather_rt);

    const char *weather_obs_time = weather_rt.weather.report_time[0] ? weather_rt.weather.report_time : "-";
    uint32_t sample_time_s = g_last_sensor_sample_time_s;
    uint32_t cloud_report_time_s = g_last_cloud_report_time_s;

    char weather_text[192];
    build_weather_prompt_text(weather_text, sizeof(weather_text));
    plant_profile_runtime_info_t profile_rt_unused = {0};
    get_plant_profile_runtime_info(&profile_rt_unused);
    char profile_text[512];
    if (profile_rt_unused.ready && profile_rt_unused.matches_current) {
        snprintf(profile_text, sizeof(profile_text),
                 "目标范围:气压%.1f-%.1fkPa,环境温度%.1f-%.1f℃,环境湿度%.0f-%.0f%%,土壤温度%.1f-%.1f℃,土壤湿度%.0f-%.0f%%,pH%.1f-%.1f,N%.1f-%.1f,P%.1f-%.1f,K%.1f-%.1f,光照%.0f-%.0flux。资料:%s。",
                 profile_rt_unused.profile.press_min,
                 profile_rt_unused.profile.press_max,
                 profile_rt_unused.profile.env_temp_min,
                 profile_rt_unused.profile.env_temp_max,
                 profile_rt_unused.profile.env_humi_min,
                 profile_rt_unused.profile.env_humi_max,
                 profile_rt_unused.profile.soil_temp_min,
                 profile_rt_unused.profile.soil_temp_max,
                 profile_rt_unused.profile.soil_humi_min,
                 profile_rt_unused.profile.soil_humi_max,
                 profile_rt_unused.profile.ph_min,
                 profile_rt_unused.profile.ph_max,
                 profile_rt_unused.profile.n_min,
                 profile_rt_unused.profile.n_max,
                 profile_rt_unused.profile.p_min,
                 profile_rt_unused.profile.p_max,
                 profile_rt_unused.profile.k_min,
                 profile_rt_unused.profile.k_max,
                 profile_rt_unused.profile.lux_min,
                 profile_rt_unused.profile.lux_max,
                 profile_rt_unused.profile.summary);
    } else {
        snprintf(profile_text, sizeof(profile_text), "植物目标范围正在检索中。");
    }

    plant_profile_runtime_info_t profile_rt = {0};
    get_plant_profile_runtime_info(&profile_rt);

    char profile_text[640];
    if (profile_rt.ready && profile_rt.matches_current) {
        snprintf(profile_text, sizeof(profile_text),
                 "plant_profile:%.24s press=%.1f-%.1fkPa temp=%.1f-%.1fC air_humi=%.0f-%.0f soil_temp=%.1f-%.1fC soil_humi=%.0f-%.0f ph=%.1f-%.1f N=%.1f-%.1f P=%.1f-%.1f K=%.1f-%.1f lux=%.0f-%.0f summary=%.96s source=%.16s age_s=%lu",
                 profile_rt.profile.plant,
                 profile_rt.profile.press_min,
                 profile_rt.profile.press_max,
                 profile_rt.profile.env_temp_min,
                 profile_rt.profile.env_temp_max,
                 profile_rt.profile.env_humi_min,
                 profile_rt.profile.env_humi_max,
                 profile_rt.profile.soil_temp_min,
                 profile_rt.profile.soil_temp_max,
                 profile_rt.profile.soil_humi_min,
                 profile_rt.profile.soil_humi_max,
                 profile_rt.profile.ph_min,
                 profile_rt.profile.ph_max,
                 profile_rt.profile.n_min,
                 profile_rt.profile.n_max,
                 profile_rt.profile.p_min,
                 profile_rt.profile.p_max,
                 profile_rt.profile.k_min,
                 profile_rt.profile.k_max,
                 profile_rt.profile.lux_min,
                 profile_rt.profile.lux_max,
                 profile_rt.profile.summary,
                 profile_rt.profile.source,
                 (unsigned long)profile_rt.age_s);
    } else {
        snprintf(profile_text, sizeof(profile_text),
                 "plant_profile_pending=1 current=%.24s use conservative strategy", g_monitor_plant);
    }

    int written = snprintf(buf, size,
        "你是%s温室执行器控制助手。"
        "仅输出JSON，禁止解释。"
        "格式:{\"fan\":0-100,\"pump\":0-100,\"servo\":0-180,\"light\":0-100,\"reason\":\"<=20字\"}。"
        "数据:lux=%.0f,et=%.1f,eh=%.0f,st=%.1f,sh=%.0f,ph=%.2f,N=%u,P=%u,K=%u。"
        "当前执行器:fan=%u,pump=%u,servo=%u,light=%u。"
        "告警:%.120s。"
        "天气:%s。"
        "时间戳:sample=%lu,weather_obs=%.31s,cloud=%lu。"
        "weather_age_s=%lu,weather_stale=%d。"
        "规则:高温或强光增大风扇,土壤偏干提高水泵,土壤过湿降低或关闭水泵,低光照可增大补光灯,阴雨高湿时避免过度灌溉。"
        "当weather_stale=1时按保守策略,避免执行器大幅拉升。",
        g_monitor_plant,
        latest_pkt.lux,
        latest_pkt.env_temp_c,
        latest_pkt.env_humi_pct,
        latest_pkt.soil_temp_c,
        latest_pkt.soil_humi_pct,
        latest_pkt.ph,
        latest_pkt.n,
        latest_pkt.p,
        latest_pkt.k,
        g_control_state.fan_speed,
        g_control_state.pump_speed,
        g_control_state.servo_angle,
        g_control_state.light_level,
        alert_text,
        weather_text,
        (unsigned long)sample_time_s,
        weather_obs_time,
        (unsigned long)cloud_report_time_s,
        (unsigned long)weather_rt.weather_age_s,
        weather_rt.weather_stale ? 1 : 0);

    written = snprintf(buf, size,
        "你是%s温室执行器控制助手。"
        "只输出JSON，不要解释。"
        "格式:{\"fan\":0-100,\"pump\":0-100,\"servo\":0-180,\"light\":0-100,\"reason\":\"<=20字\"}。"
        "数据:lux=%.0f,et=%.1f,eh=%.0f,press=%.1f,st=%.1f,sh=%.0f,ph=%.2f,N=%u,P=%u,K=%u。"
        "当前执行器:fan=%u,pump=%u,servo=%u,light=%u。"
        "告警:%.120s。天气:%s。"
        "时间戳:sample=%lu,weather_obs=%.31s,cloud=%lu。weather_age_s=%lu,weather_stale=%d。"
        "规则:仅可控制风扇、水泵、补光灯和舵机遮阳。"
        "土壤湿度越低水泵越大，高温、土温过高或强光时增大风扇并加强遮阳，光照不足时增加补光。"
        "pH、N、P、K、气压异常无法直接执行，只能在reason中简述人工处理建议。"
        "weather_stale=1时采取保守策略，避免执行器大幅拉升。",
        g_monitor_plant,
        latest_pkt.lux,
        latest_pkt.env_temp_c,
        latest_pkt.env_humi_pct,
        latest_pkt.press_kpa,
        latest_pkt.soil_temp_c,
        latest_pkt.soil_humi_pct,
        latest_pkt.ph,
        latest_pkt.n,
        latest_pkt.p,
        latest_pkt.k,
        g_control_state.fan_speed,
        g_control_state.pump_speed,
        g_control_state.servo_angle,
        g_control_state.light_level,
        alert_text,
        weather_text,
        (unsigned long)sample_time_s,
        weather_obs_time,
        (unsigned long)cloud_report_time_s,
        (unsigned long)weather_rt.weather_age_s,
        weather_rt.weather_stale ? 1 : 0);

    if (written > 0 && (size_t)written < size) {
        written += snprintf(buf + written, size - (size_t)written, " %s.", profile_text);
    }

    if (written < 0) {
        buf[0] = '\0';
        return;
    }
    if ((size_t)written >= size) {
        ESP_LOGW(TAG, "智能控制prompt被截断 len=%d size=%u", written, (unsigned)size);
    }
}

#endif

static void build_profile_prompt_text(char *buf, size_t size, bool compact)
{
    if (!buf || size == 0) {
        return;
    }

    plant_profile_runtime_info_t profile_rt = {0};
    get_plant_profile_runtime_info(&profile_rt);

    if (profile_rt.ready && profile_rt.matches_current) {
        snprintf(buf, size,
                 compact
                     ? "profile target=%.24s press=%.1f-%.1fkPa env_temp=%.1f-%.1fC env_humi=%.0f-%.0f soil_temp=%.1f-%.1fC soil_humi=%.0f-%.0f ph=%.1f-%.1f N=%.1f-%.1f P=%.1f-%.1f K=%.1f-%.1f lux=%.0f-%.0f source=%.16s age_s=%lu summary=%.96s"
                     : "Suitable range for %.24s: pressure %.1f-%.1fkPa, ambient temperature %.1f-%.1fC, ambient humidity %.0f-%.0f%%, soil temperature %.1f-%.1fC, soil humidity %.0f-%.0f%%, pH %.1f-%.1f, N %.1f-%.1f, P %.1f-%.1f, K %.1f-%.1f, light %.0f-%.0f lux. Source %.16s, age %lu s, summary %.96s.",
                 profile_rt.profile.plant,
                 profile_rt.profile.press_min,
                 profile_rt.profile.press_max,
                 profile_rt.profile.env_temp_min,
                 profile_rt.profile.env_temp_max,
                 profile_rt.profile.env_humi_min,
                 profile_rt.profile.env_humi_max,
                 profile_rt.profile.soil_temp_min,
                 profile_rt.profile.soil_temp_max,
                 profile_rt.profile.soil_humi_min,
                 profile_rt.profile.soil_humi_max,
                 profile_rt.profile.ph_min,
                 profile_rt.profile.ph_max,
                 profile_rt.profile.n_min,
                 profile_rt.profile.n_max,
                 profile_rt.profile.p_min,
                 profile_rt.profile.p_max,
                 profile_rt.profile.k_min,
                 profile_rt.profile.k_max,
                 profile_rt.profile.lux_min,
                 profile_rt.profile.lux_max,
                 profile_rt.profile.source,
                 (unsigned long)profile_rt.age_s,
                 profile_rt.profile.summary);
        return;
    }

    snprintf(buf, size,
             compact
                 ? "profile pending target=%.24s use conservative strategy"
                 : "Suitable-range profile for %.24s is still loading. Use conservative greenhouse advice until the profile is ready.",
             g_monitor_plant);
}

static void build_smart_control_prompt(char *buf, size_t size)
{
    if (!buf || size == 0) {
        return;
    }

    const char *alert_text = g_last_alert[0] ? g_last_alert : "normal";

    char profile_text[640];
    build_profile_prompt_text(profile_text, sizeof(profile_text), true);

    /* 安全截断 alert_text 以避免 UTF-8 中间字节被截断 */
    char safe_alert[512];
    {
        size_t max_len = sizeof(safe_alert) - 1;
        size_t src_len = strlen(alert_text);
        if (src_len <= max_len) {
            strcpy(safe_alert, alert_text);
        } else {
            /* 回退到最近的 UTF-8 字符边界 */
            size_t cut = max_len;
            while (cut > 0 && (alert_text[cut] & 0xC0) == 0x80) {
                cut--;
            }
            memcpy(safe_alert, alert_text, cut);
            safe_alert[cut] = '\0';
        }
    }

    int written = snprintf(
        buf,
        size,
        "You are a greenhouse actuator decision assistant for %s. "
        "Return JSON only with format "
        "{\"fan\":0-100,\"pump\":0-100,\"servo\":0-180,\"light\":0-100,\"reason\":\"中文简短建议<=50字\"}. "
        "Current sensor data: light=%.0f lux, env_temp=%.1f C, env_humi=%.0f%%, pressure=%.1f kPa, soil_temp=%.1f C, soil_humi=%.0f%%, pH=%.2f, N=%u, P=%u, K=%u. "
        "Current actuators: fan=%u, pump=%u, servo=%u, light=%u. "
        "Alerts: %s. "
        "Only four actuators can be changed: fan, pump, light, servo shade. "
        "Lower soil humidity means higher pump level. Excess heat or strong light should increase fan and shade. Low light can increase grow light. reason必须用中文回复。%s",
        g_monitor_plant,
        latest_pkt.lux,
        latest_pkt.env_temp_c,
        latest_pkt.env_humi_pct,
        latest_pkt.press_kpa,
        latest_pkt.soil_temp_c,
        latest_pkt.soil_humi_pct,
        latest_pkt.ph,
        latest_pkt.n,
        latest_pkt.p,
        latest_pkt.k,
        g_control_state.fan_speed,
        g_control_state.pump_speed,
        g_control_state.servo_angle,
        g_control_state.light_level,
        safe_alert,
        profile_text);

    if (written < 0) {
        buf[0] = '\0';
        return;
    }
    if ((size_t)written >= size) {
        ESP_LOGW(TAG, "smart control prompt truncated len=%d size=%u", written, (unsigned)size);
    }
}

static void build_fallback_control(const sensor_packet_t *pkt,
                                   int *fan, int *pump, int *servo, int *light,
                                   char *reason, size_t reason_size)
{
    int f = 20;
    int p = 0;
    int s = 90;
    int l = 0;
    float temp_hot = 32.0f;
    float temp_warm = 28.0f;
    float soil_temp_hot = 35.0f;
    float soil_temp_warm = 30.0f;
    float lux_hot = 60000.0f;
    float lux_warm = 30000.0f;
    float soil_dry = 15.0f;
    float soil_low = 30.0f;
    float soil_high = 80.0f;

    plant_profile_runtime_info_t profile_rt = {0};
    get_plant_profile_runtime_info(&profile_rt);
    if (profile_rt.ready && profile_rt.matches_current) {
        temp_warm = profile_rt.profile.env_temp_max;
        temp_hot = profile_rt.profile.env_temp_max + 3.0f;
        soil_temp_warm = profile_rt.profile.soil_temp_max;
        soil_temp_hot = profile_rt.profile.soil_temp_max + 3.0f;
        lux_warm = profile_rt.profile.lux_max;
        lux_hot = profile_rt.profile.lux_max * 1.5f;
        soil_dry = profile_rt.profile.soil_humi_min;
        soil_low = profile_rt.profile.soil_humi_min + 8.0f;
        soil_high = profile_rt.profile.soil_humi_max;
    }

    if (pkt->env_temp_c >= temp_hot || pkt->soil_temp_c >= soil_temp_hot || pkt->lux >= lux_hot) {
        f = 80;
        s = 130;
    } else if (pkt->env_temp_c >= temp_warm || pkt->soil_temp_c >= soil_temp_warm || pkt->lux >= lux_warm) {
        f = 55;
        s = 110;
    }

    if (pkt->soil_humi_pct <= soil_dry) {
        float dryness = soil_dry > 0.1f ? (soil_dry - pkt->soil_humi_pct) / soil_dry : 0.0f;
        if (dryness < 0.0f) dryness = 0.0f;
        p = 60 + (int)(dryness * 40.0f);
        if (p > 100) p = 100;
    } else if (pkt->soil_humi_pct <= soil_low) {
        float span = soil_low - soil_dry;
        float ratio = span > 0.1f ? (soil_low - pkt->soil_humi_pct) / span : 0.0f;
        if (ratio < 0.0f) ratio = 0.0f;
        p = 25 + (int)(ratio * 30.0f);
    } else if (pkt->soil_humi_pct >= soil_high) {
        p = 0;
    }

    float lux_low = 5000.0f;
    float lux_mid = 2000.0f;
    float lux_very_low = 500.0f;
    if (profile_rt.ready && profile_rt.matches_current) {
        lux_low = profile_rt.profile.lux_min;
        lux_mid = profile_rt.profile.lux_min * 0.6f;
        lux_very_low = profile_rt.profile.lux_min * 0.3f;
    }
    if (pkt->lux < lux_very_low) {
        l = 80;
    } else if (pkt->lux < lux_mid) {
        l = 50;
    } else if (pkt->lux < lux_low) {
        l = 20;
    }

    *fan = f;
    *pump = p;
    *servo = s;
    *light = l;
    if (reason && reason_size > 0) {
        snprintf(reason, reason_size, "传感器本地兜底控制");
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
    float dry_floor = 15.0f;
    float wet_ceiling = 80.0f;
    plant_profile_runtime_info_t profile_rt = {0};
    get_plant_profile_runtime_info(&profile_rt);
    if (profile_rt.ready && profile_rt.matches_current) {
        dry_floor = profile_rt.profile.soil_humi_min;
        wet_ceiling = profile_rt.profile.soil_humi_max;
    }

    if (pkt->soil_humi_pct <= dry_floor) {
        if (*pump < 60) {
            *pump = 60;
        }
        if (reason && reason_size > 0) {
            snprintf(reason, reason_size, "安全规则：土壤干燥，水泵>=60");
        }
    } else if (pkt->soil_humi_pct >= wet_ceiling) {
        if (*pump != 0) {
            *pump = 0;
        }
        if (reason && reason_size > 0) {
            snprintf(reason, reason_size, "安全规则：土壤过湿，水泵关闭");
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

static void enforce_weather_stale_conservative(bool weather_stale, uint32_t weather_age_s,
                                               int *fan, int *pump, int *servo, int *light,
                                               char *reason, size_t reason_size)
{
    if (!weather_stale || !fan || !pump || !servo || !light) {
        return;
    }

    if (*fan > 55) *fan = 55;
    if (*pump > 45) *pump = 45;
    if (*servo > 130) *servo = 130;
    if (*servo < 50) *servo = 50;
    if (*light > 60) *light = 60;

    if (reason && reason_size > 0) {
        snprintf(reason, reason_size, "天气数据过期(%lu秒)，保守控制", (unsigned long)weather_age_s);
    }
}

/* 检测数据是否异常，返回告警描述(空字符串=正常) */
static void check_alerts(const sensor_packet_t *pkt, char *alert_buf, size_t buf_size)
{
    if (!pkt || !alert_buf || buf_size == 0) {
        return;
    }

    alert_buf[0] = '\0';

    float press_min = 95.0f;
    float press_max = 105.0f;
    float env_temp_min = ALERT_TEMP_LOW;
    float env_temp_max = ALERT_TEMP_HIGH;
    float env_humi_min = ALERT_HUMI_LOW;
    float env_humi_max = ALERT_HUMI_HIGH;
    float soil_temp_min = 12.0f;
    float soil_temp_max = ALERT_SOIL_TEMP_HIGH;
    float soil_humi_min = ALERT_SOIL_HUMI_LOW;
    float soil_humi_max = ALERT_SOIL_HUMI_HIGH;
    float ph_min = ALERT_PH_LOW;
    float ph_max = ALERT_PH_HIGH;
    float n_min = 1.0f;
    float n_max = 8.0f;
    float p_min = 1.0f;
    float p_max = 6.0f;
    float k_min = 1.0f;
    float k_max = 10.0f;
    float lux_min = 1000.0f;
    float lux_max = ALERT_LUX_HIGH;

    plant_profile_runtime_info_t profile_rt = {0};
    get_plant_profile_runtime_info(&profile_rt);
    bool using_ai_profile = false;
    if (profile_rt.ready && profile_rt.matches_current) {
        using_ai_profile = true;
        press_min = profile_rt.profile.press_min;
        press_max = profile_rt.profile.press_max;
        env_temp_min = profile_rt.profile.env_temp_min;
        env_temp_max = profile_rt.profile.env_temp_max;
        env_humi_min = profile_rt.profile.env_humi_min;
        env_humi_max = profile_rt.profile.env_humi_max;
        soil_temp_min = profile_rt.profile.soil_temp_min;
        soil_temp_max = profile_rt.profile.soil_temp_max;
        soil_humi_min = profile_rt.profile.soil_humi_min;
        soil_humi_max = profile_rt.profile.soil_humi_max;
        ph_min = profile_rt.profile.ph_min;
        ph_max = profile_rt.profile.ph_max;
        n_min = profile_rt.profile.n_min;
        n_max = profile_rt.profile.n_max;
        p_min = profile_rt.profile.p_min;
        p_max = profile_rt.profile.p_max;
        k_min = profile_rt.profile.k_min;
        k_max = profile_rt.profile.k_max;
        lux_min = profile_rt.profile.lux_min;
        lux_max = profile_rt.profile.lux_max;
    }

    /* 打印当前使用的阈值和实际值对比 */
    static uint32_t last_threshold_log_s = 0;
    uint32_t now_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (now_s - last_threshold_log_s >= 30) {  /* 每30秒打印一次 */
        last_threshold_log_s = now_s;
        /* 
         * 单位说明：
         * - 气压: 传感器上报 hPa，适宜范围存储为 kPa，需转换 (1 kPa = 10 hPa)
         * - 光照: lux
         * - 温度: ℃
         * - 湿度: %
         * - NPK: mg/kg
         */
        float press_hpa_min = press_min * 10.0f;  /* kPa -> hPa */
        float press_hpa_max = press_max * 10.0f;
        
        ESP_LOGI(TAG, "╔═══════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║  监测对象: %-20s [%s]  ║", g_monitor_plant, using_ai_profile ? "AI联网数据" : "默认阈值");
        ESP_LOGI(TAG, "╠═══════════════════════════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  参数         │ 当前值      │ 适宜范围              ║");
        ESP_LOGI(TAG, "╠═══════════════════════════════════════════════════════════╣");
        ESP_LOGI(TAG, "║  光照         │ %8.1f lux │ %7.0f - %-7.0f lux  ║", pkt->lux, lux_min, lux_max);
        ESP_LOGI(TAG, "║  环境温度     │ %8.1f ℃  │ %7.1f - %-7.1f ℃   ║", pkt->env_temp_c, env_temp_min, env_temp_max);
        ESP_LOGI(TAG, "║  环境湿度     │ %8.1f %%  │ %7.0f - %-7.0f %%   ║", pkt->env_humi_pct, env_humi_min, env_humi_max);
        ESP_LOGI(TAG, "║  气压         │ %8.1f hPa│ %7.0f - %-7.0f hPa ║", pkt->press_kpa, press_hpa_min, press_hpa_max);
        ESP_LOGI(TAG, "║  土壤温度     │ %8.1f ℃  │ %7.1f - %-7.1f ℃   ║", pkt->soil_temp_c, soil_temp_min, soil_temp_max);
        ESP_LOGI(TAG, "║  土壤湿度     │ %8.1f %%  │ %7.0f - %-7.0f %%   ║", pkt->soil_humi_pct, soil_humi_min, soil_humi_max);
        ESP_LOGI(TAG, "║  pH值         │ %8.2f    │ %7.1f - %-7.1f     ║", pkt->ph, ph_min, ph_max);
        ESP_LOGI(TAG, "║  氮(N)        │ %5u mg/kg │ %7.0f - %-5.0f mg/kg║", pkt->n, n_min, n_max);
        ESP_LOGI(TAG, "║  磷(P)        │ %5u mg/kg │ %7.0f - %-5.0f mg/kg║", pkt->p, p_min, p_max);
        ESP_LOGI(TAG, "║  钾(K)        │ %5u mg/kg │ %7.0f - %-5.0f mg/kg║", pkt->k, k_min, k_max);
        ESP_LOGI(TAG, "╚═══════════════════════════════════════════════════════════╝");
    }

    if (pkt->lux < lux_min) {
        append_alert_text(alert_buf, buf_size, "光照%.0flux偏低，建议补光；", pkt->lux);
    } else if (pkt->lux > lux_max) {
        append_alert_text(alert_buf, buf_size, "光照%.0flux过强，建议遮阳降温；", pkt->lux);
    }

    if (pkt->env_temp_c < env_temp_min) {
        append_alert_text(alert_buf, buf_size, "环境温度%.1f℃低于适宜范围；", pkt->env_temp_c);
    } else if (pkt->env_temp_c > env_temp_max) {
        append_alert_text(alert_buf, buf_size, "环境温度%.1f℃高于适宜范围；", pkt->env_temp_c);
    }

    if (pkt->env_humi_pct < env_humi_min) {
        append_alert_text(alert_buf, buf_size, "环境湿度%.0f%%偏低；", pkt->env_humi_pct);
    } else if (pkt->env_humi_pct > env_humi_max) {
        append_alert_text(alert_buf, buf_size, "环境湿度%.0f%%偏高；", pkt->env_humi_pct);
    }

    /* 气压比较：传感器数据为 hPa，阈值为 kPa，需要转换 */
    float press_hpa_min = press_min * 10.0f;  /* kPa -> hPa */
    float press_hpa_max = press_max * 10.0f;
    if (pkt->press_kpa < press_hpa_min) {
        append_alert_text(alert_buf, buf_size, "气压%.1fhPa偏低，请结合天气检查；", pkt->press_kpa);
    } else if (pkt->press_kpa > press_hpa_max) {
        append_alert_text(alert_buf, buf_size, "气压%.1fhPa偏高，请结合天气检查；", pkt->press_kpa);
    }

    if (pkt->soil_temp_c < soil_temp_min) {
        append_alert_text(alert_buf, buf_size, "土壤温度%.1f℃偏低；", pkt->soil_temp_c);
    } else if (pkt->soil_temp_c > soil_temp_max) {
        append_alert_text(alert_buf, buf_size, "土壤温度%.1f℃偏高；", pkt->soil_temp_c);
    }

    if (pkt->soil_humi_pct < soil_humi_min) {
        float ratio = soil_humi_min > 0.1f ? (pkt->soil_humi_pct / soil_humi_min) : 1.0f;
        if (ratio <= 0.5f) {
            append_alert_text(alert_buf, buf_size, "土壤湿度%.0f%%低于适宜下限50%%以上；", pkt->soil_humi_pct);
        } else {
            append_alert_text(alert_buf, buf_size, "土壤湿度%.0f%%偏低；", pkt->soil_humi_pct);
        }
    } else if (pkt->soil_humi_pct > soil_humi_max) {
        append_alert_text(alert_buf, buf_size, "土壤湿度%.0f%%偏高；", pkt->soil_humi_pct);
    }

    if (pkt->ph < ph_min) {
        append_alert_text(alert_buf, buf_size, "土壤pH%.2f偏酸；", pkt->ph);
    } else if (pkt->ph > ph_max) {
        append_alert_text(alert_buf, buf_size, "土壤pH%.2f偏碱；", pkt->ph);
    }

    if ((float)pkt->n < n_min) {
        float ratio = n_min > 0.1f ? ((float)pkt->n / n_min) : 1.0f;
        if (ratio <= 0.5f) {
            append_alert_text(alert_buf, buf_size, "氮含量%u严重不足，建议尽快补施氮肥；", pkt->n);
        } else {
            append_alert_text(alert_buf, buf_size, "氮含量%u偏低，建议补施氮肥；", pkt->n);
        }
    } else if ((float)pkt->n > n_max) {
        append_alert_text(alert_buf, buf_size, "氮含量%u偏高，建议减少氮肥；", pkt->n);
    }

    if ((float)pkt->p < p_min) {
        float ratio = p_min > 0.1f ? ((float)pkt->p / p_min) : 1.0f;
        if (ratio <= 0.5f) {
            append_alert_text(alert_buf, buf_size, "磷含量%u严重不足，建议补施磷肥；", pkt->p);
        } else {
            append_alert_text(alert_buf, buf_size, "磷含量%u偏低，建议补施磷肥；", pkt->p);
        }
    } else if ((float)pkt->p > p_max) {
        append_alert_text(alert_buf, buf_size, "磷含量%u偏高；", pkt->p);
    }

    if ((float)pkt->k < k_min) {
        float ratio = k_min > 0.1f ? ((float)pkt->k / k_min) : 1.0f;
        if (ratio <= 0.5f) {
            append_alert_text(alert_buf, buf_size, "钾含量%u严重不足，建议补施钾肥；", pkt->k);
        } else {
            append_alert_text(alert_buf, buf_size, "钾含量%u偏低，建议补施钾肥；", pkt->k);
        }
    } else if ((float)pkt->k > k_max) {
        append_alert_text(alert_buf, buf_size, "钾含量%u偏高；", pkt->k);
    }
}

static bool sensor_change_requires_fast_control(const sensor_packet_t *prev_pkt,
                                                const sensor_packet_t *cur_pkt,
                                                bool had_prev_data,
                                                bool prev_alert,
                                                bool cur_alert)
{
    if (!cur_pkt) {
        return false;
    }
    if (!had_prev_data || !prev_pkt) {
        return true;
    }
    if (prev_alert != cur_alert) {
        return true;
    }

    float d_env_temp = cur_pkt->env_temp_c - prev_pkt->env_temp_c;
    float d_env_humi = cur_pkt->env_humi_pct - prev_pkt->env_humi_pct;
    float d_soil_humi = cur_pkt->soil_humi_pct - prev_pkt->soil_humi_pct;
    float d_lux = cur_pkt->lux - prev_pkt->lux;

    if (d_env_temp < 0) d_env_temp = -d_env_temp;
    if (d_env_humi < 0) d_env_humi = -d_env_humi;
    if (d_soil_humi < 0) d_soil_humi = -d_soil_humi;
    if (d_lux < 0) d_lux = -d_lux;

    if (d_env_temp >= 0.8f || d_env_humi >= 5.0f || d_soil_humi >= 3.0f || d_lux >= 3500.0f) {
        return true;
    }
    return false;
}

static void emergency_fast_control_from_sensor(const sensor_packet_t *pkt)
{
    if (!pkt || g_control_mode != CONTROL_MODE_SMART) {
        return;
    }

    uint32_t now_s = monotonic_time_s();
    static uint32_t s_last_emergency_cmd_s = 0;
    static uint32_t s_last_emergency_voice_s = 0;
    if ((now_s - s_last_emergency_cmd_s) < 2U) {
        return;
    }

    if (pkt->soil_humi_pct >= ALERT_SOIL_HUMI_HIGH) {
        if (g_control_state.pump_speed != 0) {
            apply_control_and_send_immediate(CONTROL_MODE_SMART,
                                             g_control_state.fan_speed,
                                             0,
                                             g_control_state.servo_angle,
                                             g_control_state.light_level,
                                             "emergency wet soil pump=0");
            s_last_emergency_cmd_s = now_s;
            ESP_LOGW(TAG, "emergency control: soil=%.1f, stop pump now", pkt->soil_humi_pct);
            if (g_tts_ready && (now_s - s_last_emergency_voice_s) >= 20U) {
                g_tts_playing = true;
                esp_err_t tts_ret = baidu_tts_speak(&g_tts, "土壤过湿，已立即关闭水泵。");
                g_tts_playing = false;
                ESP_LOGI(TAG, "emergency wet-soil tts ret=%s", esp_err_to_name(tts_ret));
                if (tts_ret == ESP_OK) {
                    s_last_emergency_voice_s = now_s;
                }
            }
            ESP_LOGW(TAG, "应急控制: 土壤湿度%.1f%%，立即关泵", pkt->soil_humi_pct);
        }
        return;
    }

    if (pkt->soil_humi_pct <= ALERT_SOIL_HUMI_CRITICAL) {
        if (g_control_state.pump_speed < 60) {
            apply_control_and_send_immediate(CONTROL_MODE_SMART,
                                             g_control_state.fan_speed,
                                             60,
                                             g_control_state.servo_angle,
                                             g_control_state.light_level,
                                             "emergency critical dry soil pump>=60");
            s_last_emergency_cmd_s = now_s;
            ESP_LOGW(TAG, "emergency control: soil=%.1f, raise pump to 60 now", pkt->soil_humi_pct);
            if (g_tts_ready && (now_s - s_last_emergency_voice_s) >= 20U) {
                g_tts_playing = true;
                esp_err_t tts_ret = baidu_tts_speak(&g_tts, "土壤极干，已立即启动水泵。");
                g_tts_playing = false;
                ESP_LOGI(TAG, "emergency dry-soil tts ret=%s", esp_err_to_name(tts_ret));
                if (tts_ret == ESP_OK) {
                    s_last_emergency_voice_s = now_s;
                }
            }
            ESP_LOGW(TAG, "应急控制: 土壤湿度%.1f%%，立即提升水泵", pkt->soil_humi_pct);
        }
    }
}

static void plant_profile_refresh_task(void *arg)
{
    (void)arg;

    while (1) {
        if (!g_plant_profile_refresh_requested || !g_ai_clients_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        char plant_name[sizeof(g_monitor_plant)] = {0};
        snprintf(plant_name, sizeof(plant_name), "%s", g_monitor_plant);

        if (!ai_lock_take(pdMS_TO_TICKS(5000))) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        spark_chat_clear_history(&g_profile_lookup);
        spark_chat_add_message(&g_profile_lookup, "system",
            "You are an agronomy assistant. Use web search when needed and reply with JSON only.");

        char prompt[1024];
        snprintf(prompt, sizeof(prompt),
                 "For plant \"%s\", return only JSON with fields "
                 "{\"plant\":\"%s\",\"press_min\":number,\"press_max\":number,"
                 "\"env_temp_min\":number,\"env_temp_max\":number,"
                 "\"env_humi_min\":number,\"env_humi_max\":number,"
                 "\"soil_temp_min\":number,\"soil_temp_max\":number,"
                 "\"soil_humi_min\":number,\"soil_humi_max\":number,"
                 "\"ph_min\":number,\"ph_max\":number,"
                 "\"n_min\":number,\"n_max\":number,"
                 "\"p_min\":number,\"p_max\":number,"
                 "\"k_min\":number,\"k_max\":number,"
                 "\"lux_min\":number,\"lux_max\":number,"
                 "\"summary\":\"short text\"}. "
                 "Use suitable growth targets for monitoring and greenhouse management. No markdown.",
                 plant_name, plant_name);
        spark_chat_add_message(&g_profile_lookup, "user", prompt);

        bool ok = spark_chat_request(&g_profile_lookup);
        spark_chat_close_connection(&g_profile_lookup);

        plant_profile_t profile = {0};
        bool parsed = false;
        const char *resp = spark_chat_get_last_response(&g_profile_lookup);
        if (ok) {
            parsed = parse_plant_profile_json_from_text(resp, &profile, plant_name);
        }
        ai_lock_give();

        if (!parsed) {
            fill_default_plant_profile(&profile, plant_name, "default");
            ESP_LOGW(TAG, "plant profile refresh failed, use default profile for %s", plant_name);
        }

        if (strcmp(plant_name, g_monitor_plant) == 0) {
            plant_profile_store(&profile);
            g_plant_profile_refresh_requested = false;
            g_smart_control_force_run = true;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
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
    ctrl_cfg.stream = false;  /* 控制通道使用非流式，提高稳定性 */
    ctrl_cfg.enable_web_search = false;
    ctrl_cfg.timeout_ms = 20000;  /* 增加超时到20秒 */
    spark_chat_init(&g_smart_ctrl, &ctrl_cfg);

    spark_chat_config_t profile_cfg = spark_cfg;
    profile_cfg.stream = false;
    profile_cfg.enable_web_search = true;
    profile_cfg.timeout_ms = 20000;  /* 增加超时到20秒 */
    spark_chat_init(&g_profile_lookup, &profile_cfg);
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
    g_tts_ready = (ret == ESP_OK);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "百度TTS初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    /* 启动阶段避免立即拉高功放和播放自检音，降低峰值电流防止Brownout。
     * 首次TTS播放时会通过max98357a_write自动使能功放。 */
#if AUDIO_SELF_TEST_ON_BOOT
    ret = max98357a_prepare_output(g_audio_handle, 16000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX98357A 自检前准备失败: %s", esp_err_to_name(ret));
        return;
    }
    audio_self_test_beep();
#endif

    ESP_LOGI(TAG, "AI 语音播报模块初始化完成");
}

/* 构建传感器数据摘要发送给 AI */
#if 0
static void build_sensor_prompt(char *buf, size_t size, bool is_alert, const char *alerts)
{
    weather_runtime_info_t weather_rt = {0};
    get_weather_runtime_info(&weather_rt);

    const char *weather_obs_time = weather_rt.weather.report_time[0] ? weather_rt.weather.report_time : "-";
    uint32_t sample_time_s = g_last_sensor_sample_time_s;
    uint32_t cloud_report_time_s = g_last_cloud_report_time_s;

    char weather_text[192];
    build_weather_prompt_text(weather_text, sizeof(weather_text));
    plant_profile_runtime_info_t profile_rt = {0};
    get_plant_profile_runtime_info(&profile_rt);
    char profile_text[512];
    if (profile_rt.ready && profile_rt.matches_current) {
        snprintf(profile_text, sizeof(profile_text),
                 "目标范围:气压%.1f-%.1fkPa,环境温度%.1f-%.1f℃,环境湿度%.0f-%.0f%%,土壤温度%.1f-%.1f℃,土壤湿度%.0f-%.0f%%,pH%.1f-%.1f,N%.1f-%.1f,P%.1f-%.1f,K%.1f-%.1f,光照%.0f-%.0flux。资料:%s。",
                 profile_rt.profile.press_min,
                 profile_rt.profile.press_max,
                 profile_rt.profile.env_temp_min,
                 profile_rt.profile.env_temp_max,
                 profile_rt.profile.env_humi_min,
                 profile_rt.profile.env_humi_max,
                 profile_rt.profile.soil_temp_min,
                 profile_rt.profile.soil_temp_max,
                 profile_rt.profile.soil_humi_min,
                 profile_rt.profile.soil_humi_max,
                 profile_rt.profile.ph_min,
                 profile_rt.profile.ph_max,
                 profile_rt.profile.n_min,
                 profile_rt.profile.n_max,
                 profile_rt.profile.p_min,
                 profile_rt.profile.p_max,
                 profile_rt.profile.k_min,
                 profile_rt.profile.k_max,
                 profile_rt.profile.lux_min,
                 profile_rt.profile.lux_max,
                 profile_rt.profile.summary);
    } else {
        snprintf(profile_text, sizeof(profile_text), "植物目标范围正在检索中。");
    }

    if (has_data) {
        snprintf(buf, size,
                 "你是%s农业监测助手。"
                 "当前数据:光照%.0flux,环境温度%.1f℃,环境湿度%.0f%%,气压%.1fkPa,土壤温度%.1f℃,土壤湿度%.0f%%,pH%.2f,N=%u,P=%u,K=%u。"
                 "%s天气:%s。告警:%s。"
                 "请先判断哪些指标偏离适宜范围，再给出极简中文建议。"
                 "可自动调节的只有水泵、风扇、补光灯、舵机遮阳；肥料、pH、气压异常只提醒工作人员处理。"
                 "总字数不超过90字。",
                 g_monitor_plant,
                 latest_pkt.lux,
                 latest_pkt.env_temp_c,
                 latest_pkt.env_humi_pct,
                 latest_pkt.press_kpa,
                 latest_pkt.soil_temp_c,
                 latest_pkt.soil_humi_pct,
                 latest_pkt.ph,
                 latest_pkt.n,
                 latest_pkt.p,
                 latest_pkt.k,
                 profile_text,
                 weather_text,
                 (is_alert && alerts[0]) ? alerts : "无");
    } else {
        snprintf(buf, size,
                 "暂无传感器数据。天气：%s。时间戳:sample=%lu,weather_obs=%.31s,cloud=%lu。"
                 "weather_age_s=%lu,weather_stale=%d。提醒检查设备。",
                 weather_text,
                 (unsigned long)sample_time_s,
                 weather_obs_time,
                 (unsigned long)cloud_report_time_s,
                 (unsigned long)weather_rt.weather_age_s,
                 weather_rt.weather_stale ? 1 : 0);
    }
}

#endif

static void build_sensor_prompt(char *buf, size_t size, bool is_alert, const char *alerts)
{
    if (!buf || size == 0) {
        return;
    }

    weather_runtime_info_t weather_rt = {0};
    get_weather_runtime_info(&weather_rt);

    const char *weather_obs_time = weather_rt.weather.report_time[0] ? weather_rt.weather.report_time : "-";
    uint32_t sample_time_s = g_last_sensor_sample_time_s;
    uint32_t cloud_report_time_s = g_last_cloud_report_time_s;

    char weather_text[192];
    char profile_text[512];
    build_weather_prompt_text(weather_text, sizeof(weather_text));
    build_profile_prompt_text(profile_text, sizeof(profile_text), false);

    if (has_data) {
        snprintf(buf, size,
                 "You are an agricultural monitoring assistant for %s. "
                 "Current sensor data: light %.0f lux, ambient temperature %.1f C, ambient humidity %.0f%%, pressure %.1f kPa, soil temperature %.1f C, soil humidity %.0f%%, pH %.2f, N=%u, P=%u, K=%u. "
                 "%s Weather: %s. Alerts: %s. "
                 "First compare all ten metrics with the suitable range, then give a concise Chinese advisory within 90 Chinese characters. "
                 "Only water pump, fan, grow light and shading servo are controllable in smart mode. "
                 "For pressure, pH, N, P and K, do not invent actuator actions; only remind staff what to handle.",
                 g_monitor_plant,
                 latest_pkt.lux,
                 latest_pkt.env_temp_c,
                 latest_pkt.env_humi_pct,
                 latest_pkt.press_kpa,
                 latest_pkt.soil_temp_c,
                 latest_pkt.soil_humi_pct,
                 latest_pkt.ph,
                 latest_pkt.n,
                 latest_pkt.p,
                 latest_pkt.k,
                 profile_text,
                 weather_text,
                 (is_alert && alerts && alerts[0]) ? alerts : "normal");
    } else {
        snprintf(buf, size,
                 "No sensor data has arrived yet. Weather: %s. "
                 "Timestamps: sample=%lu, weather_obs=%.31s, cloud=%lu, weather_age_s=%lu, weather_stale=%d. "
                 "Remind staff to inspect the device link and keep the answer concise in Chinese.",
                 weather_text,
                 (unsigned long)sample_time_s,
                 weather_obs_time,
                 (unsigned long)cloud_report_time_s,
                 (unsigned long)weather_rt.weather_age_s,
                 weather_rt.weather_stale ? 1 : 0);
    }
}

/* AI 分析 + 语音播报任务 */
static void ai_report_task(void *arg)
{
    /* 等待系统基本启动完成，尽快触发首次播报以便确认音频链路 */
    ESP_LOGI(TAG, "ai_report task started, wait 2000 ms before intro");
    vTaskDelay(pdMS_TO_TICKS(2000));

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
        char alert_buf[512];  /* 增大以容纳完整中文异常描述 */
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
            g_alert_control_hold = false;
        }

        normal_elapsed_s = 0;  /* 重置计时 */

        /* 1) 构建传感器数据提示词 */
        if (is_alert && g_control_mode == CONTROL_MODE_SMART) {
            g_smart_control_force_run = true;
        }

        char prompt[1024];
        build_sensor_prompt(prompt, sizeof(prompt), is_alert, alert_buf);

        /* 2) 发送给大模型 */
        if (is_alert && g_control_mode == CONTROL_MODE_SMART) {
            /* Give smart-control task a short head start, then still speak advisory suggestions. */
            vTaskDelay(pdMS_TO_TICKS(1200));
        }

        bool ok = false;
        char response_local[512] = {0};
        if (g_ai_clients_ready && ai_lock_take(pdMS_TO_TICKS(30000))) {
            spark_chat_add_message(&g_spark, "user", prompt);
            spark_chat_trim_history(&g_spark);

            ok = spark_chat_request(&g_spark);
            /* 每次请求后释放HTTP连接，避免复用过期连接导致下次收不到数据 */
            spark_chat_close_connection(&g_spark);

            if (ok) {
                const char *response = spark_chat_get_last_response(&g_spark);
                snprintf(response_local, sizeof(response_local), "%s", response ? response : "");
                spark_chat_add_message(&g_spark, "assistant", response_local);
            }
            ai_lock_give();
        }

        if (ok) {
            ESP_LOGI(TAG, "AI播报: %s", response_local);
            snprintf(g_last_ai_reply, sizeof(g_last_ai_reply), "%.255s", response_local);

            /* 3) TTS 语音播放 */
            if (strlen(response_local) > 0) {
                g_tts_playing = true;
                baidu_tts_speak(&g_tts, response_local);
                g_tts_playing = false;
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
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint32_t last_run_s = 0;
    uint32_t consecutive_ai_fail = 0;
    uint32_t last_alert_voice_s = 0;

    while (1) {
        uint32_t now_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
        bool interval_due = (now_s - last_run_s) >= SMART_CONTROL_INTERVAL_S;
        bool force_run_requested = g_smart_control_force_run;
        char alert_buf[512] = {0};  /* 增大以容纳完整中文异常描述 */

        if (!has_data || g_control_mode != CONTROL_MODE_SMART) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!force_run_requested && !interval_due) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        g_smart_control_force_run = false;
        check_alerts(&latest_pkt, alert_buf, sizeof(alert_buf));
        bool is_alert = (alert_buf[0] != '\0');

        char prompt[2048];
        build_smart_control_prompt(prompt, sizeof(prompt));

        weather_runtime_info_t weather_rt = {0};
        get_weather_runtime_info(&weather_rt);

        bool ok = false;
        const int max_retry = force_run_requested ? 1 : 2;
        char resp_local[512] = {0};
        /* 增加锁等待时间，因为 profile 任务可能持锁长达 15 秒 */
        if (g_ai_clients_ready && ai_lock_take(pdMS_TO_TICKS(18000))) {
            spark_chat_add_message(&g_smart_ctrl, "user", prompt);
            spark_chat_trim_history(&g_smart_ctrl);

            for (int i = 0; i < max_retry; i++) {
                ok = spark_chat_request(&g_smart_ctrl);
                spark_chat_close_connection(&g_smart_ctrl);
                if (ok) {
                    const char *resp = spark_chat_get_last_response(&g_smart_ctrl);
                    snprintf(resp_local, sizeof(resp_local), "%s", resp ? resp : "");
                    break;
                }

                int last_http = spark_chat_get_last_http_status(&g_smart_ctrl);
                bool permanent_err = spark_chat_last_error_is_permanent(&g_smart_ctrl);
                bool overdue_balance = spark_chat_last_error_is_overdue_balance(&g_smart_ctrl);
                if (permanent_err) {
                    if (overdue_balance) {
                        ESP_LOGE(TAG, "智能控制AI不可重试: 账户欠费(HTTP=%d)，停止重试并回退本地策略", last_http);
                    } else {
                        ESP_LOGW(TAG, "智能控制AI不可重试 HTTP=%d，停止重试并回退本地策略", last_http);
                    }
                    break;
                }

                if (i == 0) {
                    /* 首次失败后清空对话上下文，避免历史语境污染导致连续失败 */
                    reset_ai_prompts_locked();
                    spark_chat_add_message(&g_smart_ctrl, "user", prompt);
                    spark_chat_trim_history(&g_smart_ctrl);
                    ESP_LOGW(TAG, "智能控制首次失败，已重置上下文后重试");
                }
                ESP_LOGW(TAG, "智能控制AI请求失败，重试 %d/%d", i + 1, max_retry);
                if (i < max_retry - 1) {
                    vTaskDelay(pdMS_TO_TICKS((i + 1) * 1000));
                }
            }
            ai_lock_give();
        } else if (g_ai_clients_ready) {
            ESP_LOGW(TAG, "智能控制等待AI锁超时，跳过AI并使用本地降级");
        }

        if (!ok) {
            consecutive_ai_fail++;
            ESP_LOGW(TAG, "智能控制AI请求失败，连续失败=%lu", (unsigned long)consecutive_ai_fail);

            // 本轮AI请求失败即启用本地降级控制，避免土壤极干时迟迟不开泵。
            int fan = 0, pump = 0, servo = 90;
            int light = g_control_state.light_level;
            char reason[96] = "传感器本地兜底控制";
            build_fallback_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            enforce_sensor_safety_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            apply_control_and_send(CONTROL_MODE_SMART, fan, pump, servo, light, reason);
            snprintf(g_last_ai_reply, sizeof(g_last_ai_reply), "兜底控制:%s", reason);
            ESP_LOGW(TAG, "AI请求失败，启用本地降级控制 fan=%d pump=%d servo=%d light=%d", fan, pump, servo, light);
            if (is_alert && (now_s - last_alert_voice_s) >= 20U) {
                char voice_buf[256];
                snprintf(voice_buf, sizeof(voice_buf),
                         "检测到异常。已执行兜底控制。风扇%d，水泵%d，舵机%d，补光%d。原因%.80s。",
                         fan, pump, servo, light, reason);
                g_tts_playing = true;
                esp_err_t tts_ret = baidu_tts_speak(&g_tts, voice_buf);
                g_tts_playing = false;
                ESP_LOGI(TAG, "alert fallback tts ret=%s", esp_err_to_name(tts_ret));
                last_alert_voice_s = now_s;
            }

            if (SMART_CONTROL_INTERVAL_S > SMART_CONTROL_FAIL_BACKOFF_S) {
                last_run_s = now_s - (SMART_CONTROL_INTERVAL_S - SMART_CONTROL_FAIL_BACKOFF_S);
            } else {
                last_run_s = now_s;
            }

            // 失败后短暂退避，避免长时间阻塞导致异常播报后的控制响应变慢
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        consecutive_ai_fail = 0;

        int fan = 0, pump = 0, servo = 90;
        int light = g_control_state.light_level;
        char reason[96] = "智能调节";
        snprintf(g_last_ai_reply, sizeof(g_last_ai_reply), "%.255s", resp_local);

        if (parse_control_json_from_text(resp_local, &fan, &pump, &servo, &light, reason, sizeof(reason))) {
            enforce_weather_stale_conservative(weather_rt.weather_stale,
                                              weather_rt.weather_age_s,
                                              &fan,
                                              &pump,
                                              &servo,
                                              &light,
                                              reason,
                                              sizeof(reason));
            enforce_sensor_safety_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            apply_control_and_send(CONTROL_MODE_SMART, fan, pump, servo, light, reason);
            ESP_LOGI(TAG, "智能控制: fan=%d pump=%d servo=%d light=%d reason=%s",
                     fan, pump, servo, light, reason);
            if (is_alert && (now_s - last_alert_voice_s) >= 20U) {
                char voice_buf[256];
                snprintf(voice_buf, sizeof(voice_buf),
                         "检测到异常。智能决策已生效。风扇%d，水泵%d，舵机%d，补光%d。原因%.80s。",
                         fan, pump, servo, light, reason);
                g_tts_playing = true;
                baidu_tts_speak(&g_tts, voice_buf);
                g_tts_playing = false;
                last_alert_voice_s = now_s;
            }
        } else {
            ESP_LOGW(TAG, "智能控制解析失败: %s", resp_local[0] ? resp_local : "(null)");

            build_fallback_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            enforce_sensor_safety_control(&latest_pkt, &fan, &pump, &servo, &light, reason, sizeof(reason));
            apply_control_and_send(CONTROL_MODE_SMART, fan, pump, servo, light, reason);
            ESP_LOGW(TAG, "解析失败，启用本地降级控制 fan=%d pump=%d servo=%d light=%d", fan, pump, servo, light);
            if (is_alert && (now_s - last_alert_voice_s) >= 20U) {
                char voice_buf[256];
                snprintf(voice_buf, sizeof(voice_buf),
                         "检测到异常。智能解析失败，已执行兜底控制。风扇%d，水泵%d，舵机%d，补光%d。",
                         fan, pump, servo, light);
                g_tts_playing = true;
                esp_err_t tts_ret = baidu_tts_speak(&g_tts, voice_buf);
                g_tts_playing = false;
                ESP_LOGI(TAG, "alert parse-fallback tts ret=%s", esp_err_to_name(tts_ret));
                last_alert_voice_s = now_s;
            }
        }

        last_run_s = now_s;
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

    g_weather_lock = xSemaphoreCreateMutex();
    g_plant_profile_lock = xSemaphoreCreateMutex();
    if (!g_plant_profile_lock) {
        ESP_LOGW(TAG, "plant profile lock create failed");
    }

    request_plant_profile_refresh(g_monitor_plant);
    if (!g_weather_lock) {
        ESP_LOGW(TAG, "创建天气互斥锁失败，将使用无锁访问");
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

    /* 启动天气获取任务（高德） */
    xTaskCreate(weather_fetch_task, "weather_fetch", 8192, NULL, 4, NULL);

    ESP_LOGI(TAG, "A39C RX start, MD0=%d MD1=%d AUX=%d",
             gpio_get_level(A39C_MD0_PIN),
             gpio_get_level(A39C_MD1_PIN),
             gpio_get_level(A39C_AUX_PIN));

    /* 初始化 ST7735S 显示屏（须在 LoRa 任务之前，否则 lcd_update 使用未初始化的 SPI） */
    if (lcd_init() == ESP_OK) {
        lcd_update(NULL, false, monitor_target_for_lcd(), false, false, "");
    }

    /* 在独立任务中运行 LoRa 接收循环 */
    xTaskCreate(lora_rx_task, "lora_rx", 8192, NULL, 10, NULL);
    xTaskCreate(control_retry_task, "ctrl_retry", 4096, NULL, 8, NULL);

    /* 初始化 AI 语音播报并启动任务 */
    ai_audio_init();

    /*
     * 使用 heap_caps_malloc 显式从 PSRAM 分配栈内存，TCB 从内部 RAM 分配。
     * 这样可以绕过 SPIRAM_MALLOC_ALWAYSINTERNAL 的限制。
     */
    BaseType_t task_ret = pdFAIL;

    /* plant_profile 任务 */
    {
        const size_t stack_size = 12288;
        StaticTask_t *tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        StackType_t *stack = heap_caps_malloc(stack_size * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (tcb && stack) {
            TaskHandle_t h = xTaskCreateStatic(plant_profile_refresh_task, "plant_profile", stack_size, NULL, 3, stack, tcb);
            task_ret = (h != NULL) ? pdPASS : pdFAIL;
        } else {
            task_ret = pdFAIL;
            ESP_LOGE(TAG, "plant_profile alloc failed: tcb=%p stack=%p", tcb, stack);
        }
    }

    /* ai_report 任务 */
    {
        const size_t stack_size = 16384;
        StaticTask_t *tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        StackType_t *stack = heap_caps_malloc(stack_size * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (tcb && stack) {
            TaskHandle_t h = xTaskCreateStatic(ai_report_task, "ai_report", stack_size, NULL, 5, stack, tcb);
            task_ret = (h != NULL) ? pdPASS : pdFAIL;
        } else {
            task_ret = pdFAIL;
            ESP_LOGE(TAG, "ai_report alloc failed: tcb=%p stack=%p", tcb, stack);
        }
    }

    /* ai_smart_control 任务 */
    {
        const size_t stack_size = 12288;
        StaticTask_t *tcb = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        StackType_t *stack = heap_caps_malloc(stack_size * sizeof(StackType_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (tcb && stack) {
            TaskHandle_t h = xTaskCreateStatic(ai_smart_control_task, "ai_smart_ctl", stack_size, NULL, 3, stack, tcb);
            task_ret = (h != NULL) ? pdPASS : pdFAIL;
        } else {
            task_ret = pdFAIL;
            ESP_LOGE(TAG, "ai_smart_ctl alloc failed: tcb=%p stack=%p", tcb, stack);
        }
    }

    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  智慧农业监测系统已启动");
    ESP_LOGI(TAG, "============================================");
}
