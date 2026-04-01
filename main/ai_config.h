#pragma once

#include "driver/gpio.h"

/* ====== 豆包(Ark) 大模型配置 ====== */
#define AI_API_KEY          "Bearer 247f57b8-a974-4319-82df-5231e8d958e0"
#define AI_URL              "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
#define AI_MODEL            "ep-m-20251108165819-4j5wk"  /* Doubao-Seed-1.6-lite 接入点 */

/* ====== 百度 TTS 配置 ====== */
#define BAIDU_API_KEY       "QIZavSTgRHtRiTZZMmPnL56r"
#define BAIDU_SECRET_KEY    "Rg9dG5eHNMpjdRq7nw0KUy3CPsALiIol"

/* ====== MAX98357A 音频输出引脚（ESP32-S3 N16R8 安全引脚） ====== */
/* 注意: GPIO26-37 被 Octal Flash/PSRAM 占用，不可使用 */
#define AUDIO_BCLK_PIN      GPIO_NUM_41
#define AUDIO_LRCLK_PIN     GPIO_NUM_40
#define AUDIO_DIN_PIN       GPIO_NUM_42
#define AUDIO_SD_MODE_PIN   GPIO_NUM_21    /* MAX98357 SD/EN 建议避开USB脚(GPIO19/20) */

/* ====== AI 语音播报间隔（秒） ====== */
#define AI_REPORT_INTERVAL_S  60    /* 每60秒让AI分析一次传感器数据并语音播报 */

/* ====== 种植品种配置 ======
 * 修改此项即可切换监测对象，AI会联网搜索该品种的种植资料自动分析
 * 示例: "桉树", "水稻", "茶树", "柑橘", "芒果", "甘蔗", "橡胶树"
 */
#define PLANT_SPECIES     "桉树"
#define PLANT_SPECIES_EN  "Eucalyptus"   /* 屏幕显示用英文名，≤13字符 */

/* ====== ST7735S 128×160 显示屏引脚配置 ======
 * 接线: VCC→3.3V  GND→GND
 *       CS→GPIO10  RST→GPIO9  DC→GPIO13
 *       SDA/MOSI→GPIO11  SCK→GPIO12  LED→GPIO8
 */
#define LCD_SPI_HOST   SPI2_HOST
#define LCD_MOSI_PIN   GPIO_NUM_11
#define LCD_SCLK_PIN   GPIO_NUM_12
#define LCD_CS_PIN     GPIO_NUM_10
#define LCD_DC_PIN     GPIO_NUM_13
#define LCD_RST_PIN    GPIO_NUM_9
#define LCD_BL_PIN     GPIO_NUM_8
/* 部分模块存在地址偏移（如2,3），若显示位置偏移则修改以下两项 */
#define LCD_X_OFFSET   0
#define LCD_Y_OFFSET   0
