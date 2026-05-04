#pragma once

#include "driver/gpio.h"

/* ====== 豆包(Doubao) 大语言模型配置 ====== */
// AI引擎的鉴权凭证（Bearer Token形式）
#define AI_API_KEY          "Bearer 247f57b8-a974-4319-82df-5231e8d958e0"
// 火山引擎（Ark）对话API地址
#define AI_URL              "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
// 所使用的具体模型型号
#define AI_MODEL            "ep-m-20251108165819-4j5wk"

/* ====== 高德地图天气 API 配置 (默认城市：南宁) ====== */
// 高德开放平台 Web 服务 API Key
#define AMAP_WEATHER_API_KEY            "1c14cbf8116ed22e6128feb03e2361b2"
// 南宁市的行政区划代码 (Adcode)
#define AMAP_WEATHER_CITY_ADCODE        "450100"
// 天气数据定时刷新间隔 (单位: 秒, 默认 15 分钟)
#define AMAP_WEATHER_REFRESH_INTERVAL_S 900
// 天气数据过期阈值 (单位: 秒, 默认 30 分钟)
#define AMAP_WEATHER_STALE_THRESHOLD_S  1800

/* ====== 百度语音合成 (TTS) 鉴权配置 ====== */
#define BAIDU_API_KEY       "QIZavSTgRHtRiTZZMmPnL56r"
#define BAIDU_SECRET_KEY    "Rg9dG5eHNMpjdRq7nw0KUy3CPsALiIol"

/* ====== MAX98357A 音频功放 I2S 引脚配置 ====== */
// BCLK (串行时钟) 引脚
#define AUDIO_BCLK_PIN      GPIO_NUM_41
// LRCLK (左右声道时钟/帧同步) 引脚
#define AUDIO_LRCLK_PIN     GPIO_NUM_40
// DIN (数据输入) 引脚
#define AUDIO_DIN_PIN       GPIO_NUM_42
// SD_MODE (关断/声道选定) 引脚
#define AUDIO_SD_MODE_PIN   GPIO_NUM_21

/* ====== AI 语音播报间隔 ====== */
// 设定每隔多长时间执行一次大模型推理及语音播报 (单位: 秒)
#define AI_REPORT_INTERVAL_S  60

/* ====== 默认监测植物名称 ======
 * 网页端与AI业务逻辑使用中文名称 (PLANT_SPECIES)
 * 当LCD无法显示中文时，退而显示英文名称 (PLANT_SPECIES_EN)
 */
#define PLANT_SPECIES     "桉树苗"
#define PLANT_SPECIES_EN  "EucaSeedling"

/* ====== ST7735S (128x160) TFT液晶屏引脚配置 ====== */
#define LCD_SPI_HOST   SPI2_HOST
// SPI 数据引脚 (MOSI)
#define LCD_MOSI_PIN   GPIO_NUM_11
// SPI 时钟引脚 (SCLK)
#define LCD_SCLK_PIN   GPIO_NUM_12
// 片选引脚 (CS)
#define LCD_CS_PIN     GPIO_NUM_10
// 数据/命令选择引脚 (DC)
#define LCD_DC_PIN     GPIO_NUM_13
// 硬件复位引脚 (RST)
#define LCD_RST_PIN    GPIO_NUM_9
// 背光控制引脚 (BLK)
#define LCD_BL_PIN     GPIO_NUM_8
// 屏幕显示内容的坐标偏移量
#define LCD_X_OFFSET   0
#define LCD_Y_OFFSET   0
