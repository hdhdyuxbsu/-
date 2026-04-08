#pragma once

#include "driver/gpio.h"

/* ====== Doubao Ark model config ====== */
#define AI_API_KEY          "Bearer 247f57b8-a974-4319-82df-5231e8d958e0"
#define AI_URL              "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
#define AI_MODEL            "ep-m-20251108165819-4j5wk"

/* ====== AMap weather config (Nanning) ====== */
#define AMAP_WEATHER_API_KEY            "1c14cbf8116ed22e6128feb03e2361b2"
#define AMAP_WEATHER_CITY_ADCODE        "450100"
#define AMAP_WEATHER_REFRESH_INTERVAL_S 900
#define AMAP_WEATHER_STALE_THRESHOLD_S  1800

/* ====== Baidu TTS config ====== */
#define BAIDU_API_KEY       "QIZavSTgRHtRiTZZMmPnL56r"
#define BAIDU_SECRET_KEY    "Rg9dG5eHNMpjdRq7nw0KUy3CPsALiIol"

/* ====== MAX98357A audio pins ====== */
#define AUDIO_BCLK_PIN      GPIO_NUM_41
#define AUDIO_LRCLK_PIN     GPIO_NUM_40
#define AUDIO_DIN_PIN       GPIO_NUM_42
#define AUDIO_SD_MODE_PIN   GPIO_NUM_21

/* ====== AI voice report interval ====== */
#define AI_REPORT_INTERVAL_S  60

/* ====== Default monitor target ======
 * Web and AI logic use the Chinese target name.
 * LCD falls back to the ASCII label below when needed.
 */
#define PLANT_SPECIES     "桉树苗"
#define PLANT_SPECIES_EN  "EucaSeedling"

/* ====== ST7735S 128x160 display pins ====== */
#define LCD_SPI_HOST   SPI2_HOST
#define LCD_MOSI_PIN   GPIO_NUM_11
#define LCD_SCLK_PIN   GPIO_NUM_12
#define LCD_CS_PIN     GPIO_NUM_10
#define LCD_DC_PIN     GPIO_NUM_13
#define LCD_RST_PIN    GPIO_NUM_9
#define LCD_BL_PIN     GPIO_NUM_8
#define LCD_X_OFFSET   0
#define LCD_Y_OFFSET   0
