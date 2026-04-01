/**
 * @file st7735_display.c
 * @brief ST7735S 128×160 SPI 驱动 + 智慧农业监测UI实现
 */

#include "st7735_display.h"
#include "ai_config.h"
#include "char_bitmap.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "LCD";

/* ── SPI 句柄 ── */
static spi_device_handle_t s_spi = NULL;

/* ── ST7735S 命令 ── */
#define ST77_SWRESET  0x01
#define ST77_SLPOUT   0x11
#define ST77_NORON    0x13
#define ST77_INVOFF   0x20
#define ST77_DISPON   0x29
#define ST77_CASET    0x2A
#define ST77_RASET    0x2B
#define ST77_RAMWR    0x2C
#define ST77_MADCTL   0x36
#define ST77_COLMOD   0x3A
#define ST77_FRMCTR1  0xB1
#define ST77_FRMCTR2  0xB2
#define ST77_FRMCTR3  0xB3
#define ST77_INVCTR   0xB4
#define ST77_PWCTR1   0xC0
#define ST77_PWCTR2   0xC1
#define ST77_PWCTR3   0xC2
#define ST77_PWCTR4   0xC3
#define ST77_PWCTR5   0xC4
#define ST77_VMCTR1   0xC5
#define ST77_GMCTRP1  0xE0
#define ST77_GMCTRN1  0xE1

/* ── 5×7 点阵字体（可打印 ASCII 0x20~0x7E，每字符 5 列）── */
static const uint8_t s_font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */ {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */ {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */ {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */ {0x00,0x05,0x03,0x00,0x00}, /* '\'' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */ {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */ {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */ {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */ {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */ {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */ {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */ {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */ {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */ {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */ {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */ {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */ {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */ {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */ {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */ {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */ {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */ {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */ {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x04,0x02,0x7F}, /* 'M' */ {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */ {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */ {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */ {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */ {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */ {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */ {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */ {0x02,0x04,0x08,0x10,0x20}, /* '\\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */ {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */ {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */ {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */ {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */ {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */ {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */ {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */ {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */ {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */ {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */ {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */ {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */ {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */ {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */ {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */ {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */ {0x10,0x08,0x08,0x10,0x08}, /* '~' */
};

/* ────────────────────────────────────────────────────
 *  底层 SPI / GPIO 操作
 * ──────────────────────────────────────────────────── */

static inline void set_dc(int level) { gpio_set_level(LCD_DC_PIN, level); }

static void spi_send(const uint8_t *data, size_t len, bool is_cmd)
{
    set_dc(is_cmd ? 0 : 1);
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static inline void lcd_cmd(uint8_t cmd)              { spi_send(&cmd, 1, true); }
static inline void lcd_dat1(uint8_t d)               { spi_send(&d,   1, false); }

static void lcd_cmd_data(uint8_t cmd, const uint8_t *d, size_t n)
{
    lcd_cmd(cmd);
    spi_send(d, n, false);
}

/* ────────────────────────────────────────────────────
 *  地址窗口 + 矩形填充
 * ──────────────────────────────────────────────────── */

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    x0 += LCD_X_OFFSET;  x1 += LCD_X_OFFSET;
    y0 += LCD_Y_OFFSET;  y1 += LCD_Y_OFFSET;

    uint8_t ca[] = { x0>>8, x0&0xFF, x1>>8, x1&0xFF };
    uint8_t ra[] = { y0>>8, y0&0xFF, y1>>8, y1&0xFF };
    lcd_cmd_data(ST77_CASET, ca, 4);
    lcd_cmd_data(ST77_RASET, ra, 4);
    lcd_cmd(ST77_RAMWR);
}

/* 行缓冲区（最宽 128 像素 × 2 字节 = 256 字节，DMA 可访问） */
static uint8_t WORD_ALIGNED_ATTR s_line_buf[LCD_WIDTH * 2];

void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0 || h == 0) return;
    if (x >= LCD_WIDTH  || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH  - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    set_window(x, y, x+w-1, y+h-1);
    set_dc(1);

    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (uint16_t i = 0; i < w; i++) { s_line_buf[i*2] = hi; s_line_buf[i*2+1] = lo; }

    spi_transaction_t t = { .length = w * 16, .tx_buffer = s_line_buf };
    for (uint16_t row = 0; row < h; row++) {
        spi_device_polling_transmit(s_spi, &t);
    }
}

static void lcd_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    lcd_fill_rect(x, y, w, 1, color);
}

/* ────────────────────────────────────────────────────
 *  字符 / 字符串绘制
 * ──────────────────────────────────────────────────── */

/* 字符像素缓冲（最大 scale=2：12×16 = 192 像素 × 2 = 384 字节） */
static uint8_t WORD_ALIGNED_ATTR s_char_buf[12 * 16 * 2];

/**
 * @brief 绘制单个 ASCII 字符
 * @param scale  1 → 6×8px，2 → 12×16px
 */
static void draw_char(uint16_t x, uint16_t y, char c,
                      uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *bm = s_font5x7[(uint8_t)c - 0x20];
    uint8_t cw = 6 * scale;   /* 含 1px 右间距 */
    uint8_t ch = 8 * scale;   /* 含 1px 下间距 */

    uint8_t fg_hi = fg>>8, fg_lo = fg&0xFF;
    uint8_t bg_hi = bg>>8, bg_lo = bg&0xFF;
    uint32_t idx = 0;

    for (uint8_t row = 0; row < 8; row++) {           /* 7行数据 + 1行间距 */
        for (uint8_t sr = 0; sr < scale; sr++) {
            for (uint8_t col = 0; col < 6; col++) {   /* 5列数据 + 1列间距 */
                uint8_t on = (row < 7 && col < 5) ? ((bm[col] >> row) & 1) : 0;
                for (uint8_t sc = 0; sc < scale; sc++) {
                    s_char_buf[idx++] = on ? fg_hi : bg_hi;
                    s_char_buf[idx++] = on ? fg_lo : bg_lo;
                }
            }
        }
    }
    set_window(x, y, x+cw-1, y+ch-1);
    set_dc(1);
    spi_transaction_t t = { .length = idx*8, .tx_buffer = s_char_buf };
    spi_device_polling_transmit(s_spi, &t);
}

static uint16_t draw_string(uint16_t x, uint16_t y, const char *str,
                            uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*str) {
        draw_char(x, y, *str, fg, bg, scale);
        x += 6 * scale;
        str++;
    }
    return x;
}

/** 在 [x, x+width) 范围内居中绘制字符串 */
static void draw_string_centered(uint16_t x, uint16_t y, uint16_t width,
                                 const char *str, uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint16_t sw = (uint16_t)(strlen(str) * 6 * scale);
    uint16_t cx = x + (width > sw ? (width - sw) / 2 : 0);
    draw_string(cx, y, str, fg, bg, scale);
}

/* ────────────────────────────────────────────────────
 *  16×16 中文字符绘制
 * ──────────────────────────────────────────────────── */

static uint8_t WORD_ALIGNED_ATTR s_cjk_buf[16 * 16 * 2];

/** 绘制单个 16×16 中文字符 */
static void draw_cjk_char(uint16_t x, uint16_t y, const uint16_t *bitmap,
                          uint16_t fg, uint16_t bg)
{
    uint8_t fg_hi = fg >> 8, fg_lo = fg & 0xFF;
    uint8_t bg_hi = bg >> 8, bg_lo = bg & 0xFF;
    uint32_t idx = 0;

    for (int row = 0; row < 16; row++) {
        uint16_t bits = bitmap[row];
        for (int col = 0; col < 16; col++) {
            uint8_t on = (bits >> (15 - col)) & 1;
            s_cjk_buf[idx++] = on ? fg_hi : bg_hi;
            s_cjk_buf[idx++] = on ? fg_lo : bg_lo;
        }
    }
    set_window(x, y, x + 15, y + 15);
    set_dc(1);
    spi_transaction_t t = { .length = idx * 8, .tx_buffer = s_cjk_buf };
    spi_device_polling_transmit(s_spi, &t);
}

#define CJK_STEP  17   /* 16px字宽 + 1px间距 */

/**
 * @brief 绘制一组中文字符（通过 Unicode 码点数组）
 * @return 绘制结束后的 x 坐标
 */
static uint16_t draw_cjk_label(uint16_t x, uint16_t y, const uint32_t *cps, int count,
                               uint16_t fg, uint16_t bg)
{
    for (int i = 0; i < count; i++) {
        const uint16_t *bmp = get_char_bitmap(cps[i]);
        if (bmp) {
            draw_cjk_char(x, y, bmp, fg, bg);
            x += CJK_STEP;
        }
    }
    return x;
}

/** 居中绘制一组中文字符 */
static void draw_cjk_centered(uint16_t x, uint16_t y, uint16_t width,
                               const uint32_t *cps, int count,
                               uint16_t fg, uint16_t bg)
{
    uint16_t sw = (uint16_t)(count * CJK_STEP - 1);
    uint16_t cx = x + (width > sw ? (width - sw) / 2 : 0);
    draw_cjk_label(cx, y, cps, count, fg, bg);
}

/* ────────────────────────────────────────────────────
 *  ST7735S 初始化序列
 * ──────────────────────────────────────────────────── */

static void st7735_init_seq(void)
{
    lcd_cmd(ST77_SWRESET); vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(ST77_SLPOUT);  vTaskDelay(pdMS_TO_TICKS(500));

    lcd_cmd_data(ST77_FRMCTR1, (uint8_t[]){0x01,0x2C,0x2D}, 3);
    lcd_cmd_data(ST77_FRMCTR2, (uint8_t[]){0x01,0x2C,0x2D}, 3);
    lcd_cmd_data(ST77_FRMCTR3, (uint8_t[]){0x01,0x2C,0x2D,0x01,0x2C,0x2D}, 6);
    lcd_cmd_data(ST77_INVCTR,  (uint8_t[]){0x07}, 1);
    lcd_cmd_data(ST77_PWCTR1,  (uint8_t[]){0xA2,0x02,0x84}, 3);
    lcd_cmd_data(ST77_PWCTR2,  (uint8_t[]){0xC5}, 1);
    lcd_cmd_data(ST77_PWCTR3,  (uint8_t[]){0x0A,0x00}, 2);
    lcd_cmd_data(ST77_PWCTR4,  (uint8_t[]){0x8A,0x2A}, 2);
    lcd_cmd_data(ST77_PWCTR5,  (uint8_t[]){0x8A,0xEE}, 2);
    lcd_cmd_data(ST77_VMCTR1,  (uint8_t[]){0x0E}, 1);
    lcd_cmd(ST77_INVOFF);
    lcd_cmd_data(ST77_MADCTL,  (uint8_t[]){0x08}, 1);  /* BGR, portrait */
    lcd_cmd_data(ST77_COLMOD,  (uint8_t[]){0x05}, 1);  /* 16bit RGB565 */
    lcd_cmd_data(ST77_GMCTRP1, (uint8_t[]){
        0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
        0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10}, 16);
    lcd_cmd_data(ST77_GMCTRN1, (uint8_t[]){
        0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
        0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10}, 16);
    lcd_cmd(ST77_NORON);  vTaskDelay(pdMS_TO_TICKS(10));
    lcd_cmd(ST77_DISPON); vTaskDelay(pdMS_TO_TICKS(100));
}

/* ────────────────────────────────────────────────────
 *  公开 API：初始化
 * ──────────────────────────────────────────────────── */

esp_err_t lcd_init(void)
{
    /* RST + BL GPIO */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL<<LCD_RST_PIN) | (1ULL<<LCD_BL_PIN) | (1ULL<<LCD_DC_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), TAG, "GPIO config");

    gpio_set_level(LCD_BL_PIN,  0);  /* 初始化期间背光关 */
    gpio_set_level(LCD_DC_PIN,  1);

    /* SPI 总线 */
    spi_bus_config_t bus = {
        .mosi_io_num     = LCD_MOSI_PIN,
        .miso_io_num     = -1,
        .sclk_io_num     = LCD_SCLK_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_WIDTH * 2 * 2 + 8,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO),
                        TAG, "SPI bus init");

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = LCD_CS_PIN,
        .queue_size     = 4,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(LCD_SPI_HOST, &dev, &s_spi),
                        TAG, "SPI add device");

    /* 硬件复位 */
    gpio_set_level(LCD_RST_PIN, 1); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LCD_RST_PIN, 0); vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(LCD_RST_PIN, 1); vTaskDelay(pdMS_TO_TICKS(120));

    st7735_init_seq();

    /* 填充深色背景 */
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_BG);

    /* 背光开 */
    gpio_set_level(LCD_BL_PIN, 1);

    ESP_LOGI(TAG, "ST7735S OK, %dx%d", LCD_WIDTH, LCD_HEIGHT);
    return ESP_OK;
}

void lcd_set_backlight(uint8_t brightness)
{
    gpio_set_level(LCD_BL_PIN, brightness > 0 ? 1 : 0);
}

/* ────────────────────────────────────────────────────
 *  UI 绘制辅助：双列数据卡
 *
 *  每张卡（H=30px，宽 64px）：
 *    y+ 0  中文标签（CYAN，16px）
 *    y+16  数值（VALUE，scale=1 8px，大号数字太占空间改用 scale=1）
 * ──────────────────────────────────────────────────── */

#define COL0_X   0
#define COL1_X   64
#define COL_W    64
#define CARD_H   30

static void draw_card(uint16_t x, uint16_t y,
                      const uint32_t *label_cps, int label_cnt,
                      const char *value, uint16_t val_color)
{
    lcd_fill_rect(x, y, COL_W, CARD_H, COLOR_CARD_BG);
    draw_cjk_label(x+2, y+1, label_cps, label_cnt, COLOR_CYAN, COLOR_CARD_BG);
    draw_string(x+2, y+19, value, val_color, COLOR_CARD_BG, 1);
}

/** 单行紧凑卡（H=20px）：中文标签 + 数值同一行 */
static void draw_card_inline(uint16_t x, uint16_t y, uint16_t w,
                             const uint32_t *label_cps, int label_cnt,
                             const char *value, uint16_t val_color)
{
    lcd_fill_rect(x, y, w, 20, COLOR_CARD_BG);
    uint16_t nx = draw_cjk_label(x+2, y+2, label_cps, label_cnt, COLOR_CYAN, COLOR_CARD_BG);
    draw_string(nx+2, y+6, value, val_color, COLOR_CARD_BG, 1);
}

/* ────────────────────────────────────────────────────
 *  公开 API：刷新全屏
 * ──────────────────────────────────────────────────── */

void lcd_update(const display_sensor_data_t *data,
                bool        has_data,
                const char *species_en,
                bool        wifi_ok,
                bool        mqtt_ok,
                const char *alert_str)
{
    char buf[24];

    /* ── Unicode 码点定义（标签用） ── */
    static const uint32_t CP_QIWEN[] = {0x6C14,0x6E29};               /* 气温 */
    static const uint32_t CP_SHIDU[] = {0x6E7F,0x5EA6};               /* 湿度 */
    static const uint32_t CP_TUWEN[] = {0x571F,0x6E29};               /* 土温 */
    static const uint32_t CP_TUSHI[] = {0x571F,0x6E7F};               /* 土湿 */
    static const uint32_t CP_GUANGZHAO[] = {0x5149,0x7167};           /* 光照 */
    static const uint32_t CP_QIYA[]  = {0x6C14,0x538B};               /* 气压 */
    static const uint32_t CP_DENGDAI[] = {0x7B49,0x5F85,0x4E2D};     /* 等待中 */
    static const uint32_t CP_WUSHUJU[] = {0x65E0,0x6570,0x636E};     /* 无数据 */

    /* ── 品种栏 Y=0..13 (14px) ── */
    lcd_fill_rect(0, 0, LCD_WIDTH, 14, COLOR_SPECIES_BG);
    {
        char spc[18];
        snprintf(spc, sizeof(spc), "> %s <", species_en ? species_en : "Unknown");
        draw_string_centered(0, 3, LCD_WIDTH, spc, COLOR_YELLOW, COLOR_SPECIES_BG, 1);
    }

    /* ── 分割线 Y=14 ── */
    lcd_hline(0, 14, LCD_WIDTH, COLOR_LINE);

    if (!has_data || data == NULL) {
        lcd_fill_rect(0, 15, LCD_WIDTH, 145, COLOR_BG);
        draw_cjk_centered(0, 65, LCD_WIDTH, CP_DENGDAI, 3, COLOR_GRAY, COLOR_BG);
        draw_cjk_centered(0, 85, LCD_WIDTH, CP_WUSHUJU, 3, COLOR_GRAY, COLOR_BG);
        return;
    }

    /* ── 气温 / 湿度 Y=15..46 (32px) ── */
    snprintf(buf, sizeof(buf), "%.1fC", data->env_temp_c);
    draw_card(COL0_X, 15, CP_QIWEN, 2, buf,
              data->env_temp_c > 38.0f ? COLOR_ALERT : COLOR_VALUE);

    snprintf(buf, sizeof(buf), "%.0f%%", data->env_humi_pct);
    draw_card(COL1_X, 15, CP_SHIDU, 2, buf,
              data->env_humi_pct > 93.0f || data->env_humi_pct < 22.0f ? COLOR_ALERT : COLOR_VALUE);

    lcd_hline(0, 45, LCD_WIDTH, COLOR_LINE);

    /* ── 土温 / 土湿 Y=46..77 (32px) ── */
    snprintf(buf, sizeof(buf), "%.1fC", data->soil_temp_c);
    draw_card(COL0_X, 46, CP_TUWEN, 2, buf,
              data->soil_temp_c > 38.0f ? COLOR_ALERT : COLOR_VALUE);

    snprintf(buf, sizeof(buf), "%.0f%%", data->soil_humi_pct);
    draw_card(COL1_X, 46, CP_TUSHI, 2, buf,
              data->soil_humi_pct > 88.0f || data->soil_humi_pct < 17.0f ? COLOR_ALERT : COLOR_VALUE);

    lcd_hline(0, 76, LCD_WIDTH, COLOR_LINE);

    /* ── 光照 Y=77..98 (22px) ── */
    if (data->lux >= 1000.0f) {
        snprintf(buf, sizeof(buf), "%.1fklx", data->lux / 1000.0f);
    } else {
        snprintf(buf, sizeof(buf), "%.1flx", data->lux);
    }
    lcd_fill_rect(0, 77, LCD_WIDTH, 22, COLOR_CARD_BG);
    {
        uint16_t nx = draw_cjk_label(2, 80, CP_GUANGZHAO, 2, COLOR_CYAN, COLOR_CARD_BG);
        draw_string(nx+2, 84, buf,
                    data->lux > 100000.0f ? COLOR_ALERT : COLOR_VALUE,
                    COLOR_CARD_BG, 1);
    }

    lcd_hline(0, 99, LCD_WIDTH, COLOR_LINE);

    /* ── 气压 Y=100..119 (20px full-width) ── */
    lcd_fill_rect(0, 100, LCD_WIDTH, 20, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%.1f kPa", data->press_kpa / 1000.0f);  /* hPa 转 kPa */
    {
        uint16_t nx = draw_cjk_label(2, 102, CP_QIYA, 2, COLOR_CYAN, COLOR_CARD_BG);
        draw_string(nx+2, 106, buf, COLOR_VALUE, COLOR_CARD_BG, 1);
    }

    lcd_hline(0, 120, LCD_WIDTH, COLOR_LINE);

    /* ── PH Y=121..135 (15px full-width) ── */
    lcd_fill_rect(0, 121, LCD_WIDTH, 15, COLOR_CARD_BG);
    snprintf(buf, sizeof(buf), "%.2f", data->ph);
    {
        uint16_t nx = draw_string(2, 125, "PH", COLOR_CYAN, COLOR_CARD_BG, 1);
        draw_string(nx+2, 125, buf,
                    data->ph < 3.5f || data->ph > 9.0f ? COLOR_ALERT : COLOR_VALUE,
                    COLOR_CARD_BG, 1);
    }

    lcd_hline(0, 136, LCD_WIDTH, COLOR_LINE);

    /* ── NPK Y=137..149 (13px) ── */
    lcd_fill_rect(0, 137, LCD_WIDTH, 13, COLOR_CARD_BG);
    {
        uint16_t x = 2;
        draw_string(x, 140, "N:", COLOR_NPK_N, COLOR_CARD_BG, 1);  x += 12;
        snprintf(buf, sizeof(buf), "%u", data->n);
        x = draw_string(x, 140, buf, COLOR_WHITE, COLOR_CARD_BG, 1);
        x += 4;
        draw_string(x, 140, "P:", COLOR_NPK_P, COLOR_CARD_BG, 1);  x += 12;
        snprintf(buf, sizeof(buf), "%u", data->p);
        x = draw_string(x, 140, buf, COLOR_WHITE, COLOR_CARD_BG, 1);
        x += 4;
        draw_string(x, 140, "K:", COLOR_NPK_K, COLOR_CARD_BG, 1);  x += 12;
        snprintf(buf, sizeof(buf), "%u", data->k);
        draw_string(x, 140, buf, COLOR_WHITE, COLOR_CARD_BG, 1);
        draw_string(97, 140, "mg/kg", COLOR_WHITE, COLOR_CARD_BG, 1);
    }

    lcd_hline(0, 150, LCD_WIDTH, COLOR_LINE);

    /* ── 底部状态条 Y=151..159 WiFi/MQTT (9px) ── */
    {
        uint16_t bg = COLOR_RGB(10, 30, 20);
        lcd_fill_rect(0, 151, LCD_WIDTH, 9, bg);
        uint16_t x = 4;
        draw_string(x, 152, "WiFi:", COLOR_YELLOW, bg, 1); x += 30;
        draw_string(x, 152, wifi_ok ? "Y" : "N",
                    wifi_ok ? COLOR_OK : COLOR_ALERT, bg, 1); x += 14;
        draw_string(x, 152, "MQTT:", COLOR_YELLOW, bg, 1); x += 30;
        draw_string(x, 152, mqtt_ok ? "Y" : "N",
                    mqtt_ok ? COLOR_OK : COLOR_ALERT, bg, 1);
    }
}
