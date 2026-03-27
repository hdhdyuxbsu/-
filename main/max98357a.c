/**
 * @file max98357a.c
 * @brief MAX98357A I2S数字音频功率放大器驱动实现
 */

#include "max98357a.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "MAX98357A";

/* 默认配置参数 */
#define DEFAULT_SAMPLE_RATE     48000
#define DEFAULT_BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT
#define DEFAULT_I2S_PORT        I2S_NUM_0

/* SD_MODE引脚电平定义 */
#define SD_MODE_SHUTDOWN        0
#define SD_MODE_ENABLE          1

/**
 * @brief 获取默认配置
 */
max98357a_config_t max98357a_get_default_config(void)
{
    max98357a_config_t config = {
        .i2s_port = DEFAULT_I2S_PORT,
        .bclk_pin = GPIO_NUM_NC,
        .lrclk_pin = GPIO_NUM_NC,
        .din_pin = GPIO_NUM_NC,
        .sd_mode_pin = GPIO_NUM_NC,
        .sample_rate = DEFAULT_SAMPLE_RATE,
        .bits_per_sample = DEFAULT_BITS_PER_SAMPLE,
        .gain = MAX98357A_GAIN_12DB,
        .channel = MAX98357A_CHANNEL_LEFT,
    };
    return config;
}

/**
 * @brief 初始化MAX98357A
 */
esp_err_t max98357a_init(const max98357a_config_t *config, max98357a_handle_t **handle)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(config->bclk_pin != GPIO_NUM_NC, ESP_ERR_INVALID_ARG, TAG, "invalid bclk_pin");
    ESP_RETURN_ON_FALSE(config->lrclk_pin != GPIO_NUM_NC, ESP_ERR_INVALID_ARG, TAG, "invalid lrclk_pin");
    ESP_RETURN_ON_FALSE(config->din_pin != GPIO_NUM_NC, ESP_ERR_INVALID_ARG, TAG, "invalid din_pin");
    
    /* 检查采样率范围 (8kHz - 96kHz) */
    ESP_RETURN_ON_FALSE(config->sample_rate >= 8000 && config->sample_rate <= 96000, 
                        ESP_ERR_INVALID_ARG, TAG, "sample_rate out of range (8000-96000)");

    esp_err_t ret = ESP_OK;
    
    /* 分配句柄内存 */
    max98357a_handle_t *dev = (max98357a_handle_t *)calloc(1, sizeof(max98357a_handle_t));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG, "no memory for handle");
    
    dev->sd_mode_pin = config->sd_mode_pin;
    dev->channel = config->channel;
    dev->sample_rate = config->sample_rate;
    dev->is_enabled = false;
    
    /* 配置SD_MODE引脚 */
    if (config->sd_mode_pin != GPIO_NUM_NC) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << config->sd_mode_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure SD_MODE pin");
            free(dev);
            return ret;
        }
        
        /* 初始状态设置为关断 */
        gpio_set_level(config->sd_mode_pin, SD_MODE_SHUTDOWN);
        ESP_LOGI(TAG, "SD_MODE pin configured on GPIO%d", config->sd_mode_pin);
    }
    
    /* 配置I2S标准模式 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(config->i2s_port, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;  /* 自动清除DMA缓冲区，避免杂音 */
    chan_cfg.dma_desc_num = 12;   /* DMA描述符数量，增大可降低下溢概率 */
    chan_cfg.dma_frame_num = 960; /* 每个DMA缓冲区的帧数，进一步平滑播放 */
    
    ret = i2s_new_channel(&chan_cfg, &dev->tx_handle, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        free(dev);
        return ret;
    }
    
    /* 配置I2S标准接口 */
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = config->sample_rate,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256, /* MCLK = 256 * sample_rate */
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,  /* MAX98357A不需要MCLK */
            .bclk = config->bclk_pin,
            .ws = config->lrclk_pin,
            .dout = config->din_pin,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    ret = i2s_channel_init_std_mode(dev->tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode: %s", esp_err_to_name(ret));
        i2s_del_channel(dev->tx_handle);
        free(dev);
        return ret;
    }
    
    /* 使能I2S通道 */
    ret = i2s_channel_enable(dev->tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        i2s_del_channel(dev->tx_handle);
        free(dev);
        return ret;
    }
    
    *handle = dev;
    
    ESP_LOGI(TAG, "MAX98357A initialized successfully");
    ESP_LOGI(TAG, "  Sample Rate: %lu Hz", config->sample_rate);
    ESP_LOGI(TAG, "  Bits/Sample: %d", config->bits_per_sample * 8);
    ESP_LOGI(TAG, "  Gain: %ddB", (config->gain * 3) + 3);
    ESP_LOGI(TAG, "  Channel: %s",
             config->channel == MAX98357A_CHANNEL_LEFT ? "Left" :
             config->channel == MAX98357A_CHANNEL_RIGHT ? "Right" : "Mixed");
    ESP_LOGI(TAG, "  DMA Buffers: %d x %d frames", chan_cfg.dma_desc_num, chan_cfg.dma_frame_num);
    
    return ESP_OK;
}

/**
 * @brief 反初始化MAX98357A
 */
esp_err_t max98357a_deinit(max98357a_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    
    /* 关断设备 */
    if (handle->is_enabled) {
        max98357a_disable(handle);
    }
    
    /* 禁用并删除I2S通道 */
    if (handle->tx_handle) {
        i2s_channel_disable(handle->tx_handle);
        i2s_del_channel(handle->tx_handle);
    }
    
    /* 释放内存 */
    free(handle);
    
    ESP_LOGI(TAG, "MAX98357A deinitialized");
    return ESP_OK;
}

/**
 * @brief 使能MAX98357A
 */
esp_err_t max98357a_enable(max98357a_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    
    if (handle->is_enabled) {
        ESP_LOGW(TAG, "Already enabled");
        return ESP_OK;
    }
    
    /* 先在I2S上预写一小段静音，让数据线稳定为0，再退出关断。
     * 否则SD_MODE拉高瞬间如果DMA里残留非零样本，容易产生“啪”的爆音。 */
    if (handle->tx_handle != NULL) {
        static const int16_t s_zeros[512] = {0}; /* 512 samples -> 2048 bytes stereo16 */
        size_t written = 0;
        // 尝试写两次，覆盖更多DMA边界
        (void)i2s_channel_write(handle->tx_handle, s_zeros, sizeof(s_zeros), &written, pdMS_TO_TICKS(100));
        (void)i2s_channel_write(handle->tx_handle, s_zeros, sizeof(s_zeros), &written, pdMS_TO_TICKS(100));
    }

    /* 设置SD_MODE为高电平以退出关断模式 */
    if (handle->sd_mode_pin != GPIO_NUM_NC) {
        gpio_set_level(handle->sd_mode_pin, SD_MODE_ENABLE);

        /* 等待启动时间(数据手册: 7.5ms典型值) */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    handle->is_enabled = true;
    ESP_LOGI(TAG, "MAX98357A enabled");
    
    return ESP_OK;
}

/**
 * @brief 关断MAX98357A
 */
esp_err_t max98357a_disable(max98357a_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    
    if (!handle->is_enabled) {
        ESP_LOGW(TAG, "Already disabled");
        return ESP_OK;
    }
    
    /* 设置SD_MODE为低电平以进入关断模式 */
    if (handle->sd_mode_pin != GPIO_NUM_NC) {
        gpio_set_level(handle->sd_mode_pin, SD_MODE_SHUTDOWN);
    }
    
    handle->is_enabled = false;
    ESP_LOGI(TAG, "MAX98357A disabled (shutdown mode)");
    
    return ESP_OK;
}

/**
 * @brief 设置声道
 */
esp_err_t max98357a_set_channel(max98357a_handle_t *handle, max98357a_channel_t channel)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(channel <= MAX98357A_CHANNEL_SHUTDOWN, ESP_ERR_INVALID_ARG, TAG, "invalid channel");
    
    if (channel == MAX98357A_CHANNEL_SHUTDOWN) {
        return max98357a_disable(handle);
    }
    
    handle->channel = channel;
    
    ESP_LOGI(TAG, "Channel set to: %s", 
             channel == MAX98357A_CHANNEL_LEFT ? "Left" :
             channel == MAX98357A_CHANNEL_RIGHT ? "Right" : "Mixed");
    
    ESP_LOGW(TAG, "Note: Channel selection requires hardware configuration (SD_MODE resistor)");
    
    return ESP_OK;
}

/**
 * @brief 写入音频数据
 */
esp_err_t max98357a_write(max98357a_handle_t *handle, 
                          const void *data, 
                          size_t size, 
                          size_t *bytes_written, 
                          uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is NULL");
    ESP_RETURN_ON_FALSE(size > 0, ESP_ERR_INVALID_ARG, TAG, "size is 0");
    
    if (!handle->is_enabled) {
        ESP_LOGW(TAG, "Device is disabled, enabling now");
        max98357a_enable(handle);
    }
    
    // 使用更长的超时时间
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (ticks < pdMS_TO_TICKS(100)) {
        ticks = pdMS_TO_TICKS(100);
    }
    
    esp_err_t ret = i2s_channel_write(handle->tx_handle, data, size, bytes_written, ticks);
    
    // 只在非超时错误时打印日志，减少日志刷屏
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed to write I2S data: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief 设置采样率
 */
esp_err_t max98357a_set_sample_rate(max98357a_handle_t *handle, uint32_t sample_rate)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(handle->tx_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "tx_handle is NULL (device not initialized)");
    ESP_RETURN_ON_FALSE(sample_rate >= 8000 && sample_rate <= 96000, 
                        ESP_ERR_INVALID_ARG, TAG, "sample_rate out of range (8000-96000)");
    
    // 如果采样率相同，无需更改
    if (handle->sample_rate == sample_rate) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Changing sample rate from %lu to %lu Hz", handle->sample_rate, sample_rate);
    
    // 需要先禁用通道才能重新配置时钟
    esp_err_t ret = i2s_channel_disable(handle->tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable channel for reconfig: %s", esp_err_to_name(ret));
        return ret;  // 如果禁用失败，不继续
    }
    
    // 等待 DMA 完成
    vTaskDelay(pdMS_TO_TICKS(20));
    
    /* 重新配置I2S时钟 */
    i2s_std_clk_config_t clk_cfg = {
        .sample_rate_hz = sample_rate,
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    };
    
    ret = i2s_channel_reconfig_std_clock(handle->tx_handle, &clk_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reconfigure sample rate: %s", esp_err_to_name(ret));
        // 尝试重新启用通道
        i2s_channel_enable(handle->tx_handle);
        return ret;
    }
    
    // 等待时钟稳定
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 重新启用通道
    ret = i2s_channel_enable(handle->tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-enable channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    handle->sample_rate = sample_rate;
    ESP_LOGI(TAG, "Sample rate changed to %lu Hz", sample_rate);
    
    return ESP_OK;
}

/**
 * @brief 获取当前状态
 */
esp_err_t max98357a_get_status(max98357a_handle_t *handle, 
                               bool *is_enabled, 
                               max98357a_channel_t *channel)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    
    if (is_enabled != NULL) {
        *is_enabled = handle->is_enabled;
    }
    
    if (channel != NULL) {
        *channel = handle->channel;
    }
    
    return ESP_OK;
}