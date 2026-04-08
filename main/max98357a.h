/**
 * @file max98357a.h
 * @brief MAX98357A I2S数字音频功率放大器驱动
 * 
 * 特性:
 * - 支持I2S数字音频接口
 * - 无需MCLK信号
 * - 支持8kHz-96kHz采样率
 * - 支持16/24/32位数据
 * - 5种增益设置(3dB, 6dB, 9dB, 12dB, 15dB)
 * - 左/右/混合声道选择
 * - 低功耗关断模式
 * 
 * 硬件连接:
 * - DIN:   I2S数据输入
 * - BCLK:  位时钟输入
 * - LRCLK: 帧时钟输入(左右声道选择)
 * - SD_MODE: 关断和声道选择
 * - GAIN_SLOT: 增益选择
 * - VDD:   电源(2.5V-5.5V)
 * - OUTP/OUTN: 差分扬声器输出
 */

#ifndef MAX98357A_H
#define MAX98357A_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MAX98357A增益设置
 * 根据数据手册Table 8
 */
typedef enum {
    MAX98357A_GAIN_3DB = 0,   /*!< 3dB增益 - GAIN_SLOT通过100kΩ连接到VDD */
    MAX98357A_GAIN_6DB,       /*!< 6dB增益 - GAIN_SLOT直接连接到VDD */
    MAX98357A_GAIN_9DB,       /*!< 9dB增益 - GAIN_SLOT悬空 */
    MAX98357A_GAIN_12DB,      /*!< 12dB增益 - GAIN_SLOT连接到GND */
    MAX98357A_GAIN_15DB       /*!< 15dB增益 - GAIN_SLOT通过100kΩ连接到GND */
} max98357a_gain_t;

/**
 * @brief MAX98357A声道选择
 * 根据数据手册Table 5
 */
typedef enum {
    MAX98357A_CHANNEL_LEFT = 0,     /*!< 左声道 - SD_MODE直接连接到高电平 */
    MAX98357A_CHANNEL_RIGHT,        /*!< 右声道 - SD_MODE通过RSMALL上拉 */
    MAX98357A_CHANNEL_MIXED,        /*!< 混合声道(L/2+R/2) - SD_MODE通过RLARGE上拉 */
    MAX98357A_CHANNEL_SHUTDOWN      /*!< 关断模式 - SD_MODE为低电平 */
} max98357a_channel_t;

/**
 * @brief MAX98357A配置结构体
 */
typedef struct {
    /* I2S配置 */
    i2s_port_t i2s_port;              /*!< I2S端口号 */
    gpio_num_t bclk_pin;              /*!< BCLK引脚 */
    gpio_num_t lrclk_pin;             /*!< LRCLK引脚 */
    gpio_num_t din_pin;               /*!< DIN引脚 */
    
    /* 控制引脚 */
    gpio_num_t sd_mode_pin;           /*!< SD_MODE引脚(关断和声道选择) */
    
    /* 音频参数 */
    uint32_t sample_rate;             /*!< 采样率(8000-96000Hz) */
    i2s_data_bit_width_t bits_per_sample; /*!< 位宽(16/24/32位) */
    
    /* 增益和声道(硬件配置,仅用于文档说明) */
    max98357a_gain_t gain;            /*!< 增益设置(通过硬件GAIN_SLOT配置) */
    max98357a_channel_t channel;      /*!< 声道选择(通过SD_MODE控制) */
} max98357a_config_t;

/**
 * @brief MAX98357A驱动句柄
 */
typedef struct {
    i2s_chan_handle_t tx_handle;     /*!< I2S发送通道句柄 */
    gpio_num_t sd_mode_pin;          /*!< SD_MODE引脚 */
    max98357a_channel_t channel;     /*!< 当前声道设置 */
    uint32_t sample_rate;            /*!< 采样率 */
    bool is_enabled;                 /*!< 使能状态 */
} max98357a_handle_t;

/**
 * @brief 获取默认配置
 * 
 * @return 默认配置结构体
 */
max98357a_config_t max98357a_get_default_config(void);

/**
 * @brief 初始化MAX98357A
 * 
 * @param config 配置参数
 * @param handle 返回的驱动句柄
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 *     - ESP_ERR_NO_MEM: 内存不足
 *     - 其他: I2S初始化错误
 */
esp_err_t max98357a_init(const max98357a_config_t *config, max98357a_handle_t **handle);

/**
 * @brief 反初始化MAX98357A
 * 
 * @param handle 驱动句柄
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 */
esp_err_t max98357a_deinit(max98357a_handle_t *handle);

/**
 * @brief 使能MAX98357A(退出关断模式)
 * 
 * @param handle 驱动句柄
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 */
esp_err_t max98357a_enable(max98357a_handle_t *handle);

/**
 * @brief 关断MAX98357A(进入低功耗模式)
 * 
 * @param handle 驱动句柄
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 */
esp_err_t max98357a_disable(max98357a_handle_t *handle);

/**
 * @brief 设置声道
 * 
 * @param handle 驱动句柄
 * @param channel 声道选择
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 * 
 * @note 声道切换需要硬件支持(SD_MODE引脚需要通过不同阻值上拉)
 *       如果硬件固定连接,此函数仅更新软件状态
 */
esp_err_t max98357a_set_channel(max98357a_handle_t *handle, max98357a_channel_t channel);

/**
 * @brief 写入音频数据
 * 
 * @param handle 驱动句柄
 * @param data 音频数据缓冲区
 * @param size 数据大小(字节)
 * @param bytes_written 实际写入的字节数
 * @param timeout_ms 超时时间(毫秒)
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 *     - ESP_ERR_TIMEOUT: 超时
 */
esp_err_t max98357a_write(max98357a_handle_t *handle, 
                          const void *data, 
                          size_t size, 
                          size_t *bytes_written, 
                          uint32_t timeout_ms);

/**
 * @brief 设置采样率
 * 
 * @param handle 驱动句柄
 * @param sample_rate 新的采样率(8000-96000Hz)
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 */
esp_err_t max98357a_set_sample_rate(max98357a_handle_t *handle, uint32_t sample_rate);

/**
 * @brief 播放前执行一次输出链路恢复
 *
 * @param handle 驱动句柄
 * @param sample_rate 目标采样率
 * @return
 *     - ESP_OK: 成功
 *     - 其他: 恢复失败
 */
esp_err_t max98357a_prepare_output(max98357a_handle_t *handle, uint32_t sample_rate);

/**
 * @brief 获取当前状态
 * 
 * @param handle 驱动句柄
 * @param is_enabled 返回使能状态
 * @param channel 返回当前声道
 * @return 
 *     - ESP_OK: 成功
 *     - ESP_ERR_INVALID_ARG: 参数错误
 */
esp_err_t max98357a_get_status(max98357a_handle_t *handle, 
                               bool *is_enabled, 
                               max98357a_channel_t *channel);

#ifdef __cplusplus
}
#endif

#endif /* MAX98357A_H */
