# ESP32 农业传感器项目 - Copilot 自定义指令

## 项目概述
这是一个基于 ESP32-S3 的智能农业环境监测系统，集成了多种传感器采集、AI语音交互、LCD显示和云平台上报功能。

## 技术栈
- **硬件平台**: ESP32-S3 N16R8
- **开发框架**: ESP-IDF v5.5.1
- **编程语言**: C
- **通信协议**: MQTT (OneNet), WebSocket (讯飞), HTTP (百度TTS)

## 核心模块

### 1. 传感器数据采集
通过 UART 与 A39C LoRa 模块通信，接收远端传感器数据：
- 环境温湿度、光照强度、大气压力
- 土壤温湿度、pH值、NPK养分含量

### 2. AI 语音交互
- **讯飞星火**: WebSocket 流式对话
- **豆包**: HTTP REST API 对话
- **百度TTS**: 语音合成播报

### 3. 显示系统
- ST7735 LCD 显示屏
- 12x12 中文点阵字库 (char_bitmap.h)

### 4. 云平台
- OneNet MQTT 物联网平台
- 实时数据上报与远程控制

## 编码规范

### 命名约定
- 函数名: `snake_case` (如 `sensor_data_process`)
- 变量名: `snake_case` (如 `env_temp_c`)
- 常量: `UPPER_SNAKE_CASE` (如 `WIFI_MAX_RETRY`)
- 类型定义: `snake_case_t` (如 `sensor_packet_t`)

### 错误处理
```c
esp_err_t ret = some_function();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error: %s", esp_err_to_name(ret));
    return ret;
}
```

### FreeRTOS 任务
```c
void task_function(void *pvParameters) {
    while (1) {
        // 任务逻辑
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// 栈大小至少 4096
xTaskCreate(task_function, "task_name", 4096, NULL, 5, NULL);
```

## 常用命令
```bash
idf.py build              # 编译项目
idf.py -p COM18 flash     # 烧录固件
idf.py -p COM18 monitor   # 串口监控
idf.py fullclean          # 完全清理
idf.py menuconfig         # 配置菜单
```

## 文件结构
```
main/
├── main.c              # 主程序入口
├── ai_config.h         # AI服务密钥配置
├── spark_chat.c/h      # 讯飞星火对话
├── doubao_chat.c/h     # 豆包对话
├── baidu_tts.c/h       # 百度语音合成
├── max98357a.c/h       # 音频DAC驱动
├── st7735_display.c/h  # LCD显示驱动
└── char_bitmap.h       # 中文点阵字库
```

## 传感器数据帧格式
- 帧头: `0xAA 0x55`
- 载荷长度: `0x19` (25字节)
- 数据结构: `sensor_packet_t`

## 注意事项
1. GPIO26-37 被 Octal Flash/PSRAM 占用，不可使用
2. 在 ISR 中禁止使用阻塞调用
3. 注意内存管理，避免堆栈溢出
4. API 密钥保存在 ai_config.h，请勿提交到公开仓库
