# ESP32 智慧农业交互传感器项目

本项目是一个基于 ESP32-S3 芯片的智能农业环境监测系统，集成了多种传感器数据采集、AI语音交互（大模型分析）、LCD 本地展示和 Web/云端远程状态同步等功能。

## 🛠️ 硬件需求
- **核心板**: ESP32-S3 (推荐 N16R8)
- **显示屏**: ST7735S TFT 彩色 LCD 屏 (128x160)
- **音频输出**: MAX98357A I2S 音频功放模块 + 扬声器
- **传感器**: 兼容 UART 协议的环境/土壤传感器（例如通过 A39C LoRa 模块透传的数据，含光照、温湿度、大气压、土壤NPK、pH等）

## 📁 文件及代码架构

本系统的所有业务层代码均存放在 `main/` 目录下：

- **`main.c` / `CMakeLists.txt`**
  入口主程序和构建脚本。包含了设备初始化、WiFi连接、传感器数据获取以及统筹各种系统任务的核心大循环。
- **`ai_config.h`**
  **（核心配置文件）**：包含所有需要在编译前修改的宏定义，比如百度 TTS、AI 大模型（星火/豆包）、高德地图模块的 API Key，还有屏幕与 I2S 音频模块的硬件引脚（GPIO）配对声明。
- **`st7735_display.c` & `st7735_display.h` & `char_bitmap.h`**
  显示模块：ST7735S 屏幕的 SPI 层驱动代码与 GUI 显示逻辑。`char_bitmap.h` 是为了让这块屏幕能够显示中文而预先生成的 12x12 文字点阵库。
- **`max98357a.c` & `max98357a.h`**
  音频输出：使用 ESP32 的 I2S 硬件接口驱动 MAX98357A，将数字音频流转换为发声信号。
- **`baidu_tts.c` & `baidu_tts.h`**
  语音合成模块：将 AI 模型生成的文本送给百度开放平台的 TTS 服务，获取到音频流后丢给 `max98357a` 模块进行播报。
- **`spark_chat.c` & `spark_chat.h`**
  AI 模型模块：负责与科大讯飞星火大模型建立 WebSocket 连接。把农作物的当前实时数据以及用户的自然语言请求提交给大模型，获取对应的分析指导。
- **`index.html`**
  板载 Web UI：用于在局域网内通过浏览器访问 ESP32，直接在图形化界面查看当前各项传感器指标并发起交互逻辑。

## 🚀 如何编译和运行本项目

### 第一步：搭建 ESP-IDF 开发环境
本项目基于 Espressif 官方的 **ESP-IDF v5.x** 框架构建。如果你还没安装：
1. 请前往乐鑫官方网站下载并安装 [ESP-IDF 离线/在线编译器](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/index.html)。
2. 在 VS Code 中推荐安装官方 `Espressif IDF` 插件以便于直接在编辑器内开发。

### 第二步：克隆项目代码
```bash
git clone https://github.com/hdhdyuxbsu/-.git
cd -  # 进入你 clone 下来的仓库目录
```

### 第三步：硬件接线与修改配置
在编译前，你**必须**设置你自己的各项鉴权 Key 并且核对引脚物理连接：
1. 找到并打开 `main/ai_config.h` 文件。
2. 填入你自己申请的大模型（如豆包/星火）、高德天气、百度语音 TTS 的 **API KEY**。
3. 检查宏定义 `AUDIO_***_PIN`（音频引脚）与 `LCD_***_PIN`（屏幕 SPI 引脚），确保与你实际物理的面包板接线一一对应。

### 第四步：设定目标芯片并编译
打开 ESP-IDF 终端（ESP-IDF Terminal），或者在 VS Code 的底侧栏点击对应的 IDF 工具图标执行以下命令：
```bash
# 1. 设定当前芯片型号为 ESP32-S3
idf.py set-target esp32s3

# 2. 如果需修改 WiFi 账密或其他底层设置可拉起配置菜单（可选）
idf.py menuconfig

# 3. 编译工程（第一次编译可能需要 1~3 分钟）
idf.py build
```

### 第五步：烧录与监控
使用 Type-C 数据线将 ESP32-S3 板子连接到电脑，找到对应的电脑端口端号（比如 `COM3` 或 `/dev/ttyUSB0`）。
```bash
# 烧录固件并立即监控串口打印的实时运行信息（按 Ctrl+] 可以退出监控面板）
idf.py -p COM3 flash monitor
```

## ⚠️ 注意事项（避坑指南）
1. **内存与芯片限制**：大模型的网络握手交互加上音频流转码解码极度耗费内存。当前要求板子务必带有 PSRAM（推荐 N16R8 型号），并确保已在 `menuconfig` 中将使用外部 PSRAM 特性开启。
2. **引脚冲突避坑**：ESP32-S3 核心板内置的 `GPIO26` 到 `GPIO37` 引脚通常会被板载的 Octal Flash / PSRAM 的 SPI 总线强制占用，**绝对不可以用作普通外设的接线**，否则直接导致疯狂无限重启或者烧录失败！
