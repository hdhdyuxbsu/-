# GitHub Copilot Agent Skills 使用指南

本文档介绍如何在 ESP32 农业传感器项目中使用 GitHub Copilot 的自定义 Agent Skills。

## 📁 配置文件

项目中包含两个 Copilot 配置文件：

```
.github/
├── copilot-instructions.md   # 通用项目指令 (Copilot 自动读取)
└── copilot-agents.yaml       # 自定义 Agent 定义
```

## 🎯 配置文件说明

### 1. copilot-instructions.md

这是 GitHub Copilot 的**官方支持格式**。当你在 VS Code、GitHub.com 或 Copilot CLI 中使用 Copilot 时，它会自动读取这个文件作为项目上下文。

**主要内容：**
- 项目技术栈和架构概述
- 编码规范和命名约定
- 常用命令
- 注意事项和约束

### 2. copilot-agents.yaml

这是**自定义 Agent 配置**，定义了专门针对本项目的 AI 助手角色。

**包含的 Agent：**

| Agent 名称 | 描述 | 使用场景 |
|-----------|------|----------|
| `esp32-expert` | ESP32-S3 开发专家 | 驱动开发、FreeRTOS、外设配置 |
| `sensor-analyzer` | 传感器数据分析师 | 数据解读、阈值设置、农业建议 |
| `display-helper` | LCD 显示助手 | UI 设计、汉字点阵、显示优化 |
| `ai-integrator` | AI 服务集成专家 | 语音对话、TTS、API 集成 |

## 🚀 如何使用

### 方法一：VS Code 中使用 (推荐)

1. **确保配置文件存在**
   ```
   .github/copilot-instructions.md
   ```

2. **在 Copilot Chat 中提问**
   
   Copilot 会自动读取 `copilot-instructions.md`，获得项目上下文。
   
   示例问题：
   - "如何添加一个新的传感器类型？"
   - "帮我实现 MQTT 断线重连逻辑"
   - "优化 LCD 刷新性能"

### 方法二：GitHub Copilot CLI 中使用

1. **安装 Copilot CLI**
   ```bash
   npm install -g @anthropic/copilot-cli
   # 或
   gh extension install github/gh-copilot
   ```

2. **在项目目录运行**
   ```bash
   cd E:\ESP32\dachuang
   copilot
   ```

3. **使用自然语言交互**
   ```
   > 帮我实现 WiFi 断线自动重连
   > 解释 sensor_packet_t 结构体
   > 如何添加新的汉字到字库
   ```

### 方法三：手动引用 Agent 指令

在任何 AI 助手中，你可以复制 `copilot-agents.yaml` 中的 `system_prompt` 内容作为系统提示词。

例如，使用 ESP32 专家 Agent：

```
将 esp32-expert 的 system_prompt 内容粘贴到 AI 助手的系统提示中
```

## 📝 自定义你的 Skills

### 添加新的 Agent

编辑 `.github/copilot-agents.yaml`，添加新的 Agent 定义：

```yaml
agents:
  - name: "my-custom-agent"
    description: "自定义 Agent 描述"
    model: "claude-sonnet-4"
    system_prompt: |
      你的专业领域是...
      
      ## 技术要求
      - 要点 1
      - 要点 2
      
      ## 任务
      帮助用户完成...
```

### 修改项目指令

编辑 `.github/copilot-instructions.md`，添加新的上下文信息：

- 新增的模块说明
- 更新的编码规范
- 特殊注意事项

## 🔧 实用示例

### 示例 1：请求代码审查

```
> 请检查 main.c 中的内存泄漏风险
```

### 示例 2：生成新功能

```
> 帮我添加一个定时上报传感器数据到 OneNet 的功能，每 30 秒上报一次
```

### 示例 3：调试帮助

```
> MQTT 连接总是失败，错误码 -1，可能是什么原因？
```

### 示例 4：优化建议

```
> 如何减少 LCD 刷新时的闪烁？
```

## ⚠️ 注意事项

1. **API 密钥安全**
   - `ai_config.h` 包含敏感信息，已在 `.gitignore` 中排除
   - 不要在 Copilot 对话中暴露真实密钥

2. **硬件约束**
   - GPIO26-37 不可用 (PSRAM 占用)
   - 串口使用 COM18

3. **内存限制**
   - ESP32-S3 有 8MB PSRAM，但仍需注意大数组的分配
   - 使用 `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)` 分配 PSRAM

## 📚 相关资源

- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [GitHub Copilot 文档](https://docs.github.com/copilot)
- [OneNet 开发文档](https://open.iot.10086.cn/doc/v5)

---

Happy Coding with AI! 🤖
