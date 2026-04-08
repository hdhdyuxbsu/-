/**
 * @file baidu_tts.c
 * @brief 百度TTS语音合成驱动实现
 */

#include "baidu_tts.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "BaiduTTS";

#define TTS_PCM_TARGET_PEAK 15000
#define TTS_INPUT_GAIN      0.55f
#define TTS_DC_BLOCK_R      0.995f

/* TTS请求互斥锁 - 避免并发HTTP/SSL连接导致资源竞争 */
static SemaphoreHandle_t s_tts_request_mutex = NULL;

/* 百度TTS API地址 */
#define BAIDU_TOKEN_URL "https://aip.baidubce.com/oauth/2.0/token"
#define BAIDU_TTS_URL   "https://tsn.baidu.com/text2audio"

/* Token有效期（秒），提前5分钟刷新 */
#define TOKEN_VALID_DURATION (30 * 24 * 3600 - 300)

/* HTTP响应初始缓冲区大小，按需扩容 */
#define HTTP_RECV_BUFFER_INITIAL_SIZE (256 * 1024)

/* 前向声明 */
static esp_err_t baidu_tts_speak_segment(baidu_tts_handle_t *handle, const char *text);

static esp_err_t baidu_tts_speak_segment_with_retry(baidu_tts_handle_t *handle,
                                                    const char *text,
                                                    int retry_times)
{
    esp_err_t ret = ESP_FAIL;
    int max_try = (retry_times < 1) ? 1 : retry_times;
    for (int i = 0; i < max_try; i++) {
        ret = baidu_tts_speak_segment(handle, text);
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "TTS segment failed, retry %d/%d", i + 1, max_try);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    return ret;
}

/**
 * @brief HTTP事件处理回调（用于接收音频数据）
 */
typedef struct {
    uint8_t *buffer;
    size_t buffer_capacity;
    size_t data_len;
    bool is_audio;
    bool truncated;
    max98357a_handle_t *audio_handle;  // 用于流式播放
    bool streaming;  // 是否启用流式播放
    int16_t *stereo_buf;               // 流式播放用的立体声缓冲
    size_t stereo_buf_size;
} http_audio_context_t;

static size_t wav_find_pcm_data_offset(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len < 12) {
        return 0;
    }

    // WAV: "RIFF" + size + "WAVE"
    if (memcmp(buf, "RIFF", 4) != 0 || memcmp(buf + 8, "WAVE", 4) != 0) {
        return 0;
    }

    // Scan chunks: [4-byte id][4-byte size][payload...]
    size_t off = 12;
    while (off + 8 <= len) {
        const uint8_t *ck = buf + off;
        uint32_t ck_size = (uint32_t)ck[4] |
                           ((uint32_t)ck[5] << 8) |
                           ((uint32_t)ck[6] << 16) |
                           ((uint32_t)ck[7] << 24);
        size_t payload = off + 8;
        if (payload > len) {
            return 0;
        }
        if (memcmp(ck, "data", 4) == 0) {
            if (payload <= len) {
                return payload;
            }
            return 0;
        }

        // Chunk sizes are padded to even.
        size_t advance = 8 + (size_t)ck_size;
        if (advance & 1) {
            advance++;
        }
        if (advance == 0) {
            return 0;
        }
        off += advance;
    }

    return 0;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    http_audio_context_t *ctx = (http_audio_context_t *)evt->user_data;
    
    switch (evt->event_id) {
        case HTTP_EVENT_ON_HEADER:
            // 检查Content-Type判断是否为音频数据
            if (strcasecmp(evt->header_key, "Content-Type") == 0) {
                if (strstr(evt->header_value, "audio") != NULL) {
                    ctx->is_audio = true;
                    ESP_LOGI(TAG, "Receiving audio data");
                } else {
                    ctx->is_audio = false;
                    ESP_LOGW(TAG, "Response is not audio: %s", evt->header_value);
                }
            }
            break;
            
        case HTTP_EVENT_ON_DATA:
            // 缓冲模式: 接收所有数据后再播放（音质更好，无噪音）
            // 如果空间不足，尝试扩容（倍增），避免丢帧
            if (ctx->data_len + evt->data_len > ctx->buffer_capacity) {
                size_t needed = ctx->data_len + evt->data_len;
                size_t new_cap = ctx->buffer_capacity;
                if (new_cap == 0) {
                    new_cap = HTTP_RECV_BUFFER_INITIAL_SIZE;
                }
                while (new_cap < needed) {
                    new_cap *= 2;
                }

                uint8_t *new_buf = heap_caps_realloc(ctx->buffer, new_cap,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (new_buf == NULL) {
                    new_buf = realloc(ctx->buffer, new_cap);
                }

                if (new_buf) {
                    ctx->buffer = new_buf;
                    ctx->buffer_capacity = new_cap;
                    ESP_LOGW(TAG, "Audio buffer expanded to %u bytes", (unsigned)new_cap);
                } else {
                    // 不要静默截断，否则会导致“播放不完整”且很难定位。
                    ctx->truncated = true;
                    ESP_LOGE(TAG, "Buffer overflow: need=%u cap=%u, aborting TTS download",
                             (unsigned)needed, (unsigned)ctx->buffer_capacity);
                    return ESP_FAIL;
                }
            }

            memcpy(ctx->buffer + ctx->data_len, evt->data, evt->data_len);
            ctx->data_len += evt->data_len;
            break;
            
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP error");
            break;
            
        default:
            break;
    }
    
    return ESP_OK;
}

/**
 * @brief URL编码
 */
static void url_encode(const char *src, char *dst, size_t dst_size) {
    const char *hex = "0123456789ABCDEF";
    size_t dst_idx = 0;
    
    for (size_t i = 0; src[i] != '\0' && dst_idx < dst_size - 1; i++) {
        unsigned char c = (unsigned char)src[i];
        
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || 
            (c >= '0' && c <= '9') || c == '-' || c == '_' || 
            c == '.' || c == '~') {
            dst[dst_idx++] = c;
        } else {
            if (dst_idx + 3 < dst_size) {
                dst[dst_idx++] = '%';
                dst[dst_idx++] = hex[c >> 4];
                dst[dst_idx++] = hex[c & 0x0F];
            }
        }
    }
    dst[dst_idx] = '\0';
}

esp_err_t baidu_tts_init(baidu_tts_handle_t *handle, 
                         const baidu_tts_config_t *config,
                         max98357a_handle_t *audio_handle) {
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->api_key == NULL || config->secret_key == NULL) {
        ESP_LOGE(TAG, "API Key or Secret Key is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(handle, 0, sizeof(baidu_tts_handle_t));
    handle->config = *config;
    
    // 深拷贝API Key和Secret Key，避免指针失效
    strncpy(handle->api_key, config->api_key, sizeof(handle->api_key) - 1);
    strncpy(handle->secret_key, config->secret_key, sizeof(handle->secret_key) - 1);
    handle->config.api_key = handle->api_key;
    handle->config.secret_key = handle->secret_key;
    
    handle->audio_handle = audio_handle;  // 允许为NULL，稍后设置
    handle->token_expire_time = 0;
    
    ESP_LOGI(TAG, "API Key: %.8s***, Secret Key: %.8s***", 
             handle->api_key, handle->secret_key);
    
    // 设置默认值
    if (handle->config.speed == 0) handle->config.speed = 5;
    if (handle->config.pitch == 0) handle->config.pitch = 5;
    if (handle->config.volume == 0) handle->config.volume = 5;
    if (handle->config.timeout_ms == 0) handle->config.timeout_ms = 30000;
    
    // 创建TTS请求互斥锁（如果尚未创建）
    if (s_tts_request_mutex == NULL) {
        s_tts_request_mutex = xSemaphoreCreateMutex();
        if (s_tts_request_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create TTS request mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    ESP_LOGI(TAG, "Baidu TTS initialized");
    return ESP_OK;
}

esp_err_t baidu_tts_get_token(baidu_tts_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 检查token是否还有效
    int64_t now = esp_timer_get_time();
    if (handle->token_expire_time > now && strlen(handle->access_token) > 0) {
        ESP_LOGI(TAG, "Token still valid");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Getting new access token...");
    
    // 构建请求URL - 使用深拷贝的key
    char url[512];
    snprintf(url, sizeof(url), 
             "%s?grant_type=client_credentials&client_id=%s&client_secret=%s",
             BAIDU_TOKEN_URL, handle->api_key, handle->secret_key);
    
    ESP_LOGI(TAG, "Token URL: %s...%s", 
             BAIDU_TOKEN_URL, "（已隐藏key）");
    
    // 准备接收响应的缓冲区（使用堆分配避免栈溢出）
    char *response = malloc(2048);
    if (response == NULL) {
        ESP_LOGE(TAG, "Failed to allocate response buffer");
        return ESP_ERR_NO_MEM;
    }
    memset(response, 0, 2048);
    int response_len = 0;
    
    // 配置HTTP客户端（不使用perform，手动控制）
    // 注意：如果RTC时间不正确，证书验证可能失败
    uint32_t timeout = handle->config.timeout_ms;
    if (timeout > 15000) timeout = 15000;  // 限制超时
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = timeout,
        // 临时跳过证书验证以解决SSL握手失败问题
        .crt_bundle_attach = esp_crt_bundle_attach,
        .cert_pem = NULL,
        .skip_cert_common_name_check = true,
        .use_global_ca_store = false,
        .buffer_size = 2048,
        .buffer_size_tx = 1024,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }
    
    // 打开连接
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }
    
    // 获取响应头
    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    
    ESP_LOGI(TAG, "HTTP status: %d, content_length: %d", status_code, content_length);
    
    if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP status code: %d", status_code);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    
    // 读取响应数据
    if (content_length > 0 && content_length < 2047) {
        response_len = esp_http_client_read(client, response, content_length);
    } else {
        // 如果content_length未知或过大，尽可能多读取
        response_len = esp_http_client_read(client, response, 2047);
    }
    
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    
    if (response_len <= 0) {
        ESP_LOGE(TAG, "Failed to read response, read_len: %d", response_len);
        free(response);
        return ESP_FAIL;
    }
    
    response[response_len] = '\0';
    
    // 打印响应内容用于调试（只打印前200字符）
    ESP_LOGI(TAG, "Token response (%d bytes): %.200s...", response_len, response);
    
    // 直接查找access_token字段（避免完整JSON解析）
    const char *token_start = strstr(response, "\"access_token\":\"");
    if (token_start == NULL) {
        ESP_LOGE(TAG, "No access_token field in response");
        free(response);
        return ESP_FAIL;
    }
    
    // 跳过 "access_token":"
    token_start += 16;
    
    // 查找结束引号
    const char *token_end = strchr(token_start, '"');
    if (token_end == NULL) {
        ESP_LOGE(TAG, "Invalid access_token format");
        free(response);
        return ESP_FAIL;
    }
    
    // 计算token长度
    size_t token_len = token_end - token_start;
    if (token_len >= sizeof(handle->access_token)) {
        ESP_LOGE(TAG, "Access token too long: %d bytes", token_len);
        free(response);
        return ESP_FAIL;
    }
    
    // 复制token
    memcpy(handle->access_token, token_start, token_len);
    handle->access_token[token_len] = '\0';
    
    // 设置过期时间
    handle->token_expire_time = esp_timer_get_time() +
                                (int64_t)TOKEN_VALID_DURATION * 1000000LL;
    
    free(response);
    ESP_LOGI(TAG, "Access token obtained successfully");
    return ESP_OK;
}

esp_err_t baidu_tts_speak(baidu_tts_handle_t *handle, const char *text) {
    if (handle == NULL || text == NULL || strlen(text) == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 获取TTS请求锁，避免并发HTTP和并发I2S播放造成爆音
    if (s_tts_request_mutex != NULL) {
        ESP_LOGI(TAG, "Waiting for TTS mutex");
        if (xSemaphoreTake(s_tts_request_mutex, pdMS_TO_TICKS(60000)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to acquire TTS request mutex");
            return ESP_ERR_TIMEOUT;
        }
        ESP_LOGI(TAG, "TTS mutex acquired");
    }

    esp_err_t final_ret = ESP_OK;

    // 百度TTS文本长度限制（UTF-8编码，约500字符）
    const size_t MAX_TEXT_LEN = 400;  // 降低一点，确保安全
    size_t text_len = strlen(text);
    
    // 如果文本太长，分段处理
    if (text_len > MAX_TEXT_LEN) {
        ESP_LOGI(TAG, "Text too long (%d bytes), splitting into segments", text_len);
        
        const char *p = text;
        char segment[MAX_TEXT_LEN + 10];
        int segment_count = 0;
        
        while (*p) {
            // 跳过开头的空白
            while (*p == ' ' || *p == '\n' || *p == '\r') p++;
            if (!*p) break;
            
            // 找到合适的分割点
            size_t seg_len = 0;
            const char *end = p;
            const char *last_break = NULL;
            
            while (*end && seg_len < MAX_TEXT_LEN) {
                char c = *end;
                
                // 英文句子结束符（句号、问号、感叹号后跟空格或结尾）
                if ((c == '.' || c == '!' || c == '?') && 
                    (*(end+1) == ' ' || *(end+1) == '\n' || *(end+1) == '\0' || *(end+1) == '"')) {
                    last_break = end + 1;
                }
                // 英文逗号、分号后跟空格
                else if ((c == ',' || c == ';' || c == ':') && *(end+1) == ' ') {
                    last_break = end + 1;
                }
                // 换行符
                else if (c == '\n') {
                    last_break = end + 1;
                }
                // 中文标点：。，！？、；：
                else if ((unsigned char)c == 0xE3 && (unsigned char)*(end+1) == 0x80) {
                    unsigned char c3 = (unsigned char)*(end+2);
                    if (c3 == 0x82 || c3 == 0x81 || c3 == 0x8C || c3 == 0x8D ||  // 。、，
                        c3 == 0x9B || c3 == 0x9C || c3 == 0x9D) {  // ！？
                        last_break = end + 3;
                    }
                }
                // 中文句号（另一种编码）
                else if ((unsigned char)c == 0xEF && (unsigned char)*(end+1) == 0xBC) {
                    last_break = end + 3;
                }
                
                // UTF-8字符长度
                if ((c & 0x80) == 0) {
                    end++;
                    seg_len++;
                } else if ((c & 0xE0) == 0xC0) {
                    end += 2;
                    seg_len += 2;
                } else if ((c & 0xF0) == 0xE0) {
                    end += 3;
                    seg_len += 3;
                } else if ((c & 0xF8) == 0xF0) {
                    end += 4;
                    seg_len += 4;
                } else {
                    end++;
                    seg_len++;
                }
            }
            
            // 使用最后一个分隔点，或者强制截断
            const char *seg_end;
            if (last_break && last_break > p && (last_break - p) >= 50) {
                // 使用分隔点，但确保段落不会太短
                seg_end = last_break;
            } else {
                seg_end = end;
            }
            seg_len = seg_end - p;
            
            // 复制段落
            if (seg_len > sizeof(segment) - 1) {
                seg_len = sizeof(segment) - 1;
            }
            memcpy(segment, p, seg_len);
            segment[seg_len] = '\0';
            
            // 去除末尾空白
            while (seg_len > 0 && (segment[seg_len-1] == ' ' || segment[seg_len-1] == '\n')) {
                segment[--seg_len] = '\0';
            }
            
            if (seg_len > 0) {
                // 播放这一段
                segment_count++;
                ESP_LOGI(TAG, "Playing segment %d (%d bytes): %.40s...", segment_count, seg_len, segment);
                esp_err_t ret = baidu_tts_speak_segment_with_retry(handle, segment, 3);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Segment %d playback failed: %s", segment_count, esp_err_to_name(ret));
                    final_ret = ret;
                }
                
                // 段落间短暂停顿
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            
            p = seg_end;
        }
        
        ESP_LOGI(TAG, "Finished playing %d segments", segment_count);
        if (s_tts_request_mutex != NULL) {
            xSemaphoreGive(s_tts_request_mutex);
        }
        return final_ret;
    }
    
    // 文本不长，直接播放
    final_ret = baidu_tts_speak_segment_with_retry(handle, text, 3);
    if (s_tts_request_mutex != NULL) {
        xSemaphoreGive(s_tts_request_mutex);
    }
    return final_ret;
}

/**
 * @brief 播放单个TTS段落（内部函数）
 */
static esp_err_t baidu_tts_speak_segment(baidu_tts_handle_t *handle, const char *text) {
    if (handle == NULL || text == NULL || strlen(text) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->audio_handle == NULL) {
        ESP_LOGE(TAG, "audio_handle is NULL, cannot play TTS audio");
        return ESP_ERR_INVALID_STATE;
    }

    // 禁用流式播放，使用缓冲模式以获得更好的音质（避免噪音）
    const bool enable_streaming = false;
    
    // 检查可用内存（流式模式占用更小）
    size_t free_heap = esp_get_free_heap_size();
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Free heap before TTS: %lu bytes (PSRAM: %lu bytes)", free_heap, free_psram);
    
    if (!enable_streaming && free_heap < 220000) {
        ESP_LOGW(TAG, "Free heap low (%lu bytes), continue and rely on dynamic allocation", free_heap);
    }
    
    // 确保有有效的token
    esp_err_t ret = baidu_tts_get_token(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get access token");
        return ret;
    }
    
    ESP_LOGI(TAG, "Synthesizing speech: %.50s%s", text, strlen(text) > 50 ? "..." : "");
    
    // URL编码文本
    char *encoded_text = malloc(strlen(text) * 3 + 1);
    if (encoded_text == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for encoded text");
        return ESP_ERR_NO_MEM;
    }
    url_encode(text, encoded_text, strlen(text) * 3 + 1);
    
    // 构建请求URL
    // aue=6: PCM-16K采样率-16bit-单声道（无压缩，可直接播放）
    // aue=3: MP3格式（需要解码）
    char url[2048];
    snprintf(url, sizeof(url),
             "%s?tex=%s&tok=%s&cuid=ESP32&ctp=1&lan=zh&spd=%d&pit=%d&vol=%d&per=%d&aue=6",
             BAIDU_TTS_URL, encoded_text, handle->access_token,
             handle->config.speed, handle->config.pitch,
             handle->config.volume, handle->config.voice);
    free(encoded_text);
    
    // 缓冲区：流式模式不需要256KB大缓冲
    uint8_t *audio_buffer = NULL;
    size_t buffer_capacity = 0;
    if (!enable_streaming) {
        audio_buffer = heap_caps_malloc(HTTP_RECV_BUFFER_INITIAL_SIZE,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (audio_buffer == NULL) {
            // PSRAM不可用,尝试内部RAM
            ESP_LOGW(TAG, "PSRAM allocation failed, trying internal RAM");
            audio_buffer = malloc(HTTP_RECV_BUFFER_INITIAL_SIZE);
            if (audio_buffer == NULL) {
                ESP_LOGE(TAG, "Failed to allocate audio buffer");
                return ESP_ERR_NO_MEM;
            }
        } else {
            ESP_LOGI(TAG, "Audio buffer allocated in PSRAM");
        }
        buffer_capacity = HTTP_RECV_BUFFER_INITIAL_SIZE;
    }
    
    http_audio_context_t audio_ctx = {
        .buffer = audio_buffer,
        .buffer_capacity = buffer_capacity,
        .data_len = 0,
        .is_audio = false,
        .truncated = false,
        .audio_handle = handle->audio_handle,
        .streaming = enable_streaming,
        .stereo_buf = NULL,
        .stereo_buf_size = 0,
    };
    
    // 配置HTTP客户端（增大缓冲区以支持长URL）
    uint32_t timeout2 = handle->config.timeout_ms;
    if (timeout2 > 15000) timeout2 = 15000;  // 限制超时
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = timeout2,
        .event_handler = http_event_handler,
        .user_data = &audio_ctx,
        // 临时跳过证书验证以解决SSL握手失败问题
        .crt_bundle_attach = esp_crt_bundle_attach,
        .cert_pem = NULL,
        .skip_cert_common_name_check = true,
        .use_global_ca_store = false,
        .buffer_size = 8192,     // 增大HTTP头缓冲区
        .buffer_size_tx = 2048,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        free(audio_ctx.buffer);
        free(audio_ctx.stereo_buf);
        return ESP_FAIL;
    }

    // 发送请求
    ret = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (ret != ESP_OK || status_code != 200) {
        ESP_LOGE(TAG, "TTS request failed: %s, status: %d",
                 esp_err_to_name(ret), status_code);
        free(audio_ctx.buffer);
        free(audio_ctx.stereo_buf);
        return ESP_FAIL;
    }

    if (audio_ctx.truncated) {
        ESP_LOGE(TAG, "TTS audio download truncated; abort playback");
        free(audio_ctx.buffer);
        free(audio_ctx.stereo_buf);
        return ESP_ERR_NO_MEM;
    }

    if (!audio_ctx.is_audio) {
        ESP_LOGE(TAG, "Response is not audio");
        free(audio_ctx.buffer);
        free(audio_ctx.stereo_buf);
        return ESP_FAIL;
    }

    if (audio_ctx.streaming) {
        // 流式播放模式: 数据已经在接收时播放完毕
        ESP_LOGI(TAG, "Streaming playback completed");
        // 等待 I2S DMA 缓冲区播放完毕
        vTaskDelay(pdMS_TO_TICKS(300));
        free(audio_ctx.buffer);
        free(audio_ctx.stereo_buf);
        return ESP_OK;
    }

    if (audio_ctx.data_len == 0) {
        ESP_LOGE(TAG, "No audio data received in buffer mode");
        free(audio_ctx.buffer);
        free(audio_ctx.stereo_buf);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Received %d bytes of PCM audio data", audio_ctx.data_len);
    
    // PCM格式可以直接播放
    // 百度TTS返回的PCM格式：16kHz采样率，16bit，单声道
    // 需要转换为16bit立体声（左右声道相同）供I2S播放

    // 部分服务器/代理可能返回 WAV 封装（RIFF/WAVE），这里做兼容：跳过到 data chunk
    size_t pcm_off = wav_find_pcm_data_offset(audio_ctx.buffer, audio_ctx.data_len);
    const uint8_t *pcm_ptr = audio_ctx.buffer + pcm_off;
    size_t pcm_len = (pcm_off > 0 && pcm_off < audio_ctx.data_len) ? (audio_ctx.data_len - pcm_off) : audio_ctx.data_len;
    if (pcm_off > 0) {
        ESP_LOGW(TAG, "WAV header detected, skip %u bytes, pcm_len=%u",
                 (unsigned)pcm_off, (unsigned)pcm_len);
    }
    if (pcm_len < 4) {
        ESP_LOGE(TAG, "PCM payload too small (%u bytes)", (unsigned)pcm_len);
        free(audio_ctx.buffer);
        free(audio_ctx.stereo_buf);
        return ESP_FAIL;
    }

    // I2S 播放端必须与 PCM 采样率一致，否则会造成变速/音调异常且静音清 DMA 的长度计算失真
    if (handle->audio_handle != NULL) {
        esp_err_t prep_ret = max98357a_prepare_output(handle->audio_handle, 16000);
        if (prep_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to prepare MAX98357A before playback: %s", esp_err_to_name(prep_ret));
            free(audio_ctx.buffer);
            return prep_ret;
        }
    }

    // 计算样本数（16bit = 2字节），丢弃末尾不完整字节
    size_t mono_samples = (pcm_len / 2);
    size_t stereo_size = mono_samples * 2 * sizeof(int16_t);  // 16bit立体声

    ESP_LOGI(TAG, "Converting to 16-bit stereo: %d samples -> %d bytes", mono_samples, stereo_size);

    // 分配立体声缓冲区 - 优先使用PSRAM
    int16_t *stereo_buffer = heap_caps_malloc(stereo_size,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stereo_buffer == NULL) {
        ESP_LOGW(TAG, "PSRAM allocation failed, trying internal RAM");
        stereo_buffer = malloc(stereo_size);
        if (stereo_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate stereo buffer (%d bytes)", stereo_size);
            free(audio_ctx.buffer);
            return ESP_ERR_NO_MEM;
        }
    } else {
        ESP_LOGI(TAG, "Stereo buffer allocated in PSRAM");
    }

    // ===== 保守数字增益 + DC阻断 + 限幅（避免削顶爆音）=====
    int16_t *mono_data = (int16_t *)pcm_ptr;
    int32_t peak = 0;
    int32_t last_x = 0;
    int32_t last_y = 0;
    for (size_t i = 0; i < mono_samples; i++) {
        int32_t raw = mono_data[i];
        int32_t x = (int32_t)(raw * TTS_INPUT_GAIN);
        int32_t y = x - last_x + (int32_t)(TTS_DC_BLOCK_R * (float)last_y);
        last_x = x;
        last_y = y;

        /* 软限幅：比硬截断更不容易产生刺耳爆音 */
        if (y > TTS_PCM_TARGET_PEAK) {
            int32_t over = y - TTS_PCM_TARGET_PEAK;
            y = TTS_PCM_TARGET_PEAK + over / 8;
            if (y > 22000) y = 22000;
        } else if (y < -TTS_PCM_TARGET_PEAK) {
            int32_t over = (-TTS_PCM_TARGET_PEAK) - y;
            y = -TTS_PCM_TARGET_PEAK - over / 8;
            if (y < -22000) y = -22000;
        }

        int32_t abs_val = (y >= 0) ? y : -y;
        if (abs_val > peak) peak = abs_val;

        stereo_buffer[i * 2] = (int16_t)y;
        stereo_buffer[i * 2 + 1] = (int16_t)y;
    }
    ESP_LOGI(TAG, "PCM processed with limiter, peak=%ld", peak);

    /* 淡入处理：前 50ms 线性从零增加，进一步抑制启动爆音 */
    {
        const size_t fade_samples = 800;  /* 50ms @ 16kHz */
        size_t fade_len = (mono_samples < fade_samples) ? mono_samples : fade_samples;
        for (size_t i = 0; i < fade_len; i++) {
            float factor = (float)i / (float)fade_len;
            stereo_buffer[i * 2]     = (int16_t)(stereo_buffer[i * 2] * factor);
            stereo_buffer[i * 2 + 1] = (int16_t)(stereo_buffer[i * 2 + 1] * factor);
        }
    }

    /* 淡出处理：最后 50ms 线性衰减到零，进一步抑制截止爆音 */
    {
        const size_t fade_samples = 800;  /* 50ms @ 16kHz */
        size_t fade_start = (mono_samples > fade_samples) ? (mono_samples - fade_samples) : 0;
        size_t fade_len = mono_samples - fade_start;
        for (size_t i = 0; i < fade_len; i++) {
            float factor = 1.0f - (float)i / (float)fade_len;
            size_t idx = fade_start + i;
            stereo_buffer[idx * 2]     = (int16_t)(stereo_buffer[idx * 2] * factor);
            stereo_buffer[idx * 2 + 1] = (int16_t)(stereo_buffer[idx * 2 + 1] * factor);
        }
    }

    // 释放单声道缓冲区（必须使用 audio_ctx.buffer，因为 realloc 可能已更改指针）
    free(audio_ctx.buffer);
    audio_ctx.buffer = NULL;

    // 播放音频 - 分块写入以避免超时
    ESP_LOGI(TAG, "Playing audio (%d samples, %d bytes)...", mono_samples, stereo_size);

    /* 播音前写入 80ms 预静音帧，确保 DMA 缓冲区稳定在零后再衔接音频，
     * 消除从空闲到播音切换时的 DMA 边界抖动（爆音根因之一） */
    {
        const uint32_t sr = 16000;
        const uint32_t pre_ms = 200;  // 适当拉长，进一步压制启播爆音
        size_t pre_silence_size = (size_t)(sr * 4ULL * pre_ms / 1000ULL);
        pre_silence_size &= ~((size_t)3);  // 对齐到4字节(16bit stereo)
        int16_t *pre_silence = calloc(pre_silence_size / sizeof(int16_t), sizeof(int16_t));
        if (pre_silence != NULL) {
            size_t sw = 0;
            esp_err_t pre_ret = max98357a_write(handle->audio_handle, pre_silence, pre_silence_size, &sw, 1000);
            if (pre_ret != ESP_OK) {
                ESP_LOGW(TAG, "Pre-silence write failed: %s", esp_err_to_name(pre_ret));
            }
            free(pre_silence);
        }
    }
    
    // 计算播放时间: 样本数 / 采样率 * 1000ms + 余量
    uint32_t play_time_ms = (mono_samples * 1000) / 16000;
    uint32_t timeout_ms = play_time_ms + 5000;  // +5秒余量
    
    ESP_LOGI(TAG, "Estimated play time: %lu ms, timeout: %lu ms", play_time_ms, timeout_ms);
    
    size_t total_written = 0;
    const size_t chunk_size = 4096;  // 每次写入4KB，降低单次阻塞导致的抖动
    const uint8_t *data_ptr = (const uint8_t *)stereo_buffer;
    int timeout_retry = 0;
    const int timeout_retry_max = 24;
    int no_progress_retry = 0;
    const int no_progress_retry_max = 8;
    
    while (total_written < stereo_size) {
        size_t remaining = stereo_size - total_written;
        size_t to_write = (remaining > chunk_size) ? chunk_size : remaining;
        size_t bytes_written = 0;
        
        ret = max98357a_write(handle->audio_handle, data_ptr + total_written,
                     to_write, &bytes_written, 3000);  // 每块3秒超时，降低误超时
        
        if (ret == ESP_ERR_TIMEOUT) {
            if (bytes_written > 0) {
                total_written += bytes_written;
                timeout_retry = 0;
                continue;
            }
            timeout_retry++;
            if (timeout_retry <= timeout_retry_max) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            ESP_LOGE(TAG, "I2S write timeout repeatedly at %d/%d bytes", total_written, stereo_size);
            break;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Write failed at %d/%d bytes: %s",
                     total_written, stereo_size, esp_err_to_name(ret));
            break;
        }

        if (bytes_written == 0) {
            no_progress_retry++;
            ESP_LOGW(TAG, "I2S write made no progress at %d/%d bytes, retry=%d/%d",
                     total_written, stereo_size, no_progress_retry, no_progress_retry_max);
            if (no_progress_retry >= no_progress_retry_max) {
                ret = ESP_ERR_TIMEOUT;
                ESP_LOGE(TAG, "I2S write stalled without progress");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        total_written += bytes_written;
        timeout_retry = 0;
        no_progress_retry = 0;
        
        if (bytes_written < to_write) {
            ESP_LOGW(TAG, "Partial write: %d/%d bytes at offset %d",
                     bytes_written, to_write, total_written - bytes_written);
        }
    }
    
    if (ret == ESP_OK && total_written == stereo_size) {
        ESP_LOGI(TAG, "Audio playback completed: %d/%d bytes", total_written, stereo_size);

        /* 写入静音尾帧清空 DMA 缓冲区，消除播放结束时的杂音 */
        /* 这里按“时间”清空更可靠：写入足够长的静音确保 DMA 完全排空 */
        {
            const uint32_t sr = 16000;
            const uint32_t tail_ms = 900;  // 覆盖较大的 DMA depth，减少尾部爆音/残留
            size_t silence_size = (size_t)(sr * 4ULL * tail_ms / 1000ULL);
            silence_size &= ~((size_t)3);
            int16_t *silence = calloc(silence_size / sizeof(int16_t), sizeof(int16_t));
            if (silence != NULL) {
                size_t sw = 0;
                max98357a_write(handle->audio_handle, silence, silence_size, &sw, 2000);
                free(silence);
            }
        }
        /* 等待 I2S DMA 缓冲区排空，保持放大器常开避免 SD_MODE 关断噪声 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        ESP_LOGE(TAG, "Audio playback incomplete: %d/%d bytes, error: %s",
                 total_written, stereo_size, esp_err_to_name(ret));
    }
    
    // 立即清理资源以释放内存
    free(stereo_buffer);
    
    // 显示释放后的内存状态
    ESP_LOGI(TAG, "Free heap after TTS: %lu bytes", esp_get_free_heap_size());
    
    return ret;
}

esp_err_t baidu_tts_synthesize(baidu_tts_handle_t *handle, const char *text,
                                int16_t **out_buffer, size_t *out_size) {
    if (handle == NULL || text == NULL || strlen(text) == 0 ||
        out_buffer == NULL || out_size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *out_buffer = NULL;
    *out_size = 0;
    
    // 获取TTS请求锁，避免并发HTTP连接导致SSL资源竞争
    if (s_tts_request_mutex != NULL) {
        if (xSemaphoreTake(s_tts_request_mutex, pdMS_TO_TICKS(60000)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to acquire TTS request mutex");
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // 确保有有效的token
    esp_err_t ret = baidu_tts_get_token(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get access token");
        if (s_tts_request_mutex != NULL) {
            xSemaphoreGive(s_tts_request_mutex);
        }
        return ret;
    }
    
    ESP_LOGI(TAG, "Synthesizing (async): %.30s%s", text, strlen(text) > 30 ? "..." : "");
    
    // URL编码文本
    char *encoded_text = malloc(strlen(text) * 3 + 1);
    if (encoded_text == NULL) {
        if (s_tts_request_mutex != NULL) {
            xSemaphoreGive(s_tts_request_mutex);
        }
        return ESP_ERR_NO_MEM;
    }
    url_encode(text, encoded_text, strlen(text) * 3 + 1);
    
    // 构建请求URL
    char url[2048];
    snprintf(url, sizeof(url),
             "%s?tex=%s&tok=%s&cuid=ESP32&ctp=1&lan=zh&spd=%d&pit=%d&vol=%d&per=%d&aue=6",
             BAIDU_TTS_URL, encoded_text, handle->access_token,
             handle->config.speed, handle->config.pitch,
             handle->config.volume, handle->config.voice);
    free(encoded_text);
    
    // 分配接收缓冲区
    uint8_t *audio_buffer = heap_caps_malloc(HTTP_RECV_BUFFER_INITIAL_SIZE,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (audio_buffer == NULL) {
        audio_buffer = malloc(HTTP_RECV_BUFFER_INITIAL_SIZE);
        if (audio_buffer == NULL) {
            if (s_tts_request_mutex != NULL) {
                xSemaphoreGive(s_tts_request_mutex);
            }
            return ESP_ERR_NO_MEM;
        }
    }
    
    http_audio_context_t audio_ctx = {
        .buffer = audio_buffer,
        .buffer_capacity = HTTP_RECV_BUFFER_INITIAL_SIZE,
        .data_len = 0,
        .is_audio = false,
        .audio_handle = handle->audio_handle,
        .streaming = false,
        .stereo_buf = NULL,
        .stereo_buf_size = 0,
    };
    
    uint32_t timeout3 = handle->config.timeout_ms;
    if (timeout3 > 15000) timeout3 = 15000;  // 限制超时
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = timeout3,
        .event_handler = http_event_handler,
        .user_data = &audio_ctx,
        // 临时跳过证书验证以解决SSL握手失败问题
        .crt_bundle_attach = esp_crt_bundle_attach,
        .cert_pem = NULL,
        .skip_cert_common_name_check = true,
        .use_global_ca_store = false,
        .buffer_size = 8192,     // 增大HTTP头缓冲区
        .buffer_size_tx = 2048,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        free(audio_buffer);
        if (s_tts_request_mutex != NULL) {
            xSemaphoreGive(s_tts_request_mutex);
        }
        return ESP_FAIL;
    }
    
    ret = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    
    if (ret != ESP_OK || status_code != 200 || !audio_ctx.is_audio || audio_ctx.data_len == 0) {
        ESP_LOGE(TAG, "TTS request failed: ret=%d, status=%d, is_audio=%d, data_len=%d",
                 ret, status_code, audio_ctx.is_audio, audio_ctx.data_len);
        free(audio_ctx.buffer);  // 使用 audio_ctx.buffer，因为可能已被 realloc 更新
        if (s_tts_request_mutex != NULL) {
            xSemaphoreGive(s_tts_request_mutex);
        }
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Received %d bytes audio data", audio_ctx.data_len);
    
    // 转换为16bit立体声，峰值归一化
    size_t mono_samples = audio_ctx.data_len / 2;
    size_t stereo_size = mono_samples * 2 * sizeof(int16_t);  // 16bit立体声

    // 分配立体声缓冲区
    int16_t *stereo_buffer = heap_caps_malloc(stereo_size,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stereo_buffer == NULL) {
        stereo_buffer = malloc(stereo_size);
        if (stereo_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate stereo buffer (%d bytes), free heap: %lu",
                     stereo_size, esp_get_free_heap_size());
            free(audio_ctx.buffer);
            if (s_tts_request_mutex != NULL) {
                xSemaphoreGive(s_tts_request_mutex);
            }
            return ESP_ERR_NO_MEM;
        }
    }

    // 滑动窗口AGC
    int16_t *mono_data = (int16_t *)audio_ctx.buffer;
    int32_t peak = 0;
    for (size_t i = 0; i < mono_samples; i++) {
        int32_t abs_val = mono_data[i] >= 0 ? mono_data[i] : -mono_data[i];
        if (abs_val > peak) peak = abs_val;
    }

    // 峰值归一化
    float peak_norm = (peak > 0) ? 30000.0f / (float)peak : 1.0f;
    if (peak_norm > 1.0f) {
        for (size_t i = 0; i < mono_samples; i++) {
            int32_t s = (int32_t)(mono_data[i] * peak_norm);
            if (s > 32767) s = 32767;
            if (s < -32768) s = -32768;
            mono_data[i] = (int16_t)s;
        }
    }

    const int win_size = 160;
    const float target_rms = 12000.0f;
    const float max_gain = 6.0f;
    const float silence_thr = 800.0f;
    float prev_gain = 1.0f;
    ESP_LOGI(TAG, "PCM peak: %ld, AGC processing", peak);

    for (size_t w = 0; w < mono_samples; w += win_size) {
        size_t end = w + win_size;
        if (end > mono_samples) end = mono_samples;
        size_t n = end - w;

        float sum_sq = 0;
        int32_t win_peak = 0;
        for (size_t i = w; i < end; i++) {
            float s = (float)mono_data[i];
            sum_sq += s * s;
            int32_t a = mono_data[i] >= 0 ? mono_data[i] : -mono_data[i];
            if (a > win_peak) win_peak = a;
        }
        float rms = sqrtf(sum_sq / n);

        float g = 1.0f;
        if (rms > silence_thr) {
            g = target_rms / rms;
            if (win_peak > 0) {
                float peak_limit = 32000.0f / (float)win_peak;
                if (g > peak_limit) g = peak_limit;
            }
            if (g > max_gain) g = max_gain;
            if (g < 1.0f) g = 1.0f;
        }

        float step = (g - prev_gain) / (float)n;
        float cur_gain = prev_gain;

        for (size_t i = w; i < end; i++) {
            cur_gain += step;
            int32_t amplified = (int32_t)(mono_data[i] * cur_gain);
            if (amplified > 32767) amplified = 32767;
            if (amplified < -32768) amplified = -32768;
            stereo_buffer[i * 2] = (int16_t)amplified;
            stereo_buffer[i * 2 + 1] = (int16_t)amplified;
        }
        prev_gain = g;
    }
    
    free(audio_ctx.buffer);  // 使用 audio_ctx.buffer
    
    // 释放TTS请求锁
    if (s_tts_request_mutex != NULL) {
        xSemaphoreGive(s_tts_request_mutex);
    }
    
    *out_buffer = stereo_buffer;
    *out_size = stereo_size;
    
    ESP_LOGI(TAG, "Synthesized %d bytes stereo audio", stereo_size);
    return ESP_OK;
}

esp_err_t baidu_tts_play_buffer(baidu_tts_handle_t *handle, 
                                 const int16_t *stereo_buffer, size_t size) {
    if (handle == NULL || stereo_buffer == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t prep_ret = max98357a_prepare_output(handle->audio_handle, 16000);
    if (prep_ret != ESP_OK) {
        return prep_ret;
    }
    
    size_t total_written = 0;
    const size_t chunk_size = 8192;
    const uint8_t *data_ptr = (const uint8_t *)stereo_buffer;
    esp_err_t ret = ESP_OK;
    int no_progress_retry = 0;
    const int no_progress_retry_max = 8;
    
    while (total_written < size) {
        size_t remaining = size - total_written;
        size_t to_write = (remaining > chunk_size) ? chunk_size : remaining;
        size_t bytes_written = 0;
        
        ret = max98357a_write(handle->audio_handle, data_ptr + total_written,
                             to_write, &bytes_written, 1000);
        
        if (ret != ESP_OK) {
            break;
        }
        if (bytes_written == 0) {
            no_progress_retry++;
            ESP_LOGW(TAG, "play_buffer no progress at %u/%u bytes, retry=%d/%d",
                     (unsigned)total_written, (unsigned)size, no_progress_retry, no_progress_retry_max);
            if (no_progress_retry >= no_progress_retry_max) {
                ret = ESP_ERR_TIMEOUT;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        total_written += bytes_written;
        no_progress_retry = 0;
    }
    
    return ret;
}

esp_err_t baidu_tts_stop(baidu_tts_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 停止音频播放
    if (handle->audio_handle != NULL) {
        max98357a_disable(handle->audio_handle);
    }
    
    return ESP_OK;
}
