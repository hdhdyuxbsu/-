#include "doubao_chat.h"

#include "esp_crt_bundle.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

#define TAG "DoubaoChat"

static void get_url_host(const char *url, char *host_buf, size_t host_buf_len) {
    if (host_buf == NULL || host_buf_len == 0) {
        return;
    }
    host_buf[0] = '\0';
    if (url == NULL || url[0] == '\0') {
        return;
    }

    const char *start = strstr(url, "://");
    start = (start != NULL) ? (start + 3) : url;
    const char *end = strchr(start, '/');
    size_t len = (end != NULL) ? (size_t)(end - start) : strlen(start);
    if (len >= host_buf_len) {
        len = host_buf_len - 1;
    }
    memcpy(host_buf, start, len);
    host_buf[len] = '\0';
}

static void safe_strcpy(char *dst, size_t dst_len, const char *src) {
    if (dst == NULL || dst_len == 0) {
        return;
    }
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_len - 1);
    dst[dst_len - 1] = '\0';
}

static uint32_t history_total_len(const doubao_chat_client_t *client) {
    uint32_t total = 0;
    for (uint8_t i = 0; i < client->history_len; i++) {
        total += (uint32_t)strlen(client->history[i].content);
    }
    return total;
}

void doubao_chat_init(doubao_chat_client_t *client, const doubao_chat_config_t *cfg) {
    if (client == NULL) {
        return;
    }

    memset(client, 0, sizeof(*client));

    if (cfg != NULL) {
        client->cfg = *cfg;
    }

    if (client->cfg.model == NULL) {
        client->cfg.model = "ep-m-20251108165819-4j5wk";  // 默认使用豆包模型
    }
    if (client->cfg.search_mode == NULL) {
        client->cfg.search_mode = "deep";
    }
    if (client->cfg.timeout_ms <= 0) {
        client->cfg.timeout_ms = 30000;
    }

    // 不再强制覆盖 stream 设置，保留用户配置
    // client->cfg.stream = true;

    client->is_first_content = true;
    client->http_client = NULL;
    client->connection_active = false;

    char host[96];
    get_url_host(client->cfg.url, host, sizeof(host));
    ESP_LOGI(TAG, "豆包AI配置: host=%s, model=%s",
        host[0] != '\0' ? host : "(none)",
        client->cfg.model != NULL ? client->cfg.model : "(none)");
}

void doubao_chat_set_callbacks(doubao_chat_client_t *client, const doubao_chat_callbacks_t *cb) {
    if (client == NULL) {
        return;
    }
    if (cb == NULL) {
        memset(&client->cb, 0, sizeof(client->cb));
        return;
    }
    client->cb = *cb;
}

bool doubao_chat_add_message(doubao_chat_client_t *client, const char *role, const char *content) {
    if (client == NULL || role == NULL || content == NULL || content[0] == '\0') {
        return false;
    }

    if (client->history_len >= DOUBAO_CHAT_MAX_CHAT_HISTORY) {
        for (uint8_t i = 0; i < DOUBAO_CHAT_MAX_CHAT_HISTORY - 1; i++) {
            memcpy(&client->history[i], &client->history[i + 1], sizeof(doubao_chat_message_t));
        }
        client->history_len--;
        memset(&client->history[client->history_len], 0, sizeof(doubao_chat_message_t));
    }

    safe_strcpy(client->history[client->history_len].role, sizeof(client->history[client->history_len].role), role);
    safe_strcpy(client->history[client->history_len].content, sizeof(client->history[client->history_len].content), content);
    client->history_len++;
    return true;
}

void doubao_chat_clear_history(doubao_chat_client_t *client) {
    if (client == NULL) {
        return;
    }
    for (uint8_t i = 0; i < DOUBAO_CHAT_MAX_CHAT_HISTORY; i++) {
        memset(&client->history[i], 0, sizeof(doubao_chat_message_t));
    }
    client->history_len = 0;
}

void doubao_chat_trim_history(doubao_chat_client_t *client) {
    if (client == NULL) {
        return;
    }

    while (history_total_len(client) > DOUBAO_CHAT_MAX_TOTAL_CONTENT) {
        if (client->history_len <= 1) {
            break;
        }
        for (uint8_t i = 0; i < client->history_len - 1; i++) {
            memcpy(&client->history[i], &client->history[i + 1], sizeof(doubao_chat_message_t));
        }
        client->history_len--;
        memset(&client->history[client->history_len], 0, sizeof(doubao_chat_message_t));
    }
}

static bool build_request_json(const doubao_chat_client_t *client, char *json_buf, size_t buf_len) {
    if (client == NULL || json_buf == NULL || buf_len == 0) {
        return false;
    }
    memset(json_buf, 0, buf_len);

    // 豆包 Ark Chat Completions 格式: {model, stream, messages:[{role, content}]}
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return false;
    }

    cJSON_AddStringToObject(root, "model", client->cfg.model);
    cJSON_AddBoolToObject(root, "stream", client->cfg.stream);

    // Chat Completions messages 结构
    if (client->cfg.user_id != NULL) {
        cJSON_AddStringToObject(root, "user", client->cfg.user_id);
    }
    cJSON *messages = cJSON_CreateArray();
    for (uint8_t i = 0; i < client->history_len; i++) {
        cJSON *msg = cJSON_CreateObject();
        cJSON_AddStringToObject(msg, "role", client->history[i].role);
        cJSON_AddStringToObject(msg, "content", client->history[i].content);
        cJSON_AddItemToArray(messages, msg);
    }
    cJSON_AddItemToObject(root, "messages", messages);

    // 豆包 Ark Chat Completions: web_search 作为顶层参数
    if (client->cfg.enable_web_search) {
        cJSON *web_search = cJSON_CreateObject();
        cJSON_AddBoolToObject(web_search, "enable", true);
        cJSON_AddStringToObject(web_search, "search_mode",
            client->cfg.search_mode != NULL ? client->cfg.search_mode : "auto");
        cJSON_AddItemToObject(root, "web_search", web_search);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL || strlen(json_str) >= buf_len) {
        cJSON_Delete(root);
        if (json_str != NULL) {
            free(json_str);
        }
        return false;
    }

    strcpy(json_buf, json_str);
    free(json_str);
    cJSON_Delete(root);
    return true;
}

static void stream_content(doubao_chat_client_t *client, const char *text) {
    if (text == NULL || text[0] == '\0') {
        return;
    }

    if (client->is_first_content) {
        client->is_first_content = false;
    }

    if (client->cb.on_content != NULL) {
        client->cb.on_content(text, client->cb.user_ctx);
    } else {
        printf("%s", text);
        fflush(stdout);
    }

    strncat(client->full_response, text, DOUBAO_CHAT_MAX_FULL_RESPONSE - strlen(client->full_response) - 1);
}

/**
 * @brief 处理单个 SSE 数据行或完整 JSON 响应
 */
static void process_sse_line(doubao_chat_client_t *client, const char *line) {
    if (line == NULL || line[0] == '\0') {
        return;
    }
    
    // 跳过空行
    if (line[0] == '\n' || line[0] == '\r') {
        return;
    }
    
    // 检查 [DONE] 标记
    if (strstr(line, "[DONE]") != NULL) {
        return;
    }
    
    // 解析 "data: {json}" 格式（流式）或直接 JSON（非流式）
    const char *json_start = line;
    if (strncmp(line, "data:", 5) == 0) {
        json_start = line + 5;
        while (*json_start == ' ') {
            json_start++;
        }
    }
    
    if (json_start[0] == '\0') {
        return;
    }
    
    cJSON *root = cJSON_Parse(json_start);
    if (root == NULL) {
        return;
    }

    // OpenAI 兼容错误格式：{"error": {"message": "..."}}
    cJSON *error = cJSON_GetObjectItem(root, "error");
    if (error != NULL && cJSON_IsObject(error)) {
        cJSON *message = cJSON_GetObjectItem(error, "message");
        const char *msg = (message && cJSON_IsString(message)) ? message->valuestring : "未知错误";
        ESP_LOGE(TAG, "API错误: %s", msg);
        cJSON_Delete(root);
        return;
    }

    // 检查是否是错误响应
    cJSON *code = cJSON_GetObjectItem(root, "code");
    if (code != NULL && cJSON_IsNumber(code) && code->valueint != 0) {
        cJSON *message = cJSON_GetObjectItem(root, "message");
        const char *msg = (message && cJSON_IsString(message)) ? message->valuestring : "未知错误";
        ESP_LOGE(TAG, "API错误 code=%d: %s", code->valueint, msg);
        cJSON_Delete(root);
        return;
    }
    
    cJSON *choices = cJSON_GetObjectItem(root, "choices");
    if (choices == NULL || !cJSON_IsArray(choices) || cJSON_GetArraySize(choices) == 0) {
        cJSON_Delete(root);
        return;
    }
    
    cJSON *first_choice = cJSON_GetArrayItem(choices, 0);
    
    // 先尝试流式格式 (delta)
    cJSON *delta = cJSON_GetObjectItem(first_choice, "delta");
    if (delta != NULL) {
        cJSON *content = cJSON_GetObjectItem(delta, "content");
        if (content != NULL && cJSON_IsString(content) && content->valuestring != NULL) {
            stream_content(client, content->valuestring);
        }
    } else {
        // 非流式格式 (message)
        cJSON *message = cJSON_GetObjectItem(first_choice, "message");
        if (message != NULL) {
            cJSON *content = cJSON_GetObjectItem(message, "content");
            if (content != NULL && cJSON_IsString(content) && content->valuestring != NULL) {
                stream_content(client, content->valuestring);
            }
        }
    }
    
    cJSON_Delete(root);
}

static esp_err_t doubao_http_event_handler(esp_http_client_event_t *evt) {
    if (evt == NULL) {
        return ESP_OK;
    }

    doubao_chat_client_t *client = (doubao_chat_client_t *)evt->user_data;

    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA: {
            if (client == NULL || evt->data == NULL || evt->data_len <= 0) {
                break;
            }
            
            // 将新数据追加到 SSE 缓冲区
            size_t space_left = DOUBAO_CHAT_SSE_BUFFER_SIZE - client->sse_buffer_len - 1;
            size_t copy_len = (evt->data_len < space_left) ? evt->data_len : space_left;
            if (copy_len > 0) {
                memcpy(client->sse_buffer + client->sse_buffer_len, evt->data, copy_len);
                client->sse_buffer_len += copy_len;
                client->sse_buffer[client->sse_buffer_len] = '\0';
            }
            
            // 按行处理 SSE 数据
            char *line_start = client->sse_buffer;
            char *newline;
            
            while ((newline = strstr(line_start, "\n")) != NULL) {
                *newline = '\0';
                
                if (strlen(line_start) > 0) {
                    process_sse_line(client, line_start);
                }
                
                line_start = newline + 1;
                while (*line_start == '\n' || *line_start == '\r') {
                    line_start++;
                }
            }
            
            // 将未处理的数据移到缓冲区开头
            if (line_start != client->sse_buffer) {
                size_t remaining = strlen(line_start);
                if (remaining > 0) {
                    memmove(client->sse_buffer, line_start, remaining + 1);
                    client->sse_buffer_len = remaining;
                } else {
                    client->sse_buffer[0] = '\0';
                    client->sse_buffer_len = 0;
                }
            }
            break;
        }

        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP请求错误");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            if (client != NULL) {
                client->sse_buffer[0] = '\0';
                client->sse_buffer_len = 0;
            }
            break;

        case HTTP_EVENT_DISCONNECTED:
            if (client != NULL && client->sse_buffer_len > 0) {
                process_sse_line(client, client->sse_buffer);
                client->sse_buffer[0] = '\0';
                client->sse_buffer_len = 0;
            }
            break;

        default:
            break;
    }

    return ESP_OK;
}

bool doubao_chat_request(doubao_chat_client_t *client) {
    if (client == NULL || client->cfg.url == NULL || client->cfg.api_key == NULL) {
        return false;
    }

    memset(client->full_response, 0, sizeof(client->full_response));
    client->is_first_content = true;

    char request_json[DOUBAO_CHAT_MAX_REQUEST_JSON] = {0};
    if (!build_request_json(client, request_json, sizeof(request_json))) {
        ESP_LOGE(TAG, "构建请求JSON失败");
        return false;
    }

    char host[96];
    get_url_host(client->cfg.url, host, sizeof(host));
    ESP_LOGI(TAG, "请求豆包AI: host=%s, model=%s, payload=%d bytes",
        host[0] != '\0' ? host : "(none)",
        client->cfg.model != NULL ? client->cfg.model : "(none)",
        (int)strlen(request_json));
    ESP_LOGI(TAG, "Request JSON: %.500s", request_json);  // 打印前500字符

    // 如果没有活跃连接，创建新的 HTTP 客户端
    if (client->http_client == NULL) {
        uint32_t timeout = client->cfg.timeout_ms;
        if (timeout > 15000) timeout = 15000;
        
        esp_http_client_config_t config = {
            .url = client->cfg.url,
            .event_handler = doubao_http_event_handler,
            .user_data = client,
            .method = HTTP_METHOD_POST,
            .timeout_ms = timeout,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .cert_pem = NULL,
            .skip_cert_common_name_check = false,
            .use_global_ca_store = false,
            .buffer_size = 1024,
            .buffer_size_tx = 1024,
            .is_async = false,
            .keep_alive_enable = false,
            .disable_auto_redirect = false,
        };

        client->http_client = esp_http_client_init(&config);
        if (client->http_client == NULL) {
            ESP_LOGE(TAG, "初始化HTTP客户端失败");
            return false;
        }
    }

    // 构建认证头
    char auth_header[512];
    if (strncmp(client->cfg.api_key, "Bearer ", 7) == 0) {
        snprintf(auth_header, sizeof(auth_header), "%s", client->cfg.api_key);
    } else {
        snprintf(auth_header, sizeof(auth_header), "Bearer %s", client->cfg.api_key);
    }
    
    esp_http_client_set_header(client->http_client, "Authorization", auth_header);
    esp_http_client_set_header(client->http_client, "Content-Type", "application/json");
    esp_http_client_set_header(client->http_client, "Accept", "text/event-stream");
    esp_http_client_set_header(client->http_client, "Connection", "close");
    esp_http_client_set_post_field(client->http_client, request_json, (int)strlen(request_json));

    esp_err_t err = esp_http_client_perform(client->http_client);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP请求失败: %s (0x%x)", esp_err_to_name(err), err);
        esp_http_client_cleanup(client->http_client);
        client->http_client = NULL;
        client->connection_active = false;
        return false;
    }

    int http_code = esp_http_client_get_status_code(client->http_client);
    
    if (http_code != 200) {
        ESP_LOGE(TAG, "HTTP响应错误，状态码: %d", http_code);
        if (http_code == 401) {
            ESP_LOGE(TAG, "鉴权失败：请检查API Key是否正确");
        } else if (http_code == 403) {
            ESP_LOGE(TAG, "权限/计费失败：常见原因是账号欠费、模型未开通或Key无权限");
        }
        esp_http_client_cleanup(client->http_client);
        client->http_client = NULL;
        client->connection_active = false;
        return false;
    }

    client->connection_active = true;
    printf("\n");
    return true;
}

void doubao_chat_close_connection(doubao_chat_client_t *client) {
    if (client == NULL) {
        return;
    }
    
    if (client->http_client != NULL) {
        esp_http_client_cleanup(client->http_client);
        client->http_client = NULL;
    }
    client->connection_active = false;
}

const char *doubao_chat_get_last_response(const doubao_chat_client_t *client) {
    if (client == NULL) {
        return "";
    }
    return client->full_response;
}
