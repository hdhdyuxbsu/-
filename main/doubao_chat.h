#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DOUBAO_CHAT_MAX_CHAT_HISTORY
#define DOUBAO_CHAT_MAX_CHAT_HISTORY 20
#endif

#ifndef DOUBAO_CHAT_MAX_SINGLE_CONTENT
#define DOUBAO_CHAT_MAX_SINGLE_CONTENT 1024
#endif

#ifndef DOUBAO_CHAT_MAX_TOTAL_CONTENT
#define DOUBAO_CHAT_MAX_TOTAL_CONTENT 11000
#endif

#ifndef DOUBAO_CHAT_MAX_FULL_RESPONSE
#define DOUBAO_CHAT_MAX_FULL_RESPONSE 4096
#endif

#ifndef DOUBAO_CHAT_MAX_REQUEST_JSON
#define DOUBAO_CHAT_MAX_REQUEST_JSON 8192
#endif

typedef void (*doubao_chat_stream_cb_t)(const char *text, void *user_ctx);

typedef struct {
    doubao_chat_stream_cb_t on_reasoning;
    doubao_chat_stream_cb_t on_content;
    void *user_ctx;
} doubao_chat_callbacks_t;

typedef struct {
    const char *api_key;   // "Bearer ..."
    const char *url;       // e.g. https://ark.cn-beijing.volces.com/api/v3/chat/completions
    const char *user_id;   // optional
    const char *model;     // e.g. "doubao-seed-1-6-lite-251015"

    int timeout_ms;        // http timeout
    bool stream;

    bool enable_web_search;
    const char *search_mode; // e.g. "deep"

    bool skip_cert_common_name_check;
} doubao_chat_config_t;

typedef struct {
    char role[16];
    char content[DOUBAO_CHAT_MAX_SINGLE_CONTENT];
} doubao_chat_message_t;

// SSE 数据缓冲区大小
#define DOUBAO_CHAT_SSE_BUFFER_SIZE 2048

// 前向声明 HTTP 客户端句柄类型
typedef struct esp_http_client *esp_http_client_handle_t;

typedef struct {
    doubao_chat_config_t cfg;
    doubao_chat_callbacks_t cb;

    doubao_chat_message_t history[DOUBAO_CHAT_MAX_CHAT_HISTORY];
    uint8_t history_len;

    char full_response[DOUBAO_CHAT_MAX_FULL_RESPONSE];
    bool is_first_content;
    
    // SSE 数据分片缓冲区
    char sse_buffer[DOUBAO_CHAT_SSE_BUFFER_SIZE];
    size_t sse_buffer_len;
    
    // 持久 HTTP 连接
    esp_http_client_handle_t http_client;
    bool connection_active;
} doubao_chat_client_t;

void doubao_chat_init(doubao_chat_client_t *client, const doubao_chat_config_t *cfg);
void doubao_chat_set_callbacks(doubao_chat_client_t *client, const doubao_chat_callbacks_t *cb);

bool doubao_chat_add_message(doubao_chat_client_t *client, const char *role, const char *content);
void doubao_chat_trim_history(doubao_chat_client_t *client);
void doubao_chat_clear_history(doubao_chat_client_t *client);

// Blocking request; prints streaming output via callbacks (or stdout if callbacks are NULL).
bool doubao_chat_request(doubao_chat_client_t *client);

const char *doubao_chat_get_last_response(const doubao_chat_client_t *client);

/**
 * @brief 关闭持久连接
 * 在退出对话模式时调用，释放 HTTP 连接资源
 */
void doubao_chat_close_connection(doubao_chat_client_t *client);

#ifdef __cplusplus
}
#endif
