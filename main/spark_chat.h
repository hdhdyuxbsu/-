#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SPARK_CHAT_MAX_CHAT_HISTORY
#define SPARK_CHAT_MAX_CHAT_HISTORY 8
#endif

#ifndef SPARK_CHAT_MAX_SINGLE_CONTENT
#define SPARK_CHAT_MAX_SINGLE_CONTENT 2048
#endif

#ifndef SPARK_CHAT_MAX_TOTAL_CONTENT
#define SPARK_CHAT_MAX_TOTAL_CONTENT 11000
#endif

#ifndef SPARK_CHAT_MAX_FULL_RESPONSE
#define SPARK_CHAT_MAX_FULL_RESPONSE 4096
#endif

#ifndef SPARK_CHAT_MAX_REQUEST_JSON
#define SPARK_CHAT_MAX_REQUEST_JSON 12288
#endif

typedef void (*spark_chat_stream_cb_t)(const char *text, void *user_ctx);

typedef struct {
    spark_chat_stream_cb_t on_reasoning;
    spark_chat_stream_cb_t on_content;
    void *user_ctx;
} spark_chat_callbacks_t;

typedef struct {
    const char *api_key;   // "Bearer ..."
    const char *url;       // e.g. https://spark-api-open.xf-yun.com/v2/chat/completions
    const char *user_id;   // optional
    const char *model;     // e.g. "x1"

    int timeout_ms;        // http timeout
    bool stream;

    bool enable_web_search;
    const char *search_mode; // e.g. "deep"

    bool skip_cert_common_name_check;
} spark_chat_config_t;

typedef struct {
    char role[16];
    char content[SPARK_CHAT_MAX_SINGLE_CONTENT];
} spark_chat_message_t;

// SSE 数据缓冲区大小（非流式响应可能很大，需要足够空间）
#define SPARK_CHAT_SSE_BUFFER_SIZE 8192

// 前向声明 HTTP 客户端句柄类型
typedef struct esp_http_client *esp_http_client_handle_t;

typedef struct {
    spark_chat_config_t cfg;
    spark_chat_callbacks_t cb;

    spark_chat_message_t history[SPARK_CHAT_MAX_CHAT_HISTORY];
    uint8_t history_len;

    char full_response[SPARK_CHAT_MAX_FULL_RESPONSE];
    bool is_first_content;
    
    // SSE 数据分片缓冲区
    char sse_buffer[SPARK_CHAT_SSE_BUFFER_SIZE];
    size_t sse_buffer_len;
    
    // 持久 HTTP 连接
    esp_http_client_handle_t http_client;
    bool connection_active;

    // 最近一次请求状态（用于上层判定是否应该重试）
    int last_http_status;
    bool last_error_is_permanent;
    bool last_error_is_overdue_balance;
} spark_chat_client_t;

void spark_chat_init(spark_chat_client_t *client, const spark_chat_config_t *cfg);
void spark_chat_set_callbacks(spark_chat_client_t *client, const spark_chat_callbacks_t *cb);

bool spark_chat_add_message(spark_chat_client_t *client, const char *role, const char *content);
void spark_chat_trim_history(spark_chat_client_t *client);
void spark_chat_clear_history(spark_chat_client_t *client);

// Blocking request; prints streaming output via callbacks (or stdout if callbacks are NULL).
bool spark_chat_request(spark_chat_client_t *client);

const char *spark_chat_get_last_response(const spark_chat_client_t *client);

int spark_chat_get_last_http_status(const spark_chat_client_t *client);
bool spark_chat_last_error_is_permanent(const spark_chat_client_t *client);
bool spark_chat_last_error_is_overdue_balance(const spark_chat_client_t *client);

/**
 * @brief 关闭持久连接
 * 在退出对话模式时调用，释放 HTTP 连接资源
 */
void spark_chat_close_connection(spark_chat_client_t *client);

#ifdef __cplusplus
}
#endif
