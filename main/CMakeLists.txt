idf_component_register(SRCS "spark_chat.c" "st7735_display.c" "main.c"
                            "baidu_tts.c"
                            "max98357a.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "index.html"
                    PRIV_REQUIRES esp_timer nvs_flash esp_http_client esp_http_server esp-tls esp_driver_gpio esp_driver_i2s
                                  esp_driver_uart json mqtt esp_wifi esp_netif esp_event mbedtls
                                  esp_driver_spi)
