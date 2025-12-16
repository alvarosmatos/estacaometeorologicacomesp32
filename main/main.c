/*
 * ARQUIVO: main.c
 * PLACA: Franzininho WiFi Lab01 (ESP32-S2)
 * DESCRICAO: V34 - Tela Estável (Sem piscar) + Correção de Espelhamento
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_crt_bundle.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "cJSON.h"
#include "rom/ets_sys.h"
#include "esp_timer.h"
#include "esp_sntp.h"

// --- LDR ADC ---
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "FRANZININHO_V34";

// ==========================================================
//  1. CONFIGURAÇÕES
// ==========================================================
#define WIFI_SSID       "IFCE_Pesquisa01"  
#define WIFI_PASS       "sobral@pesquisa" 

#define TELEGRAM_BOT_TOKEN "8114816100:AAGu6blHj45t2XP5eRUKRww5FbP_bc4Prwk"
#define TELEGRAM_CHAT_ID   "5578468807"

// Pinos
#define BUTTON1_GPIO 7    // TELEGRAM MANUAL + MOSTRAR UMIDADE
#define BUTTON2_GPIO 6    
#define BUTTON3_GPIO 5    
#define BUTTON4_GPIO 4    
#define LED_MSG_GPIO 12   
#define LED_NIGHT_GPIO 21 
#define DHT_GPIO     15   

#define ADC_UNIT        ADC_UNIT_2
#define ADC_CHAN        ADC_CHANNEL_3 
#define ADC_ATTEN       ADC_ATTEN_DB_12

// OLED
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_NUM I2C_NUM_0
#define OLED_ADDR 0x3C

// Limites
#define TEMP_ALERTA_HIGH  30
#define HUM_ALERTA_HIGH   80
#define TEMP_ALERTA_LOW   28
#define HUM_ALERTA_LOW    75

#define LDR_ANOITECER_MV  300   
#define LDR_AMANHECER_MV  1200  
#define AMOSTRAS_MEDIA    64    

// ==========================================================
//  2. GLOBAIS
// ==========================================================
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

SemaphoreHandle_t dht_mutex;  
SemaphoreHandle_t oled_mutex;
TaskHandle_t xDisplayTaskHandle = NULL;

// Variáveis de Estado
int global_temp = 0;
int global_hum = 0;
volatile bool mostrar_umidade = false; // Controle visual

char hist_alertas[5][32]; 
int idx_alerta = 0;
char hist_leituras[5][64]; 
int idx_leitura = 0;

long last_update_id = 0; 

// ==========================================================
//  3. ADC CALIBRAÇÃO
// ==========================================================
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = { .unit_id = unit, .chan = channel, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = { .unit_id = unit, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif
    *out_handle = handle;
    return calibrated;
}

// ==========================================================
//  4. FONTE OLED 8x8 (Com Símbolo de Grau)
// ==========================================================
const uint8_t font8x8_basic[][8] = {
    {0,0,0,0,0,0,0,0}, {0,24,60,60,24,24,0,24}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, 
    {0,198,204,24,48,51,198,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0},
    {60,102,102,110,118,102,60,0}, {24,56,24,24,24,24,126,0}, {60,102,6,28,48,102,126,0}, {60,102,6,28,6,102,60,0},
    {12,28,60,108,126,12,12,0}, {126,96,124,6,6,102,60,0}, {60,96,124,102,102,102,60,0}, {126,6,12,24,48,48,48,0},
    {60,102,102,60,102,102,60,0}, {60,102,102,62,6,6,60,0}, {0,24,24,0,0,24,24,0}, {0,0,0,0,0,0,0,0},
    {60,102,12,24,24,0,24,0}, {0,0,0,0,0,0,0,0},
    {60,102,110,110,110,96,62,0}, {24,60,102,102,126,102,102,0}, {124,102,102,124,102,102,124,0}, {60,102,96,96,96,102,60,0},
    {120,108,102,102,102,108,120,0}, {126,96,96,124,96,96,126,0}, {126,96,96,124,96,96,96,0}, {60,102,96,110,102,102,60,0},
    {102,102,102,126,102,102,102,0}, {60,24,24,24,24,24,60,0}, {30,12,12,12,12,204,120,0}, {102,108,120,112,120,108,102,0},
    {96,96,96,96,96,96,126,0}, {99,119,127,107,99,99,99,0}, {102,118,126,126,110,102,102,0}, {60,102,102,102,102,102,60,0},
    {124,102,102,124,96,96,96,0}, {60,102,102,102,102,110,94,0}, {124,102,102,124,120,108,102,0}, {60,102,96,60,6,102,60,0},
    {126,24,24,24,24,24,24,0}, {102,102,102,102,102,102,60,0}, {102,102,102,102,102,60,24,0}, {99,99,99,107,127,119,99,0},
    {102,102,60,24,60,102,102,0}, {102,102,102,60,24,24,24,0}, 
    {12,18,18,12,0,0,0,0} // Indice 63: Grau
};

// ==========================================================
//  5. DRIVER OLED (CORRIGIDO A1/C8)
// ==========================================================
void i2c_init() {
    i2c_config_t conf = { .mode=I2C_MODE_MASTER, .sda_io_num=I2C_MASTER_SDA_IO, .scl_io_num=I2C_MASTER_SCL_IO, .sda_pullup_en=1, .scl_pullup_en=1, .master.clk_speed=400000 };
    i2c_param_config(I2C_MASTER_NUM, &conf); i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}
void oled_send(uint8_t byte, uint8_t type) {
    i2c_cmd_handle_t l = i2c_cmd_link_create(); i2c_master_start(l); 
    i2c_master_write_byte(l, (OLED_ADDR<<1)|0, 1);
    i2c_master_write_byte(l, type, 1); i2c_master_write_byte(l, byte, 1); 
    i2c_master_stop(l); i2c_master_cmd_begin(I2C_MASTER_NUM, l, 10); i2c_cmd_link_delete(l);
}

void oled_init() {
    oled_send(0xAE, 0); 
    oled_send(0x20, 0); oled_send(0x02, 0); 
    
    // --- CORREÇÃO DE ROTAÇÃO E ESPELHAMENTO ---
    oled_send(0xA1, 0); // A1: Desespelha horizontalmente (Arruma o "2")
    oled_send(0xC8, 0); // C8: Orientação Vertical Padrão
    // ------------------------------------------

    oled_send(0xDA, 0); oled_send(0x12, 0);
    oled_send(0x81, 0); oled_send(0xCF, 0);
    oled_send(0xD3, 0); oled_send(0x00, 0);
    oled_send(0x40, 0); 
    oled_send(0x8D, 0); oled_send(0x14, 0);
    oled_send(0xAF, 0); 
}

void oled_set_cursor(int page, int col) {
    oled_send(0xB0 + page, 0);           
    oled_send(0x00 + (col & 0x0F), 0);   
    oled_send(0x10 + ((col >> 4) & 0x0F), 0); 
}

void oled_clear() { 
    for(int i=0; i<8; i++) {
        oled_set_cursor(i, 0);
        for(int j=0; j<128; j++) oled_send(0x00, 0x40);
    }
}

// TEXTO GIGANTE
void oled_print_big(int page, int col, char *str) {
    if(xSemaphoreTake(oled_mutex, 200)){
        int start_col = col;
        char *p = str;
        oled_set_cursor(page, col);
        while(*p) {
            int idx = 0; char c = *p;
            if(c>='0' && c<='9') idx = 16+(c-'0');
            else if(c==' ') idx = 0;
            else if(c=='%') idx = 5; 
            else if(c=='*') idx = 63;
            
            const uint8_t *glyph = font8x8_basic[idx];
            for(int i=0; i<8; i++) {
                uint8_t lower = 0;
                for(int b=0; b<4; b++) { if((glyph[i] >> b) & 1) lower |= (3 << (b*2)); }
                oled_send(lower, 0x40); oled_send(lower, 0x40); 
            }
            p++;
        }
        oled_set_cursor(page + 1, start_col);
        p = str;
        while(*p) {
            int idx = 0; char c = *p;
            if(c>='0' && c<='9') idx = 16+(c-'0');
            else if(c==' ') idx = 0;
            else if(c=='%') idx = 5; 
            else if(c=='*') idx = 63; 

            const uint8_t *glyph = font8x8_basic[idx];
            for(int i=0; i<8; i++) {
                uint8_t upper = 0;
                for(int b=4; b<8; b++) { if((glyph[i] >> b) & 1) upper |= (3 << ((b-4)*2)); }
                oled_send(upper, 0x40); oled_send(upper, 0x40);
            }
            p++;
        }
        xSemaphoreGive(oled_mutex);
    }
}

// ==========================================================
//  6. AUXILIARES
// ==========================================================
void get_time_str(char *b, size_t s, bool seconds) {
    time_t n; struct tm t; time(&n); localtime_r(&n, &t);
    if(t.tm_year < 100) snprintf(b, s, "00:00:00"); 
    else { if(seconds) strftime(b, s, "%H:%M:%S", &t); else strftime(b, s, "%H:%M", &t); }
}
void save_alert(int t, int h, char *tm) { snprintf(hist_alertas[idx_alerta % 5], 32, "%s ! %dC", tm, t); idx_alerta++; }
void save_reading(int t, int h, char *tm) { snprintf(hist_leituras[idx_leitura % 5], 64, "%s %dC e %d%%", tm, t, h); idx_leitura++; }

// ==========================================================
//  7. TELEGRAM
// ==========================================================
void send_telegram(const char *msg) {
    esp_http_client_config_t cfg = { .url = "https://api.telegram.org/bot" TELEGRAM_BOT_TOKEN "/sendMessage", .transport_type = HTTP_TRANSPORT_OVER_SSL, .host = "api.telegram.org", .crt_bundle_attach = esp_crt_bundle_attach, .timeout_ms = 20000 };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    char post[1024]; snprintf(post, sizeof(post), "{\"chat_id\":\"%s\",\"text\":\"%s\"}", TELEGRAM_CHAT_ID, msg);
    esp_http_client_set_post_field(client, post, strlen(post));
    esp_http_client_perform(client); esp_http_client_cleanup(client);
}

void telegram_task(void *pv) { 
    char *msg = (char *)pv; 
    send_telegram(msg); 
    free(msg); 
    vTaskDelete(NULL); 
}

void spawn_telegram(const char *msg) { 
    char *safe_msg = strdup(msg); 
    if (safe_msg) xTaskCreate(telegram_task, "tg_send", 8192, (void *)safe_msg, 5, NULL); 
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt) { return ESP_OK; }

void check_telegram_updates() {
    char url[256];
    snprintf(url, 256, "https://api.telegram.org/bot%s/getUpdates?offset=%ld&limit=1&timeout=0", TELEGRAM_BOT_TOKEN, last_update_id + 1);

    esp_http_client_config_t cfg = {
        .url = url, .transport_type = HTTP_TRANSPORT_OVER_SSL, .host = "api.telegram.org",
        .event_handler = _http_event_handler, .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000, .buffer_size = 2048, 
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    
    if (esp_http_client_open(client, 0) == ESP_OK) {
        esp_http_client_fetch_headers(client);
        static char response_buffer[4096];
        int total_read_len = 0; int read_len;
        
        while (1) {
            read_len = esp_http_client_read(client, response_buffer + total_read_len, sizeof(response_buffer) - total_read_len - 1);
            if (read_len <= 0) break; 
            total_read_len += read_len;
        }
        response_buffer[total_read_len] = 0; 

        if (total_read_len > 0) {
            cJSON *root = cJSON_Parse(response_buffer);
            if (root) {
                cJSON *result = cJSON_GetObjectItem(root, "result");
                if (cJSON_IsArray(result) && cJSON_GetArraySize(result) > 0) {
                    cJSON *item = cJSON_GetArrayItem(result, 0);
                    cJSON *uid = cJSON_GetObjectItem(item, "update_id");
                    cJSON *msg = cJSON_GetObjectItem(item, "message");
                    cJSON *txt = cJSON_GetObjectItem(msg, "text");
                    if (uid) last_update_id = uid->valueint;
                    if (txt && strcmp(txt->valuestring, "/status") == 0) {
                        char tm[32]; get_time_str(tm, 32, true);
                        char m[256]; snprintf(m, 256, "🤖 STATUS:\n🌡 Temp: %d°C\n💧 Umid: %d%%\n🕒 %s", global_temp, global_hum, tm);
                        spawn_telegram(m);
                    }
                }
                cJSON_Delete(root);
            }
        }
    }
    esp_http_client_cleanup(client);
}

void telegram_rx_task(void *arg) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000)); 
    while(1) { check_telegram_updates(); vTaskDelay(pdMS_TO_TICKS(4000)); }
}

// ==========================================================
//  8. DRIVER DHT11
// ==========================================================
int read_dht11(int *h, int *t) {
    if(!xSemaphoreTake(dht_mutex, 1000)) return -1;
    uint8_t d[5]={0}; gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT); gpio_set_level(DHT_GPIO,0); ets_delay_us(20000);
    gpio_set_level(DHT_GPIO,1); ets_delay_us(40); gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    int c=0; while(gpio_get_level(DHT_GPIO)==1){if(c++>100){xSemaphoreGive(dht_mutex);return -2;}ets_delay_us(1);}
    c=0; while(gpio_get_level(DHT_GPIO)==0){if(c++>100){xSemaphoreGive(dht_mutex);return -2;}ets_delay_us(1);}
    c=0; while(gpio_get_level(DHT_GPIO)==1){if(c++>100){xSemaphoreGive(dht_mutex);return -2;}ets_delay_us(1);}
    for(int i=0;i<40;i++){
        c=0;while(gpio_get_level(DHT_GPIO)==0){if(c++>100){xSemaphoreGive(dht_mutex);return -2;}ets_delay_us(1);}
        uint32_t st=esp_timer_get_time();
        c=0;while(gpio_get_level(DHT_GPIO)==1){if(c++>100){xSemaphoreGive(dht_mutex);return -2;}ets_delay_us(1);}
        if((esp_timer_get_time()-st)>50) d[i/8]|=(1<<(7-(i%8)));
    }
    xSemaphoreGive(dht_mutex);
    if(d[4]==((d[0]+d[1]+d[2]+d[3])&0xFF)){ *h=d[0]; *t=d[2]; return 0; } return -3;
}

// ==========================================================
//  9. DISPLAY MINIMALISTA (ESTÁVEL)
// ==========================================================
void display_task(void *arg) {
    char big_str[10];
    const int OLED_WIDTH = 128;
    const int CHAR_W_BIG = 16;

    while(1) {
        oled_clear(); // Limpa tela

        if(mostrar_umidade) {
            // MOSTRA "XX %"
            if (global_temp == 0) sprintf(big_str, "-- %%");
            else sprintf(big_str, "%d %%", global_hum);
        } else {
            // MOSTRA "XX *"
            if (global_temp == 0) sprintf(big_str, "-- *"); 
            else sprintf(big_str, "%d *", global_temp);
        }

        // Centraliza
        int len = strlen(big_str);
        int pos_x = (OLED_WIDTH - (len * CHAR_W_BIG)) / 2;
        
        oled_print_big(3, pos_x, big_str);

        // AUMENTADO PARA 2000ms (2 SEGUNDOS) PARA NÃO PISCAR
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
    }
}

// ==========================================================
//  10. TASKS E LÓGICA
// ==========================================================

// BOTAO 1: TELEGRAM + MOSTRA UMIDADE POR 3 SEG
void btn1_task(void *arg) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, portMAX_DELAY);
    int h, t; char m[256]; char tm[32];
    
    while(1) {
        if(gpio_get_level(BUTTON1_GPIO)==0) {
            gpio_set_level(LED_MSG_GPIO, 1);
            
            // 1. Telegram
            if(read_dht11(&h,&t)==0) {
                global_temp = t; global_hum = h; get_time_str(tm, 32, true);
                snprintf(m, 256, "✅ Leitura Manual:\n🌡 Temp: %d°C\n💧 Umid: %d%%\n🕒 %s", t, h, tm);
                spawn_telegram(m);
            }

            // 2. Mostra Umidade (Notifica Display para atualizar AGORA)
            mostrar_umidade = true;
            if(xDisplayTaskHandle) xTaskNotifyGive(xDisplayTaskHandle);
            
            while(gpio_get_level(BUTTON1_GPIO)==0) { vTaskDelay(10); } 
            
            vTaskDelay(pdMS_TO_TICKS(3000)); 
            
            // Volta para temperatura
            mostrar_umidade = false;
            if(xDisplayTaskHandle) xTaskNotifyGive(xDisplayTaskHandle);

            gpio_set_level(LED_MSG_GPIO, 0); 
        } 
        vTaskDelay(100);
    }
}

void btn2_task(void *arg) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, portMAX_DELAY);
    char m[256]; char tm[32]; char tm_short[32]; int toggle = 0;
    while(1) {
        if(gpio_get_level(BUTTON2_GPIO)==0) {
            gpio_set_level(LED_MSG_GPIO, 1); get_time_str(tm, 32, true); get_time_str(tm_short, 32, false);
            if(toggle == 0) { snprintf(m, 256, "🚨 SIMULAÇÃO TEMP!\n🌡 45°C (Alta)\n🕒 %s", tm); save_alert(45, 60, tm_short); toggle = 1; } 
            else { snprintf(m, 256, "🚨 SIMULAÇÃO UMIDADE!\n💧 95%% (Alta)\n🕒 %s", tm); save_alert(25, 95, tm_short); toggle = 0; }
            spawn_telegram(m);
            while(gpio_get_level(BUTTON2_GPIO)==0) { vTaskDelay(10); } 
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_MSG_GPIO, 0); 
        } 
        vTaskDelay(100);
    }
}

void btn3_task(void *arg) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, portMAX_DELAY);
    char full[600];
    while(1) {
        if(gpio_get_level(BUTTON3_GPIO)==0) {
            gpio_set_level(LED_MSG_GPIO, 1);
            strcpy(full, "📜 HISTÓRICO DE ALERTAS:\n");
            bool tem=false; for(int i=0; i<5; i++) { if(strlen(hist_alertas[i])>0){ strcat(full, hist_alertas[i]); strcat(full, "\n"); tem=true;} }
            if(!tem) strcat(full, "(Vazio)");
            spawn_telegram(full);
            while(gpio_get_level(BUTTON3_GPIO)==0) { vTaskDelay(10); } 
            gpio_set_level(LED_MSG_GPIO, 0);
        } 
        vTaskDelay(100);
    }
}

void btn4_task(void *arg) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, portMAX_DELAY);
    char full[1024];
    while(1) {
        if(gpio_get_level(BUTTON4_GPIO)==0) {
            gpio_set_level(LED_MSG_GPIO, 1); strcpy(full, "📊 REGISTRO DE LEITURAS:\n");
            bool tem=false; for(int i=0; i<5; i++) { if(strlen(hist_leituras[i]) > 0) { strcat(full, hist_leituras[i]); strcat(full, "\n"); tem=true; } }
            if(!tem) strcat(full, "(Ainda sem leituras)");
            spawn_telegram(full);
            while(gpio_get_level(BUTTON4_GPIO)==0) { vTaskDelay(10); } 
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_MSG_GPIO, 0); 
        } 
        vTaskDelay(100);
    }
}

void dht_auto_task(void *arg) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 0, 1, portMAX_DELAY);
    bool alerta=false; int h, t; char tm[32]; char tm_short[32]; char m[256];
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    while(1) {
        if(read_dht11(&h, &t) == 0) {
            global_temp = t; global_hum = h; 
            get_time_str(tm, 32, true); get_time_str(tm_short, 32, false); 
            save_reading(t, h, tm_short);

            // ATUALIZA A TELA QUANDO LER NOVO DADO
            if(xDisplayTaskHandle) xTaskNotifyGive(xDisplayTaskHandle);

            if((t > TEMP_ALERTA_HIGH || h > HUM_ALERTA_HIGH) && !alerta) {
                snprintf(m, 256, "🚨 ALERTA AUTOMÁTICO!\n🌡 %d°C | 💧 %d%%\n🕒 %s", t, h, tm);
                spawn_telegram(m); save_alert(t, h, tm_short); alerta = true;
            } else if((t < TEMP_ALERTA_LOW && h < HUM_ALERTA_LOW) && alerta) {
                snprintf(m, 256, "✅ NORMALIZADO.\n🕒 %s", tm);
                spawn_telegram(m); alerta = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(60000)); 
    }
}

void ldr_task(void *arg) {
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT, .clk_src = ADC_DIGI_CLK_SRC_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHAN, &config));

    adc_cali_handle_t cali_handle = NULL;
    bool do_calibration = example_adc_calibration_init(ADC_UNIT, ADC_CHAN, ADC_ATTEN, &cali_handle);
    
    gpio_set_level(LED_NIGHT_GPIO, 0);
    bool escuro = false;
    char m[100];
    char tm[32];

    while(1) {
        long soma_raw = 0;
        int leituras_validas = 0;
        int raw_temp = 0;

        for(int i=0; i<AMOSTRAS_MEDIA; i++) {
            if (adc_oneshot_read(adc_handle, ADC_CHAN, &raw_temp) == ESP_OK) {
                soma_raw += raw_temp;
                leituras_validas++;
            }
            ets_delay_us(500); 
        }

        if (leituras_validas > 0) {
            int raw_medio = soma_raw / leituras_validas;
            int voltage_media = 0;

            if (do_calibration) {
                adc_cali_raw_to_voltage(cali_handle, raw_medio, &voltage_media);
            } else {
                voltage_media = (raw_medio * 2500) / 8191; 
            }

            if(voltage_media < LDR_ANOITECER_MV && !escuro) {
                gpio_set_level(LED_NIGHT_GPIO, 1);
                get_time_str(tm, 32, true);
                snprintf(m, 100, "🌑 Anoiteceu! (%d mV)\n🕒 %s", voltage_media, tm);
                spawn_telegram(m);
                escuro = true;
            } else if(voltage_media > LDR_AMANHECER_MV && escuro) {
                gpio_set_level(LED_NIGHT_GPIO, 0);
                get_time_str(tm, 32, true);
                snprintf(m, 100, "☀ Amanheceu! (%d mV)\n🕒 %s", voltage_media, tm);
                spawn_telegram(m);
                escuro = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void boot_task(void *arg) {
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL); esp_sntp_setservername(0, "pool.ntp.org"); esp_sntp_init();
    setenv("TZ", "BRT3", 1); tzset();
    time_t now = 0; struct tm timeinfo = { 0 }; int r = 0;
    while (timeinfo.tm_year < (2020 - 1900) && ++r < 20) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now); localtime_r(&now, &timeinfo);
    }
    xTaskCreate(display_task, "disp", 4096, NULL, 5, &xDisplayTaskHandle);
    spawn_telegram("SISTEMA FRANZININHO V34 INICIADO");
    vTaskDelete(NULL);
}

static void wifi_handler(void *arg, esp_event_base_t b, int32_t id, void *d) {
    if(b==WIFI_EVENT && id==WIFI_EVENT_STA_START) esp_wifi_connect();
    else if(b==WIFI_EVENT && id==WIFI_EVENT_STA_DISCONNECTED) { esp_wifi_connect(); xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT); }
    else if(b==IP_EVENT && id==IP_EVENT_STA_GOT_IP) xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

void app_main(void) {
    nvs_flash_init(); dht_mutex = xSemaphoreCreateMutex(); oled_mutex = xSemaphoreCreateMutex();
    gpio_set_direction(LED_MSG_GPIO, GPIO_MODE_OUTPUT); gpio_set_direction(LED_NIGHT_GPIO, GPIO_MODE_OUTPUT);
    
    // Configura Botões
    gpio_set_direction(BUTTON1_GPIO, GPIO_MODE_INPUT); gpio_set_pull_mode(BUTTON1_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(BUTTON2_GPIO, GPIO_MODE_INPUT); gpio_set_pull_mode(BUTTON2_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(BUTTON3_GPIO, GPIO_MODE_INPUT); gpio_set_pull_mode(BUTTON3_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(BUTTON4_GPIO, GPIO_MODE_INPUT); gpio_set_pull_mode(BUTTON4_GPIO, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT_OUTPUT); gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);
    for(int i=0;i<5;i++) { memset(hist_alertas[i],0,32); memset(hist_leituras[i],0,64); }
    
    i2c_init(); oled_init(); oled_clear(); 
    
    wifi_event_group = xEventGroupCreate(); esp_netif_init(); esp_event_loop_create_default(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_handler, NULL);
    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_set_config(WIFI_IF_STA, &wc); esp_wifi_start();

    xTaskCreate(boot_task, "boot", 8192, NULL, 5, NULL);
    xTaskCreate(dht_auto_task, "dht", 4096, NULL, 5, NULL);
    xTaskCreate(ldr_task, "ldr", 4096, NULL, 6, NULL); // Prioridade aumentada para 6
    xTaskCreate(btn1_task, "b1", 4096, NULL, 5, NULL);
    xTaskCreate(btn2_task, "b2", 4096, NULL, 5, NULL);
    xTaskCreate(btn3_task, "b3", 8192, NULL, 5, NULL);
    xTaskCreate(btn4_task, "b4", 8192, NULL, 5, NULL);
    
    xTaskCreate(telegram_rx_task, "tg_rx", 8192, NULL, 5, NULL);
}   


 