#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/gptimer.h"

// Configurações de Rede
const char* ssid = "DobotCR10A-4421-1123";
const char* password = "1234567890";
const char* destination_ip = "192.168.201.101";
const uint16_t udp_port = 8080;

WiFiUDP udp;

// Amostragem
#define ADC_PIN 34
#define SAMPLE_RATE_HZ 1000
#define ADC_SAMPLES_COUNT 20

// Filtro e Conversão
float peso_filtrado = 0.0;
const float alpha = 0.15;
const float R1 = 220000.0;
const float R2 = 98600.0;

// Estrutura de Dados Binária (12 bytes total) - Diminuir latencia
struct __attribute__((packed)) Payload 
{
    uint32_t seq;    // 4 bytes: Sequencial para checar perda de pacotes
    float peso;      // 4 bytes: Peso em kg (float 32-bit)
    float voltagem;  // 4 bytes: Voltagem do ADC (opcional para debug)
};

Payload packet;
uint32_t seq_counter = 0;
uint32_t adc_sum = 0;
uint16_t adc_counter = 0;

// Gptimer
gptimer_handle_t timer;
SemaphoreHandle_t semSample;

bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg) {
    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(semSample, &high_task_awoken);
    return high_task_awoken == pdTRUE;
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi Conectado!");

    semSample = xSemaphoreCreateBinary();

    // Configuração do Timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    gptimer_new_timer(&timer_config, &timer);
    gptimer_event_callbacks_t cbs = { .on_alarm = timer_callback };
    gptimer_register_event_callbacks(timer, &cbs, NULL);
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000, // 1ms
        .reload_count = 0,
        .flags = { .auto_reload_on_alarm = true }
    };
    gptimer_set_alarm_action(timer, &alarm_cfg);
    gptimer_enable(timer);
    gptimer_start(timer);
}

void loop() 
{
    if (xSemaphoreTake(semSample, portMAX_DELAY)) 
    {
        adc_sum += analogRead(ADC_PIN);
        adc_counter++;

        if (adc_counter >= ADC_SAMPLES_COUNT) {
            float adc_avg = (float)adc_sum / ADC_SAMPLES_COUNT;
            adc_sum = 0;
            adc_counter = 0;

            // Conversão para Peso
            float v_adc = ((adc_avg * 3.3) / 4095.0) * 1.264;
            float v_sensor = v_adc * ((R1 + R2) / R2);
            float peso_inst = (v_sensor - 0.0017) / 0.4490;

            // Montagem do Payload Binário
            packet.seq = seq_counter++;
            packet.peso = peso_inst;
            packet.voltagem = v_adc;

            // Envio via UDP
            udp.beginPacket(destination_ip, udp_port);
            udp.write((uint8_t*)&packet, sizeof(Payload));
            udp.endPacket();
        }
    }
}