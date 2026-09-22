#include <Arduino.h>

// Pinos do ADC1 (Entradas apenas, ideais para leitura analógica)
#define PINO_OUT_PAN  34
#define PINO_OUT_TILT 35

// Quantidade de leituras para média (suavização de ruído)
const int NUM_AMOSTRAS = 16;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configuração das entradas analógicas
  pinMode(PINO_OUT_PAN, INPUT);
  pinMode(PINO_OUT_TILT, INPUT);

  // Define escala de leitura do ADC para até ~3.3V (PADRÃO ESP32)
  analogSetAttenuation(ADC_11db);

  Serial.println("=============================================");
  Serial.println("    TESTE DE LEITURA ANALÓGICA AS5600 (OUT)  ");
  Serial.println("=============================================");
}

// Realiza a leitura e calcula a média das amostras
uint16_t lerAdcSuavizado(uint8_t pino) {
  uint32_t soma = 0;
  for (int i = 0; i < NUM_AMOSTRAS; i++) {
    soma += analogRead(pino);
    delayMicroseconds(100);
  }
  return (uint16_t)(soma / NUM_AMOSTRAS);
}

// Converte a leitura de 12 bits (0 a 4095) para Graus (0.0° a 360.0°)
float converterParaGraus(uint16_t valorAdc) {
  return (valorAdc / 4095.0f) * 360.0f;
}

void loop() {
  // Leitura dos encoders
  uint16_t adcPan  = lerAdcSuavizado(PINO_OUT_PAN);
  uint16_t adcTilt = lerAdcSuavizado(PINO_OUT_TILT);

  // Conversão para Graus
  float grausPan  = converterParaGraus(adcPan);
  float grausTilt = converterParaGraus(adcTilt);

  // Conversão aproximada para Volts
  float voltsPan  = (adcPan * 3.3f) / 4095.0f;
  float voltsTilt = (adcTilt * 3.3f) / 4095.0f;

  // Impressão organizada no Monitor Serial
  Serial.printf("PAN  -> ADC: %4d | V: %.2fV | Angulo: %5.1f°\n", adcPan, voltsPan, grausPan);
  Serial.printf("TILT -> ADC: %4d | V: %.2fV | Angulo: %5.1f°\n", adcTilt, voltsTilt, grausTilt);
  Serial.println("--------------------------------------------------");

  delay(200); // Atualiza 5 vezes por segundo
}