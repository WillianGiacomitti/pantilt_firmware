#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

// Endereço I2C padrão do Multiplexador PCA9548A (com A0, A1, A2 no GND)
#define ENDERECO_PCA9548A 0x70

// Pinos I2C no ESP32
#define I2C_SDA 21 // verde
#define I2C_SCL 22 //azul

// Instância única do objeto AS5600 (será reutilizada trocando o canal)
AS5600 encoder;

// Função auxiliar para mudar o canal ativo no Multiplexador PCA9548A (Canais de 0 a 7)
bool selecionarCanalI2C(uint8_t canal) {
  if (canal > 7) return false;

  Wire.beginTransmission(ENDERECO_PCA9548A);
  Wire.write(1 << canal);
  uint8_t resultado = Wire.endTransmission();

  // Pausa essencial: dá 500 microssegundos para os transistores do PCA9548A 
  // comutarem fisicamente e estabilizarem a nova linha I2C.
  delayMicroseconds(100); 

  return (resultado == 0); // Retorna true se a troca foi aceita pelo multiplexador
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n--- TESTE DE DUAL ENCODER AS5600 COM MULTIPLEXADOR PCA9548A ---");

  // Inicializa barramento I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100 kHz para maior imunidade a ruídos
  Wire.setTimeOut(1000);  // Evita o Error 263 (Timeout)

  // --------------------------------------------------------------------------
  // TESTE E CONFIGURAÇÃO DO ENCODER 1 (CANAL 0)
  // --------------------------------------------------------------------------
  selecionarCanalI2C(0);
  encoder.begin();
  
  Serial.print("Encoder 1 (Canal 0): ");
  if (encoder.isConnected()) {
    Serial.print("Conectado! | Ima: ");
    Serial.println(encoder.detectMagnet() ? "OK" : "NAO DETECTADO!");
  } else {
    Serial.println("ERRO DE CONEXAO!");
  }

  // --------------------------------------------------------------------------
  // TESTE E CONFIGURAÇÃO DO ENCODER 2 (CANAL 1)
  // --------------------------------------------------------------------------
  selecionarCanalI2C(1);
  encoder.begin();

  Serial.print("Encoder 2 (Canal 1): ");
  if (encoder.isConnected()) {
    Serial.print("Conectado! | Ima: ");
    Serial.println(encoder.detectMagnet() ? "OK" : "NAO DETECTADO!");
  } else {
    Serial.println("ERRO DE CONEXAO!");
  }

  Serial.println("-------------------------------------------------------------------\n");
}

void loop() {
  // === LEITURA DO ENCODER 1 ===
  selecionarCanalI2C(0);
  uint16_t bruto1 = encoder.readAngle();
  float angulo1 = bruto1 * (360.0 / 4096.0);

  // === LEITURA DO ENCODER 2 ===
  selecionarCanalI2C(1);
  uint16_t bruto2 = encoder.readAngle();
  float angulo2 = bruto2 * (360.0 / 4096.0);

  // Exibe as leituras lado a lado no Monitor Serial
  Serial.print("ENC 1 -> Bruto: ");
  Serial.print(bruto1);
  Serial.print(" (");
  Serial.print(angulo1, 1);
  Serial.print("°) \t | \t ENC 2 -> Bruto: ");
  Serial.print(bruto2);
  Serial.print(" (");
  Serial.print(angulo2, 1);
  Serial.println("°)");

  delay(100);
}