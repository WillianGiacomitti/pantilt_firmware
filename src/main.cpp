#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "Eixo.h"
#include "WebController.h"

// ============================================================================
// CONFIGURAÇÕES DE REDE E HARDWARE

const char* WIFI_SSID = "PTU_Controller";
const char* WIFI_PASS = "12345678";
// const char* WIFI_SSID = "lara";
// const char* WIFI_PASS = "lara271828";


#define ENDERECO_PCA9548A 0x70
#define R_SENSE 0.11f
#define RXD2 16
#define TXD2 17
#define EN_PIN 27 // PINO DE ENABLE DOS MOTORES (Crucial para eles andarem)

// INSTANCIAMENTO DE HARDWARE E SERVIDOR WEB
TMC2209Stepper driverPan(&Serial2, R_SENSE, 0b00);
TMC2209Stepper driverTilt(&Serial2, R_SENSE, 0b01);

AccelStepper motorPan(AccelStepper::DRIVER, 19, 18);
AccelStepper motorTilt(AccelStepper::DRIVER, 25, 26); // Corrigido para os pinos do TILT da sua tabela (STEP 25, DIR 26)

AS5600 encoderShared;

Eixo eixoPan(&driverPan, &motorPan, &encoderShared, 0, 17.0, 127.0, 2, "PAN", 1200);
Eixo eixoTilt(&driverTilt, &motorTilt, &encoderShared, 1, 21.0, 64.0, 2, "TILT", 800);

WebController webController(&eixoPan, &eixoTilt);

TaskHandle_t TaskMotoresHandle;

void TaskMotores(void * pvParameters) {
  for(;;) {
    eixoPan.run();
    eixoTilt.run();
    taskYIELD();
  }
}

// SETUP & LOOP PRINCIPAL (Core 0)
void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. CONFIGURAÇÃO DE HABILITAÇÃO DOS MOTORES (FALTAVA ISSO!)
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Coloca em LOW para habilitar a energia para os motores
  Serial.println("[SETUP] Pino EN (27) configurado para LOW (Motores energizados).");

  // 2. Inicializa I2C e UART
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(1000);
  
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  while(Serial2.available() > 0) { Serial2.read(); }
  delay(100);

  // 3. Inicializa os eixos
  eixoPan.begin();
  eixoTilt.begin();

  // 4. Configura Wi-Fi em Modo Access Point (AP)
  WiFi.softAP(WIFI_SSID, WIFI_PASS);

  IPAddress IP = WiFi.softAPIP();
  
  Serial.println("\n-------------------------------------------");
  Serial.print("Ponto de Acesso Wi-Fi Criado: ");
  Serial.println(WIFI_SSID);
  Serial.print("Endereço IP para Acesso Web: http://");
  Serial.println(IP);
  Serial.println("-------------------------------------------\n");

  // 5. Configura as rotas do servidor HTTP
  webController.begin();

  // 6. Cria a Task de prioridade máxima no Core 1 para os motores
  xTaskCreatePinnedToCore(
    TaskMotores,
    "TaskMotores",
    4096,
    NULL,
    1,
    &TaskMotoresHandle,
    1
  );
}

void loop() {
  webController.handleClient();
}