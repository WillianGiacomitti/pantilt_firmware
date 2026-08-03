#include <Arduino.h>
#include <Wire.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <AS5600.h>
#include "esp_task_wdt.h"

#include "Eixo.h"
#include "SerialProtocol.h"

#define ENDERECO_PCA9548A 0x70
#define R_SENSE 0.11f
#define RXD2 16
#define TXD2 17
#define EN_PIN 27
#define LED_STATUS 2

#define SERIAL_BAUD 921600
#define TELEMETRY_HZ 20
#define ENCODER_HZ 50
#define FAILSAFE_TIMEOUT_MS 500   // sem comando/heartbeat -> para os motores
#define I2C_TIMEOUT_MS 30         // limite de bloqueio do barramento I2C
#define WDT_TIMEOUT_S 3           // se uma task não "der sinal de vida" nesse tempo, reseta


// ---------------- Hardware ----------------
TMC2209Stepper driverPan(&Serial2, R_SENSE, 0b00);
TMC2209Stepper driverTilt(&Serial2, R_SENSE, 0b01);

AccelStepper motorPan(AccelStepper::DRIVER, 19, 18);
AccelStepper motorTilt(AccelStepper::DRIVER, 25, 26);

AS5600 encoderShared;

Eixo eixoPan(&driverPan, &motorPan, &encoderShared, 0, 17.0, 127.0, 2, "PAN", 1200);
Eixo eixoTilt(&driverTilt, &motorTilt, &encoderShared, 1, 21.0, 64.0, 2, "TILT", 800);

TaskHandle_t TaskMotoresHandle;
TaskHandle_t TaskSerialHandle;
TaskHandle_t TaskEncoderHandle;

// ---------------- Estado compartilhado (encoder -> serial) ----------------
// Escrito só por TaskEncoder, lido só por TaskSerial. Cada variável é lida/escrita
// atomicamente (floats de 32 bits alinhados no ESP32), então não usamos mutex aqui
// para não introduzir mais um ponto de bloqueio entre as tasks.
volatile float g_pan_pos_deg = 0, g_pan_vel_deg = 0;
volatile float g_tilt_pos_deg = 0, g_tilt_vel_deg = 0;

// ---------------- Estado da ponte serial ----------------
volatile unsigned long lastCmdMillis = 0;

void onSerialFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  switch (type) {
    case MSG_CMD_POS: {
      if (len != sizeof(CmdPosPayload)) return;
      CmdPosPayload p;
      memcpy(&p, payload, len);
      eixoPan.moverParaGrausAbsoluto(p.pan_rad * RAD_TO_DEG);
      eixoTilt.moverParaGrausAbsoluto(p.tilt_rad * RAD_TO_DEG);
      lastCmdMillis = millis();
      break;
    }
    case MSG_CMD_VEL: {
      if (len != sizeof(CmdVelPayload)) return;
      CmdVelPayload p;
      memcpy(&p, payload, len);
      float pan_dps = p.pan_rad_s * RAD_TO_DEG;
      float tilt_dps = p.tilt_rad_s * RAD_TO_DEG;

      if (fabs(pan_dps) < 0.01f) eixoPan.parar();
      else eixoPan.iniciarMovimentoContinuo(pan_dps);

      if (fabs(tilt_dps) < 0.01f) eixoTilt.parar();
      else eixoTilt.iniciarMovimentoContinuo(tilt_dps);

      lastCmdMillis = millis();
      break;
    }
    case MSG_SET_ZERO_REQ: {
      eixoPan.setZero();
      eixoTilt.setZero();
      SetZeroAckPayload ack{1};
      SerialFramer::sendFrame(Serial, MSG_SET_ZERO_ACK, (uint8_t*)&ack, sizeof(ack));
      lastCmdMillis = millis();
      break;
    }
    case MSG_HEARTBEAT:
      lastCmdMillis = millis();
      break;
  }
}

SerialFramer framer(onSerialFrame);

// ---------------- Tasks ----------------

// Core 1: Geração contínua de passos e processamento de trajetória
void TaskMotores(void* pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    eixoPan.processarRampaVelocidade();
    eixoTilt.processarRampaVelocidade();

    eixoPan.runStep();
    eixoTilt.runStep();

    esp_task_wdt_reset();
    vTaskDelay(1); // cede tempo para a idle task (evita watchdog reset por starvation)
  }
}

// Core 0: SÓ leitura dos encoders via I2C. Isolada para que um travamento do
// barramento não derrube a comunicação serial nem o fail-safe.
void TaskEncoder(void* pvParameters) {
  esp_task_wdt_add(NULL);

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / ENCODER_HZ);

  for (;;) {
    eixoPan.atualizarPosicaoEncoder();
    eixoTilt.atualizarPosicaoEncoder();

    g_pan_pos_deg  = eixoPan.getAnguloEixo();
    g_pan_vel_deg  = eixoPan.getVelocidadeEixo();
    g_tilt_pos_deg = eixoTilt.getAnguloEixo();
    g_tilt_vel_deg = eixoTilt.getVelocidadeEixo();

    esp_task_wdt_reset(); // só chega aqui se a leitura I2C não travou -> "prova de vida"
    vTaskDelayUntil(&lastWake, period);
  }
}

// Core 0: comunicação serial (não bloqueante) + fail-safe. NÃO toca em I2C.
void TaskSerial(void* pvParameters) {
  esp_task_wdt_add(NULL);
  pinMode(LED_STATUS, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  lastCmdMillis = millis();

  digitalWrite(LED_STATUS, HIGH); delay(200); digitalWrite(LED_STATUS, LOW);

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);

  for (;;) {
    while (Serial.available()) {
      framer.feed((uint8_t)Serial.read());
    }

    if (millis() - lastCmdMillis > FAILSAFE_TIMEOUT_MS) {
      eixoPan.parar();
      eixoTilt.parar();
    }

    TelemetryPayload t;
    t.pan_pos_rad  = g_pan_pos_deg  * DEG_TO_RAD;
    t.pan_vel_rad  = g_pan_vel_deg  * DEG_TO_RAD;
    t.tilt_pos_rad = g_tilt_pos_deg * DEG_TO_RAD;
    t.tilt_vel_rad = g_tilt_vel_deg * DEG_TO_RAD;
    SerialFramer::sendFrame(Serial, MSG_TELEMETRY, (uint8_t*)&t, sizeof(t));

    esp_task_wdt_reset();
    vTaskDelayUntil(&lastWake, period);
  }
}

void setup() {
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS); // limita o bloqueio máximo de qualquer transação I2C

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  eixoPan.begin();
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  eixoTilt.begin();
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  // Task Watchdog: se qualquer task registrada não chamar esp_task_wdt_reset()
  // dentro do timeout, a ESP32 reinicia sozinha em vez de ficar travada/lenta.
  esp_task_wdt_init(WDT_TIMEOUT_S, true /* panic -> reinicia */);

  xTaskCreatePinnedToCore(TaskMotores, "TaskMotores", 2048, NULL, 2, &TaskMotoresHandle, 1);
  xTaskCreatePinnedToCore(TaskEncoder, "TaskEncoder", 4096, NULL, 1, &TaskEncoderHandle, 0);
  xTaskCreatePinnedToCore(TaskSerial,  "TaskSerial",  8192, NULL, 1, &TaskSerialHandle,  0);
}

void loop() {
  vTaskDelete(NULL);
}