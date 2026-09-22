#include <Arduino.h>

#include <Wire.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <AS5600.h>

#include "esp_task_wdt.h"
#include "esp_system.h"
#include "Eixo.h"
#include "SerialProtocol.h"

#define ENDERECO_PCA9548A 0x70
#define R_SENSE 0.11f
#define RXD2 16
#define TXD2 17
#define EN_PIN 27
#define LED_STATUS 2

#define SDA_PIN 21
#define SCL_PIN 22

#define PINO_OUT_PAN  34
#define PINO_OUT_TILT 35

// Deixe true se QUALQUER eixo usar ENCODER_MODO_I2C. Como você está usando
// só o modo analógico nos dois, isso pula Wire.begin()/setClock()/setTimeOut()
// por completo - libera os pinos 21/22 e não inicializa o periférico à toa.
#define USAR_I2C false

#define SERIAL_BAUD 921600
#define TELEMETRY_HZ 20
#define ENCODER_HZ 50
#define FAILSAFE_TIMEOUT_MS 500     // sem comando/heartbeat -> para os motores
#define I2C_TIMEOUT_MS 30           // limite de bloqueio de qualquer transação I2C
#define I2C_CLOCK_HZ 100000         // 100kHz: mais lento, bem mais tolerante a ruído que 400kHz
#define WDT_TIMEOUT_S 3             // task sem "sinal de vida" por esse tempo -> reset automático
#define MAX_FALHAS_I2C_CONSECUTIVAS 5
#define HEAP_MINIMO_BYTES 20000

// ---------------- Hardware ----------------
TMC2209Stepper driverPan(&Serial2, R_SENSE, 0b00);
TMC2209Stepper driverTilt(&Serial2, R_SENSE, 0b01);

AccelStepper motorPan(AccelStepper::DRIVER, 19, 18);
AccelStepper motorTilt(AccelStepper::DRIVER, 25, 26);

AS5600 encoderShared;

Eixo eixoPan(&driverPan, &motorPan, &encoderShared, 0, 17.0, 127.0, 2, "PAN", 1200,
             PINO_OUT_PAN, ENCODER_MODO_ANALOGICO);
Eixo eixoTilt(&driverTilt, &motorTilt, &encoderShared, 1, 21.0, 64.0, 2, "TILT", 800,
              PINO_OUT_TILT, ENCODER_MODO_ANALOGICO);

TaskHandle_t TaskMotoresHandle;
TaskHandle_t TaskSerialHandle;
TaskHandle_t TaskEncoderHandle;

SemaphoreHandle_t serialMutex; // protege escrita concorrente na Serial (TaskSerial + TaskEncoder)

// ---------------- Estado compartilhado (encoder -> serial) ----------------
volatile float g_pan_pos_deg = 0, g_pan_vel_deg = 0;
volatile float g_tilt_pos_deg = 0, g_tilt_vel_deg = 0;

// ---------------- Estado da ponte serial ----------------
volatile unsigned long lastCmdMillis = 0;

// Envio thread-safe de frames (usado por TaskSerial e TaskEncoder)
bool sendFrameSafe(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    bool ok = SerialFramer::sendFrame(Serial, type, payload, len);
    xSemaphoreGive(serialMutex);
    return ok;
  }
  return false; // não conseguiu o mutex a tempo; não trava, só descarta esse envio
}

void reportError(uint8_t code) {
  ErrorPayload err{code};
  sendFrameSafe(MSG_ERROR, (uint8_t*)&err, sizeof(err));
}

// ---------------- Recuperação manual do barramento I2C ----------------
// Se um dispositivo travar segurando SDA em nível baixo (clock stretching preso),
// gera até 9 pulsos de clock manualmente para liberar, depois um STOP, e reinicia o Wire.
void recuperarBarramentoI2C() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
  }

  // Gera condição de STOP manualmente (SDA sobe enquanto SCL está em HIGH)
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

void onSerialFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  switch (type) {
    case MSG_CMD_POS: {
      if (len != sizeof(CmdPosPayload)) { reportError(ERR_CMD_POS_INVALID); return; }
      CmdPosPayload p;
      memcpy(&p, payload, len);
      eixoPan.moverParaGrausAbsoluto(p.pan_rad * RAD_TO_DEG);
      eixoTilt.moverParaGrausAbsoluto(p.tilt_rad * RAD_TO_DEG);
      lastCmdMillis = millis();
      break;
    }
    case MSG_CMD_VEL: {
      if (len != sizeof(CmdVelPayload)) { reportError(ERR_CMD_VEL_INVALID); return; }
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
      sendFrameSafe(MSG_SET_ZERO_ACK, (uint8_t*)&ack, sizeof(ack));
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

// Core 1: geração contínua de passos e processamento de trajetória
void TaskMotores(void* pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    eixoPan.processarRampaVelocidade();
    eixoTilt.processarRampaVelocidade();

    eixoPan.runStep();
    eixoTilt.runStep();

    esp_task_wdt_reset();
    vTaskDelay(1);
  }
}

// Core 0: SÓ leitura dos encoders via I2C, com detecção de falha e auto-recuperação.
// Isolada para que um travamento do barramento não derrube a serial nem o fail-safe.
void TaskEncoder(void* pvParameters) {
  esp_task_wdt_add(NULL);

  uint8_t falhasPan = 0, falhasTilt = 0;

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / ENCODER_HZ);

  for (;;) {
    if (eixoPan.atualizarPosicaoEncoder()) {
      falhasPan = 0;
      g_pan_pos_deg = eixoPan.getAnguloEixo();
      g_pan_vel_deg = eixoPan.getVelocidadeEixo();
    } else {
      falhasPan++;
      reportError(ERR_I2C_TIMEOUT_PAN);
    }

    if (eixoTilt.atualizarPosicaoEncoder()) {
      falhasTilt = 0;
      g_tilt_pos_deg = eixoTilt.getAnguloEixo();
      g_tilt_vel_deg = eixoTilt.getVelocidadeEixo();
    } else {
      falhasTilt++;
      reportError(ERR_I2C_TIMEOUT_TILT);
    }

    if (falhasPan >= MAX_FALHAS_I2C_CONSECUTIVAS || falhasTilt >= MAX_FALHAS_I2C_CONSECUTIVAS) {
      recuperarBarramentoI2C();
      reportError(ERR_I2C_BUS_RECOVERED);
      falhasPan = 0;
      falhasTilt = 0;
    }

    esp_task_wdt_reset();
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

  // Informa ao host o motivo do boot/reset anterior + heap livre
  BootInfoPayload boot{};
  boot.reset_reason = (uint8_t)esp_reset_reason();
  boot.free_heap = ESP.getFreeHeap();
  sendFrameSafe(MSG_BOOT_INFO, (uint8_t*)&boot, sizeof(boot));

  bool failsafeAtivo = false;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);

  for (;;) {
    while (Serial.available()) {
      framer.feed((uint8_t)Serial.read());
    }

    if (millis() - lastCmdMillis > FAILSAFE_TIMEOUT_MS) {
      eixoPan.parar();
      eixoTilt.parar();
      if (!failsafeAtivo) { // reporta só na borda de subida, não a cada ciclo
        reportError(ERR_FAILSAFE_TRIGGERED);
        failsafeAtivo = true;
      }
    } else {
      failsafeAtivo = false;
    }

    if (ESP.getFreeHeap() < HEAP_MINIMO_BYTES) {
      reportError(ERR_LOW_HEAP);
    }

    TelemetryPayload t;
    t.pan_pos_rad  = g_pan_pos_deg  * DEG_TO_RAD;
    t.pan_vel_rad  = g_pan_vel_deg  * DEG_TO_RAD;
    t.tilt_pos_rad = g_tilt_pos_deg * DEG_TO_RAD;
    t.tilt_vel_rad = g_tilt_vel_deg * DEG_TO_RAD;
    sendFrameSafe(MSG_TELEMETRY, (uint8_t*)&t, sizeof(t));

    esp_task_wdt_reset();
    vTaskDelayUntil(&lastWake, period);
  }
}

void setup() {
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

#if USAR_I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);      // 100kHz: mais robusto a ruído que 400kHz
  Wire.setTimeOut(I2C_TIMEOUT_MS);
#endif

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  eixoPan.begin();
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  eixoTilt.begin();
  digitalWrite(LED_STATUS, HIGH); delay(150); digitalWrite(LED_STATUS, LOW); delay(150);

  serialMutex = xSemaphoreCreateMutex();

  esp_task_wdt_init(WDT_TIMEOUT_S, true /* panic -> reinicia */);

  xTaskCreatePinnedToCore(TaskMotores, "TaskMotores", 2048, NULL, 2, &TaskMotoresHandle, 1);
  xTaskCreatePinnedToCore(TaskEncoder, "TaskEncoder", 4096, NULL, 1, &TaskEncoderHandle, 0);
  xTaskCreatePinnedToCore(TaskSerial,  "TaskSerial",  8192, NULL, 1, &TaskSerialHandle,  0);
}

void loop() {
  vTaskDelete(NULL);
}