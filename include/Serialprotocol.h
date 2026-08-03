#pragma once
#include <Arduino.h>
#include <string.h>
#include <stdint.h>

// =====================================================================
// Protocolo de frame binário para comunicação ESP32 <-> Host (ROS 2)
//
// Formato do frame:
//   [SYNC1][SYNC2][TYPE][LEN][PAYLOAD...(LEN bytes)][CRC8]
//
//   SYNC1 = 0xA5, SYNC2 = 0x5A  -> permitem resincronizar após ruído/perda
//   TYPE  -> ver enum SerialMsgType
//   LEN   -> tamanho do payload em bytes (0-32)
//   CRC8  -> calculado sobre [TYPE][LEN][PAYLOAD]
//
// Todos os floats são little-endian (padrão em ESP32 e x86/ARM Linux).
// =====================================================================

#define SB_SYNC1 0xA5
#define SB_SYNC2 0x5A
#define SB_MAX_PAYLOAD 32

enum SerialMsgType : uint8_t {
  MSG_TELEMETRY    = 0x01, // ESP32 -> Host   (TelemetryPayload)
  MSG_CMD_POS      = 0x02, // Host  -> ESP32  (CmdPosPayload)
  MSG_CMD_VEL      = 0x03, // Host  -> ESP32  (CmdVelPayload)
  MSG_SET_ZERO_REQ = 0x04, // Host  -> ESP32  (sem payload)
  MSG_SET_ZERO_ACK = 0x05, // ESP32 -> Host   (SetZeroAckPayload)
  MSG_HEARTBEAT    = 0x06, // Bidirecional    (sem payload)
};

#pragma pack(push, 1)
struct TelemetryPayload {
  float pan_pos_rad;
  float pan_vel_rad;
  float tilt_pos_rad;
  float tilt_vel_rad;
};

struct CmdPosPayload {
  float pan_rad;
  float tilt_rad;
};

struct CmdVelPayload {
  float pan_rad_s;
  float tilt_rad_s;
};

struct SetZeroAckPayload {
  uint8_t success;
};
#pragma pack(pop)

uint8_t sb_crc8(const uint8_t* data, size_t len);

class SerialFramer {
public:
  // Callback chamado quando um frame válido (CRC ok) é recebido.
  using FrameCallback = void (*)(uint8_t type, const uint8_t* payload, uint8_t len);

  explicit SerialFramer(FrameCallback cb) : onFrame(cb) {}

  // Alimenta o parser byte a byte. NÃO bloqueia — chamar em loop enquanto
  // Serial.available() > 0. Reconstrói o estado sozinho se receber lixo.
  void feed(uint8_t b);

  // Monta e escreve um frame completo diretamente em um Stream (ex: Serial).
  static bool sendFrame(Stream& out, uint8_t type, const uint8_t* payload, uint8_t len);

private:
  enum State { WAIT_SYNC1, WAIT_SYNC2, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC };

  State state = WAIT_SYNC1;
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t payload[SB_MAX_PAYLOAD];
  uint8_t payloadIdx = 0;
  FrameCallback onFrame;
};