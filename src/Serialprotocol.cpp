#include "SerialProtocol.h"

uint8_t sb_crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
  }
  return crc;
}

void SerialFramer::feed(uint8_t b) {
  switch (state) {
    case WAIT_SYNC1:
      if (b == SB_SYNC1) state = WAIT_SYNC2;
      break;

    case WAIT_SYNC2:
      state = (b == SB_SYNC2) ? WAIT_TYPE : WAIT_SYNC1;
      break;

    case WAIT_TYPE:
      type = b;
      state = WAIT_LEN;
      break;

    case WAIT_LEN:
      len = b;
      payloadIdx = 0;
      if (len > SB_MAX_PAYLOAD) {
        // frame malformado/corrompido -> descarta e volta a procurar sync
        state = WAIT_SYNC1;
      } else {
        state = (len == 0) ? WAIT_CRC : WAIT_PAYLOAD;
      }
      break;

    case WAIT_PAYLOAD:
      payload[payloadIdx++] = b;
      if (payloadIdx >= len) state = WAIT_CRC;
      break;

    case WAIT_CRC: {
      uint8_t buf[2 + SB_MAX_PAYLOAD];
      buf[0] = type;
      buf[1] = len;
      memcpy(&buf[2], payload, len);
      uint8_t calc = sb_crc8(buf, 2 + len);

      if (calc == b && onFrame) {
        onFrame(type, payload, len);
      }
      // se o CRC falhar, simplesmente descarta o frame (sem travar nada)
      state = WAIT_SYNC1;
      break;
    }
  }
}

bool SerialFramer::sendFrame(Stream& out, uint8_t type, const uint8_t* payload, uint8_t len) {
  if (len > SB_MAX_PAYLOAD) return false;

  uint8_t buf[2 + SB_MAX_PAYLOAD];
  buf[0] = type;
  buf[1] = len;
  if (len) memcpy(&buf[2], payload, len);
  uint8_t crc = sb_crc8(buf, 2 + len);

  out.write((uint8_t)SB_SYNC1);
  out.write((uint8_t)SB_SYNC2);
  out.write(type);
  out.write(len);
  if (len) out.write(payload, len);
  out.write(crc);
  return true;
}