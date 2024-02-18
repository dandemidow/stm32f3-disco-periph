// dandemidow(c)
#ifndef __TX_MESSAGE_H
#define __TX_MESSAGE_H

#include <cstdint>
#include <new>

extern "C" {
#include "main.h"
}

#include "ubx_state_machine.h"

template <class T>
struct UbxTrait;

struct UbxHeader {
  uint8_t sync1 {0xB5U};
  uint8_t sync2 {0x62U};
  uint8_t ubx_class;
  uint8_t ubx_id;
  uint16_t len;
  uint8_t payload[0];
};


struct CfgPort {
  uint8_t port_id;
  uint8_t reserved;
  uint16_t tx_ready;
  uint32_t mode;
  uint32_t baudrate;
  uint16_t in_proto_mask;
  uint16_t out_proto_mask;
  uint16_t flags;
  uint16_t reserved1;
};

struct CfgRate {
  uint16_t meas_rate;
  uint16_t nav_rate;
  uint16_t time_ref;
};

template <>
struct UbxTrait<CfgRate> {
  static constexpr uint8_t ubx_class {0x06U};
  static constexpr uint8_t ubx_id {0x08U};
};

enum PortMode {
  START = 0x1 << 4,
  CH_7BIT = 0x2U << 6,
  CH_8BIT = 0x3U << 6,
  EVEN_PARITY = 0x0 << 9,
  ODD_PARITY = 0x1 << 9,
  NO_PARITY = 0x4 << 9,
  STOP_1 = 0x0 << 12,
  STOP_15 = 0x1 << 12,
  STOP_2 = 0x2 << 12,
  STOP_05 = 0x3 << 12,
};

enum Proto {
  kUbx = 0b001,
  kNmea = 0b010,
  kRtcm = 0b100
};

template <>
struct UbxTrait<CfgPort> {
  static constexpr uint8_t ubx_class {0x06U};
  static constexpr uint8_t ubx_id {0x00U};
};

CK_t CalcCrc(uint8_t *buffer, size_t const len);

template <class T>
size_t CreateMessage(T const & msg, uint8_t *buffer) {
  size_t size {sizeof(UbxHeader)};
  UbxHeader *mg = new (buffer) UbxHeader{};
  mg->ubx_class = UbxTrait<T>::ubx_class;
  mg->ubx_id = UbxTrait<T>::ubx_id;
  mg->len = sizeof(T);
  memcpy(mg->payload, &msg, sizeof(T));
  size += sizeof(T);
  auto crc = CalcCrc(buffer + 2, size - 2);
  buffer[size++] = crc.A;
  buffer[size++] = crc.B;
  return size;
}

#endif // __TX_MESSAGE_H
