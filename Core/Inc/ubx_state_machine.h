// dandemidow(c)
#ifndef __UBX_STATE_MACHINE_H
#define __UBX_STATE_MACHINE_H

#include "ubx_state.h"

#include <array>
#include <cstdint>
#include <cstddef>

struct CK_t {
  uint8_t A {};
  uint8_t B {};

  void Add(uint8_t const v) {
    A = A + v;
    B = B + A;
  }
  void Clear() {
    A = 0U;
    B = 0U;
  }
};

struct SharedData {
  uint16_t size {};
  bool is_ok {false};
  std::array<uint8_t, 256U> payload {};
  CK_t ck {0U, 0U};
};

struct WaitStart : UbxState {
  int push(uint8_t const v) final;
};

struct WaitSecondStart : UbxState {
  int push(uint8_t const v) final;
};

struct MessageClass : UbxState {
  MessageClass(SharedData &data);
  SharedData &data_;
  uint8_t msg_class {};
  int push(uint8_t const v) final;
};

struct MessageId : UbxState {
  MessageId(SharedData &data);
  SharedData &data_;
  uint8_t msg_id {};
  int push(uint8_t const v) final;
};

struct MessageLen : UbxState {
  MessageLen(SharedData &data);
  SharedData &data_;
  size_t index {};
  alignas(uint16_t) std::array<uint8_t, 2U> buffer {};
  uint16_t *ptr {};
  inline uint16_t GetSize() const { return  *ptr; }
  int push(uint8_t const v) final;
};

struct MessagePayload : UbxState {
  MessagePayload(SharedData &data);
  SharedData &data_;
  size_t index_ {};
  int push(uint8_t const v) final;
};

struct MessageCrc : UbxState {
  MessageCrc(SharedData &data);
  SharedData &data_;
  size_t index_{};
  int push(uint8_t const v) final;
};

class GpsState {
  public:
    SharedData data {};
    WaitStart st_wait_start {};
    WaitSecondStart st_second_symbol {};
    MessageClass st_msg_class {data};
    MessageId st_msg_id {data};
    MessageLen st_msg_len {data};
    MessagePayload st_msg_payload {data};
    MessageCrc st_msg_crc {data};
    std::array<UbxState*, 7> table {{
        &st_wait_start,
        &st_second_symbol,
        &st_msg_class,
        &st_msg_id,
        &st_msg_len,
        &st_msg_payload,
        &st_msg_crc}};
    int32_t st_index {};
    std::array<uint8_t, 1024U> cyclic_ {};
    size_t read_index_ {};
    size_t write_index_ {};
    size_t get_free_space() const;
    size_t get_ready_data() const;
    void post_processing();
    void push(uint8_t const x);
    void clear();
};

#endif // __UBX_STATE_MACHINE_H
