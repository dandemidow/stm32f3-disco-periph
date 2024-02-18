// dandemidow(c)

#include "ubx_state_machine.h"

#include <new>

int WaitStart::push(uint8_t const v) {
  return (v == 0xB5U) ? 1 : 0;
}

int WaitSecondStart::push(uint8_t const v) {
  return (v == 0x62U) ? 1 : -1;
}

MessageClass::MessageClass(SharedData &data)
    : data_ {data} {
}

int MessageClass::push(uint8_t const v) {
  data_.ck.Clear();
  data_.size = 0;
  msg_class = v;
  data_.ck.Add(v);
  return 1;
}

MessageId::MessageId(SharedData &data) : data_ {data} {}
int MessageId::push(uint8_t const v) {
  msg_id = v;
  data_.ck.Add(v);
  return 1;
}


MessageLen::MessageLen(SharedData &data)
    : data_ {data},
      ptr {new (buffer.data()) uint16_t} {}

int MessageLen::push(uint8_t const v) {
  buffer[index] = v;
  data_.ck.Add(v);
  index++;
  if (index == 2U) {
    index = 0U;
    data_.size = GetSize();
    if (data_.size > data_.payload.size()) {
      return -4;
    }
    return 1;
  }
  return 0;
}


MessagePayload::MessagePayload(SharedData &data) : data_ {data} {}

int MessagePayload::push(uint8_t const v) {
  data_.payload[index_] = v;
  data_.ck.Add(v);
  index_++;
  if (index_ == data_.size) {
    index_ = 0;
    return 1;
  }
  return 0;
}

MessageCrc::MessageCrc(SharedData &data) : data_ {data} {}

int MessageCrc::push(uint8_t const v) {
  switch (index_) {
    case 0:
      index_++;
      return data_.ck.A == v ? 0 : -6;
    case 1:
      index_ = 0;
      data_.is_ok = data_.ck.B == v;
      return -6;
  }
  return 0;
}

size_t GpsState::get_free_space() const {
  return (cyclic_.size() + read_index_ - write_index_) % cyclic_.size();
}

size_t GpsState::get_ready_data() const {
  return (cyclic_.size() + write_index_ - read_index_) % cyclic_.size();
}

void GpsState::post_processing() {
  while (get_ready_data()) {
    int32_t const offset {table[st_index]->push(cyclic_[read_index_])};
    st_index += offset;
    read_index_ = (read_index_ + 1U) % cyclic_.size();
  }
}

void GpsState::push(uint8_t const x) {
  cyclic_[write_index_] = x;
  write_index_ = (write_index_ + 1U) % cyclic_.size();
  if (read_index_ == write_index_) {
    read_index_ = (read_index_ + 1U) % cyclic_.size();
  }
}

void GpsState::clear() {
  read_index_ = 0;
  write_index_ = 0;
  st_index = 0;
  data.is_ok = false;
  data.size = 0U;
}
