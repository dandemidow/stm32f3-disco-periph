// dandemidow(c)
#ifndef __UBX_STATE_H
#define __UBX_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct UbxState {
  virtual ~UbxState() = default;
  virtual int push(uint8_t const) = 0;
};

#ifdef __cplusplus
}
#endif

#endif // __UBX_STATE_H
