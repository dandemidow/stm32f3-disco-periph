// dandemidow(c)

#include "tx_message.h"

#include <cmath>

CK_t CalcCrc(uint8_t *buffer, const size_t len) {
  CK_t ck = {0U, 0U};
  for (size_t i = 0U; i < len; i++) {
    ck.A = ck.A + buffer[i];
    ck.B = ck.B + ck.A;
  }
  return ck;
}
