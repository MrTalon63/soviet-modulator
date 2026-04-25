#ifndef RS_H
#define RS_H

#include <cstdint>

void rs_init();
void rs_encode(const uint8_t* msg_base, uint8_t* parity, int offset, int stride);
void rs_apply_dual_basis(uint8_t* data, int len);

#endif // RS_H