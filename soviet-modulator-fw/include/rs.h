#ifndef RS_H
#define RS_H

#include <cstdint>

void rs_init();
void rs_encode(uint8_t* msg, uint8_t* parity);
void rs_apply_dual_basis(uint8_t* data, int len);

#endif // RS_H