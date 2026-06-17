#ifndef RS_H
#define RS_H

#include <cstdint>

void rs_init();
void rs_encode(uint8_t *msg, uint8_t *parity);

#endif // RS_H