#ifndef FIBONACCIHASH_H
#define FIBONACCIHASH_H

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
uint64_t fibonacci_hash(uint8_t* buf, uint8_t byte_len);
uint64_t fibonacci_doublehash(uint64_t prev_hash);
#ifdef __cplusplus
}
#endif
#endif//FIBONACCIHASH_H
