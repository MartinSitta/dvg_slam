#ifndef HASHUTILS_H
#define HASHUTILS_H

#ifdef __cplusplus
extern "C"{
#endif

#include "murmur3.h"
#include <stdint.h>
#include <assert.h>
#include "FibonacciHash.h"
uint32_t build_chunk_hash_table_hash(int64_t x, int64_t y, int64_t z, uint32_t seed);
uint64_t build_hash_map_hash(int64_t x, int64_t y, int64_t z, uint32_t seed);
uint64_t build_hash_map_double_hash(int64_t prev_hash, uint32_t seed);
uint64_t build_fibonacci_hash_from_coords(int64_t x, int64_t y, int64_t z);
uint16_t build_node_hash_table_hash(uint16_t coords, uint32_t seed);


#ifdef __cplusplus
}
#endif

#endif //HASHUTILS_H
