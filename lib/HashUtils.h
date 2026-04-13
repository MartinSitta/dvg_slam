#ifndef HASHUTILS_H
#define HASHUTILS_H

#ifdef __cplusplus
extern "C"{
#endif

#include "murmur3.h"
#include <stdint.h>
#include <assert.h>
#include "Vertex.h"

uint32_t build_chunk_hash_table_hash(int64_t x, int64_t y, int64_t z, uint32_t seed);

uint16_t build_node_hash_table_hash(uint16_t coords, uint32_t seed);


#ifdef __cplusplus
}
#endif

#endif //HASHUTILS_H
