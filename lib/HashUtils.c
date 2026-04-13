#include "HashUtils.h"

typedef struct{
    int64_t x;
    int64_t y;
    int64_t z;
} CoordHelper_t;

uint32_t build_chunk_hash_table_hash(int64_t x, int64_t y, int64_t z, uint32_t seed){
    assert(seed != 0);
    uint32_t output = 0;
    CoordHelper_t coords = {x,y,z};
    assert(coords.x == x);
    assert(coords.y == y);
    assert(coords.z == z);
    uint32_t byte_len = sizeof(CoordHelper_t);
    MurmurHash3_x86_32((void*)&coords, byte_len, seed, &output);
    return output;
}

uint16_t build_node_hash_table_hash(uint16_t coords, uint32_t seed){
    assert(seed != 0);
    uint32_t output = 0;
    uint16_t ret_val;
    MurmurHash3_x86_32((void*)&coords, sizeof(uint16_t), seed, &output);
    ret_val = (uint16_t) (output & ((1 << 15) - 1));
    return ret_val;
}
