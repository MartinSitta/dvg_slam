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
uint64_t build_hash_map_hash(int64_t x, int64_t y, int64_t z, uint32_t seed){
    uint64_t output [2] = {0,0};
    CoordHelper_t coords = {x,y,z};
    assert(coords.x == x);
    assert(coords.y == y);
    assert(coords.z == z);
    uint32_t byte_len = sizeof(CoordHelper_t);
    MurmurHash3_x64_128((void*)&coords, byte_len, seed, &output);
    return output[0];
}
uint64_t build_hash_map_double_hash(int64_t prev_hash, uint32_t seed){
    uint64_t output [2] = {0,0};
    uint32_t byte_len = sizeof(CoordHelper_t);
    MurmurHash3_x64_128((void*)&prev_hash, sizeof(int64_t), seed, &output);
    return output[0];
}
uint64_t build_fibonacci_hash_from_coords(int64_t x, int64_t y, int64_t z){
    int64_t buf[3] = {x, y, z};
    uint64_t hash = fibonacci_hash((uint8_t*)buf, sizeof(buf));
    return hash;
}
uint16_t build_node_hash_table_hash(uint16_t coords, uint32_t seed){
    assert(seed != 0);
    uint32_t output = 0;
    uint16_t ret_val;
    MurmurHash3_x86_32((void*)&coords, sizeof(uint16_t), seed, &output);
    ret_val = (uint16_t) (output & ((1 << 15) - 1));
    return ret_val;
}
