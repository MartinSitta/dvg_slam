#include "FibonacciHash.h"

uint64_t fibonacci_hash(uint8_t* buf, uint8_t byte_len){
    uint64_t h = 0xcbf29ce484222325ULL; // arbitrary nonzero seed
    uint64_t* words = (uint64_t*)buf;
    for(uint8_t i = 0; i < byte_len/8; i++){
        h ^= words[i];
        h *= 11400714819323198485ULL;
        h ^= h >> 32;
    }
    return h;
}
uint64_t fibonacci_doublehash(uint64_t prev_hash){
    prev_hash ^= prev_hash >> 32;
    uint64_t step = prev_hash * 2654435769ULL;
    return step | 1ULL;  // odd => coprime with any 2^n
}

