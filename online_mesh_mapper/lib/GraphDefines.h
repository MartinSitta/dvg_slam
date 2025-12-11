#ifndef GRAPHDEFINES_H
#define GRAPHDEFINES_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#define CHUNK_LIMIT (1<<14)
#define CHUNK_HASH_TABLE_SIZE (1<<29)
#define CHUNK_NODE_LIMIT (1<<14)
#define chunk_node_array_entry_t int16_t
#define CHUNK_NODE_HASHTABLE_SIZE (1<<17)
#define CHUNK_SIZE 32 //DO NOT CHANGE THIS EVER!
#ifdef __cplusplus
}
#endif
#endif
