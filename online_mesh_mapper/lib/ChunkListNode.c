#include "ChunkListNode.h"
#include "Vertex.h"
ChunkListNode_t* chunk_list_node_init(int64_t x, int64_t y, int64_t z){
    ChunkListNode_t* node = malloc(sizeof(ChunkListNode_t));
    if(node == NULL){
        return NULL;
    }
    node->prev = NULL;
    node->next = NULL;
    node->chunk = malloc(sizeof(Chunk_t));
    chunk_init(node->chunk);
    node->chunk->x_offset = build_anchor_coord(x);               
    node->chunk->y_offset = build_anchor_coord(y);               
    node->chunk->z_offset = build_anchor_coord(z);
    return node;
}

void chunk_list_node_free(ChunkListNode_t** chunk_list_node){
    free((*chunk_list_node)->chunk);
    (*chunk_list_node)->prev = NULL;
    (*chunk_list_node)->next = NULL;
    free(*chunk_list_node);
    chunk_list_node = NULL;
}
