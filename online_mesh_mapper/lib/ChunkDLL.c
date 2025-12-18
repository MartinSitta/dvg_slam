#include "ChunkDLL.h"


ChunkListNode_t* chunk_dll_insert_at_start(ChunkListNode_t** head, int64_t x, int64_t y, int64_t z){ 
    ChunkListNode_t* node = chunk_list_node_init(x, y, z);
    if(node == NULL){
        return NULL;
    }
    if((*head) == NULL){
        *(head) = node;
    }
    else{
        node->next = (*head);
        (*head)->prev = node;
        (*head) = node;
    }
    return node;
}
bool chunk_dll_delete_at_start(ChunkListNode_t** head){
    if(*head == NULL){
        return false;
    }
    ChunkListNode_t* probe = (*head);
    (*head) = (*head)->next;
    if((*head) != NULL){
        (*head)->prev = NULL;
    }
    chunk_list_node_free(&probe);
    return true;
}
void chunk_dll_free(ChunkListNode_t** head){
    while(chunk_dll_delete_at_start(&(*head))){}
}
uint8_t chunk_dll_delete_by_pointer(ChunkListNode_t** head, ChunkListNode_t** ptr){
    if((*head) == NULL){
        return 1;
    }
    if((*ptr) == (*head)){
        bool status = chunk_dll_delete_at_start(&(*head));
        return !status;
    }
    if((*ptr)->next == NULL){
        (*ptr)->next->prev = (*ptr)->prev;
    }
       
    if((*ptr)->prev == NULL){
        (*ptr)->prev->next = (*ptr)->next;
    }
    chunk_list_node_free(&(*ptr));
    return 0;
}
