#include "Vertex.h"


void vertex_set_up_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] | 128;
}
bool vertex_get_up_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    return vertex->buf[2] & 128;
}
void vertex_clear_up_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] & (~128);
}

void vertex_set_down_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] | 64;
}
bool vertex_get_down_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    return vertex->buf[2] & 64;
}
void vertex_clear_down_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] & (~64);
}

void vertex_set_left_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] | 32;
}
bool vertex_get_left_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    return vertex->buf[2] & 32;
}
void vertex_clear_left_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] & (~32);
}

void vertex_set_right_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] | 16;
}
bool vertex_get_right_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    return vertex->buf[2] & 16;
}
void vertex_clear_right_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] & (~16);
}

void vertex_set_foward_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] | 8;
}
bool vertex_get_foward_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    return vertex->buf[2] & 8;
}
void vertex_clear_foward_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] & (~8);
}

void vertex_set_back_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] | 4;
}
bool vertex_get_back_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    return vertex->buf[2] & 4;
}
void vertex_clear_back_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] & (~4);
}
void vertex_set_dead_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] | 1;
}
bool vertex_get_dead_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    return vertex->buf[2] & 1;
}
void vertex_clear_dead_bit(Vertex_t* vertex){
    assert(vertex != NULL);
    vertex->buf[2] = vertex->buf[2] & (~1);
}


uint16_t build_vertex_coords(uint8_t x, uint8_t y, uint8_t z){
    assert(x < CHUNK_SIZE);
    assert(y < CHUNK_SIZE);
    assert(z < CHUNK_SIZE);
    uint16_t output = 0;
    output = output | (z << 1);
    output = output | (y << 6);
    output = output | (x << 11);
    return output;
}

uint8_t vertex_pick_x_coord(uint16_t coords){
    return (coords >> 11) & 31; 
}
uint8_t vertex_pick_y_coord(uint16_t coords){
    return (coords >> 6) & 31; 
}
uint8_t vertex_pick_z_coord(uint16_t coords){
    return (coords >> 1) & 31; 
}
