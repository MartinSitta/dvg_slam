#ifndef VERTEX_H
#define VERTEX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <stdbool.h>
#include "GraphDefines.h"
//due to the implementation of instancing I only need to describe the topleft vertex of the shape. therefore I dont need 0-32 but 0-31
//this struct is the information for all vertices of a voxel
//this is done by one vertex struct per voxel and instancing a rectangle on it
//the direction flags apply a rotation.
//Bitlayout:
//5 bits x coord
//5 bits y coord
//5 bits z coord
//1 bit dead bit
//1 bit up
//1 bit down
//1 bit left
//1 bit right
//1 bit foward
//1 bit back
//1 bit dead bit
//1 bit deleted flag
//multiple directions will result in multiple transforms getting applied
typedef union{
    uint16_t vertex_coords;
    uint8_t buf[3];
}Vertex_t;

uint8_t vertex_pick_x_coord(uint16_t coords);
uint8_t vertex_pick_y_coord(uint16_t coords);
uint8_t vertex_pick_z_coord(uint16_t coords);

void vertex_set_up_bit(Vertex_t* vertex);
void vertex_set_down_bit(Vertex_t* vertex);
void vertex_set_left_bit(Vertex_t* vertex);
void vertex_set_right_bit(Vertex_t* vertex);
void vertex_set_foward_bit(Vertex_t* vertex);
void vertex_set_back_bit(Vertex_t* vertex);
void vertex_set_dead_bit(Vertex_t* vertex);


void vertex_clear_up_bit(Vertex_t* vertex);
void vertex_clear_down_bit(Vertex_t* vertex);
void vertex_clear_left_bit(Vertex_t* vertex);
void vertex_clear_right_bit(Vertex_t* vertex);
void vertex_clear_foward_bit(Vertex_t* vertex);
void vertex_clear_back_bit(Vertex_t* vertex);
void vertex_clear_dead_bit(Vertex_t* vertex);

bool vertex_get_up_bit(Vertex_t* vertex);
bool vertex_get_down_bit(Vertex_t* vertex);
bool vertex_get_left_bit(Vertex_t* vertex);
bool vertex_get_right_bit(Vertex_t* vertex);
bool vertex_get_foward_bit(Vertex_t* vertex);
bool vertex_get_back_bit(Vertex_t* vertex);
bool vertex_get_dead_bit(Vertex_t* vertex);

uint16_t build_vertex_coords(uint8_t x, uint8_t y, uint8_t z);

#ifdef __cplusplus
}
#endif

#endif
