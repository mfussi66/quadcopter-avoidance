/* Header File for RTS project */

/* --- Include guard --- */

#ifndef GRAPHICS_H_INCLUDED_
#define GRAPHICS_H_INCLUDED_

#include <allegro.h>
#include <stdio.h>
#include "customdata.h"

#define GRAPH_DATA_SIZE 50
#define GRAPH_STEP 2
#define GRAPH_SCALE 75
#define GRAPH_XPOS_XCOORD 150
#define GRAPH_XPOS_YCOORD 150
#define GRAPH_YPOS_XCOORD 150
#define GRAPH_YPOS_YCOORD 350
#define GRAPH_ZPOS_XCOORD 150
#define GRAPH_ZPOS_YCOORD 550

/* Graphics */

void start_allegro (void);

void close_allegro (void);

void update_graph (BITMAP* bmp, double* data, int coord_x, int coord_y);

#endif
