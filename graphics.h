/* Header File for RTS project */

/* --- Include guard --- */

#ifndef GRAPHICS_H_INCLUDED_
#define GRAPHICS_H_INCLUDED_

#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include "customdata.h"

#define ENV_OFFSET_X 20
#define ENV_OFFSET_Y 20
#define ENV_SCALE 40

#define PLT_DATA_SIZE 50
#define PLT_STEP 2
#define PLT_SCALE 75
#define PLT_XPOS_XCOORD 795
#define PLT_XPOS_YCOORD 385
#define PLT_YPOS_XCOORD 795
#define PLT_YPOS_YCOORD 490
#define PLT_ZPOS_XCOORD 795
#define PLT_ZPOS_YCOORD 595

/* Graphics */

void start_allegro (void);

void close_allegro (void);

void update_graph (BITMAP* bmp, double* data, int coord_x, int coord_y);

void update_pose (BITMAP* bmp, double* old, double* new);
#endif
