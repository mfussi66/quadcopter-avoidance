/* Header File for RTS project */

/* --- Include guard --- */

#ifndef GRAPHICS_H_INCLUDED_
#define GRAPHICS_H_INCLUDED_

#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include "customdata.h"

#define ENV_OFFSET_X 20
#define ENV_OFFSET_Y 575
#define ENV_SCALE 40

#define PLT_DATA_SIZE 25
#define PLT_STEP 4
#define PLT_SCALE 5
#define PLT_XPOS_XCOORD 795
#define PLT_XPOS_YCOORD 385
#define PLT_YPOS_XCOORD 795
#define PLT_YPOS_YCOORD 490
#define PLT_ZPOS_XCOORD 795
#define PLT_ZPOS_YCOORD 595

/* Graphics */

void start_allegro(void);

void close_allegro(void);

void build_gui(BITMAP* bmp, FONT* font, int col);

void draw_exit_screen(BITMAP* bmp, int col);

void update_plot(BITMAP* bmp, double* data, int coord_x, int coord_y);

void update_pose(BITMAP* bmp, double* old, double* new);
#endif
