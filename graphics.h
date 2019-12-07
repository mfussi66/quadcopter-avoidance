/* Header File for RTS project */

/* --- Include guard --- */

#ifndef GRAPHICS_H_INCLUDED_
#define GRAPHICS_H_INCLUDED_

#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include "customdata.h"

/* Graphics */

void start_allegro(void);

void close_allegro(void);

void build_gui(BITMAP* bmp, FONT* font, int col);

void draw_exit_screen(BITMAP* bmp, int col);

void update_plot(BITMAP* bmp, double* data, int coord_x, int coord_y);

void draw_pose(BITMAP* bmp, double* old, double* new);

void draw_laser_traces(BITMAP *bmp, Trace* old, Trace* new, double* old_pose, double *pose);

#endif
