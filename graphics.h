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

int gen_obstacles(Obstacle* arr_obstacles, int n_obs);

void draw_obstacles(BITMAP* bmp, Obstacle* obs, int n_obs, int col);
 
void update_plot(BITMAP* bmp, double* data, int coord_x, int coord_y);

void draw_quad(BITMAP* bmp, BITMAP* quad, BITMAP* bg, double* old, double* new);

void draw_pose(BITMAP* bmp, double* old, double* new);

void draw_laser_traces(BITMAP *bmp, Trace* old, Trace* new, double* old_pose, double *pose);

int waypoints_filled(WPoint *array, int size);

void draw_waypoints(BITMAP* bmp, WPoint* old_wpoints, WPoint* wpoints, int size);

#endif
