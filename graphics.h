/* -------------------------- 
	 REAL TIME SYSTEMS
	 OBSTACLE AVOIDANCE
	   Mattia Fussi
	
	    GRAPHICS HEADER
 -------------------------- */

/* --- Include guard --- */

#ifndef GRAPHICS_H_INCLUDED_
#define GRAPHICS_H_INCLUDED_

#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include "customdata.h"

/* Graphics */

void start_allegro(int mode);

void close_allegro(void);

void build_gui(BITMAP* bmp, FONT* font, int col);

void draw_exit_screen(BITMAP* bmp, int col);

int gen_obstacles(Obstacle* arr_obstacles, int n_obs);

void draw_obstacles(BITMAP* bmp, Obstacle* obs, int n_obs, int col);
 
void update_plot(BITMAP* bmp, double* data, int coord_x, int coord_y, double scale);

void draw_quad(BITMAP* bmp, BITMAP* quad, BITMAP* bg, double* old, double* new);

void draw_pose(BITMAP* bmp, double* old, double* new);

void draw_periods(BITMAP* bmp, int* tp, int size, int sel);

void draw_laser_traces(BITMAP *bmp, Trace* old, Trace* new, double* old_pose, double *pose);

void draw_laser_points(BITMAP *bmp, Trace* old, Trace* new, double* old_pose, double *pose);

void add_waypoint(BITMAP* bmp, WPoint* array, int* num,  WPoint point);

void del_waypoint(BITMAP* bmp, WPoint* array, int* num,  WPoint point);

void draw_waypoints(BITMAP* bmp, WPoint* old_wpoints, WPoint* wpoints, int size);

void draw_gains(BITMAP* bmp, double* p, double* d, int sel);

void draw_altitude(BITMAP* bmp, double alt, double sp);

void draw_msg(BITMAP* bmp, int mode, int x, int y);

#endif
