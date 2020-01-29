/* -------------------------- 
	 REAL TIME SYSTEMS
	 OBSTACLE AVOIDANCE
	   Mattia Fussi
	
   MODEL and CONTROL HEADER
 -------------------------- */


/* --- Include guard --- */

#ifndef MODEL_H_INCLUDED_
#define MODEL_H_INCLUDED_

#include <allegro.h>
#include <math.h>
#include <stdio.h>
#include "customdata.h"
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>

typedef gsl_vector Vector;
typedef gsl_matrix Matrix;

#define Jxx 0.0099
#define Jyy 0.0099
#define Jzz 0.0189
#define M 0.42

/* Model */

void quad_linear_model(Vector *u, Matrix *A, Matrix *B, Vector *x);

void lin_model(double* u, double* x, double dt);

void pid_rpy_control(double* e, double* e_prev, double dt, double* u, double* p, double* d);

void pid_xyz_control(double* e, double* e_prev, double dt, double* u, double* vel_sp, double* p, double* d, int avoid);

void pid_vel_control(double* e, double* e_prev, double dt, double* rp_sp, double* p, double* d);

void rotate_error(double* e, double yaw);

void pid_control(double* e, double* e_prev, double* u, double yaw);

void next_setpoint(double* sp, WPoint* wp, int wp_size, int* wp_flags);

void dlqr_control(Vector* sp, Vector* x, Matrix* K, Vector* u, double yaw);

void compute_error(double* sp, double* x, double* e, double* e_old, int size);

double compute_pos_dist(double* v1, double* v2);

void init_laser_scanner(Trace* tr, int n, double aperture, double* init_pose);

void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double spread, double n);

void compute_histogram(Trace* tr, int n, double* hist);

void find_valleys(double* hist, Valley* valleys, int size, int* v_size, int threshold);

double compute_heading(Trace* tr, Valley* v, int v_size, double* pose, WPoint* sp);

Trace get_shortest_beam(Trace* tr, double threshold);

void compute_repulsive_force(Trace* tr, double* pose, double *rep_forces, double ampli_thr);

int set_avoid_mode(Trace* tr, double* pose, WPoint* target, int* turn_dir, int old_mode, int old_turn_dir);

void compute_avoid_sp(double* pose, int rn, int mode, double *vel_sp, double *yaw_sp, int turn_dir);

//int compute_avoid_sp(Trace* tr, double* pose, WPoint* target, double ampli_thr, double *vel_sp, double *yaw_sp);

int chk_collisions(double* pose, Obstacle* obs, int n_obs);

void init_gains(double* p, double* d, double* p_df, double* d_df);

void adjust_gain(double* p, double* d, int mode, int updown);

void reset_gains(double* p, double* d, double* p_df, double* d_df);

double rad2deg(double n);

double deg2rad(double n);

double pow2(double n);

double atan2_safe(double y, double x);

double anglediff_safe(double sp, double theta);

double get_uniform_num();

#endif
