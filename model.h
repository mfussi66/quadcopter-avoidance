/* Header File for RTS project */

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

void lin_model(double* u, double* x, double yaw_sp);

void pid_rpy_alt_control(double* e, double* e_prev, double* u);

void pid_xy_control(double* e, double* e_prev, double* rp_sp, double yaw);

void rotate_error(double* e, double yaw);

void pid_control(double* e, double* e_prev, double* u, double yaw);

void compute_setpoint(double* sp, WPoint* wp, double* pose, double alt, int wp_size, int* wp_flags);

void dlqr_control(Vector* sp, Vector* x, Matrix* K, Vector* u, double yaw);

double compute_yaw_ref(double yaw, WPoint* sp, double yaw_laser, double gain);

void compute_error(double* sp, double* x, double* e, double* e_old, int size);

double compute_pos_dist(double* v1, double* v2);

void init_laser_scanner(Trace* tr, int n, double aperture, double* init_pose);

void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double spread, double n);

void compute_histogram(Trace* tr, int n, double* hist);

void find_valleys(double* hist, Valley* valleys, int size, int* v_size, int threshold);

double compute_heading(Trace* tr, Valley* v, int v_size, double* pose, WPoint* sp);

void compute_repulsive_force(Trace* tr, int n, double* pose, double* rep_force);

int chk_collisions(double* pose, Obstacle* obs, int n_obs);

double rad2deg(double n);

double deg2rad(double n);

double pow2(double n);

double atan2_safe(double y, double x);

#endif
