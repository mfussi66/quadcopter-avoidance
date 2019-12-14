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

/* Model */

void quad_linear_model(Vector *u, Matrix *A, Matrix *B, Vector *x);

void dlqr_control(Vector* sp, Vector* x, Matrix* K, Vector* u);

void compute_error(Vector *setpoint, Vector *state, Vector *result);

double compute_error_scabs(Vector* v1, Vector* v2, int idx);

void init_laser_scanner(Trace* tr, int n, double aperture, double* init_pose);

void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double spread, double n);

void compute_repulsive_force(Trace* tr, int n, double* pose, double* rep_force);

int chk_collisions(double* pose, Obstacle* obs, int n_obs);

double rad2deg(double n);

double deg2rad(double n);

#endif
