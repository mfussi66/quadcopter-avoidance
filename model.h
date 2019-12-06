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

typedef struct
{
	double x;
	double y;
	double z;
} Trace;

/* Model */

void quad_linear_model(Vector *u, Matrix *A, Matrix *B, Vector *x);

void dlqr_control(Vector* sp, Vector* x, Matrix* K, Vector* u);

void compute_error(Vector *setpoint, Vector *state, Vector *result);

void init_laser_scanner(Trace* tr, int n, double aperture, double* init_pose);

void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double aperture);

double rad2deg(double n);

double deg2rad(double n);

#endif
