/* Header File for RTS project */

/* --- Include guard --- */

#ifndef MODEL_H_INCLUDED_
#define MODEL_H_INCLUDED_

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

//void quad_linear_model(Vector *sp, Matrix *A, Matrix *B, Vector *x, Matrix* K);

void dlqr_control(Vector* sp, Vector* x, Matrix* K, Vector* u);

void compute_error(Vector *setpoint, Vector *state, Vector *result);

#endif
