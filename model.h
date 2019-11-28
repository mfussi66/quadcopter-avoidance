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

// typedef gsl_vector vect_f;
// typedef gsl_matrix_float matr_f;

/* Model */

void quad_linear_model(gsl_vector *u, gsl_matrix *A, gsl_matrix *B, gsl_vector *x);

//void quad_linear_model(gsl_vector *sp, gsl_matrix *A, gsl_matrix *B, gsl_vector *x, gsl_matrix* K);

void dlqr_control(gsl_vector* sp, gsl_vector* x, gsl_matrix* K, gsl_vector* u);

void compute_error(gsl_vector *setpoint, gsl_vector *state, gsl_vector *result);

#endif
