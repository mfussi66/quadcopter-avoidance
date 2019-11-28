/* Function file for RTS Project */

#include "model.h"

/* Model */

/* 
 * Function: error computation
 * ---------------------------
 * Computes error between setpoint and measured state
 * allocates a new vector so that gsl_vector_sub() does not
 * overwrite the first argument
 */
void compute_error(gsl_vector* setpoint, gsl_vector* state, gsl_vector *result)
{
	gsl_vector* temp = gsl_vector_alloc_from_vector(setpoint, 0, SIZE_X, 1);
	gsl_vector_sub(temp, state);
	gsl_vector_memcpy(result, temp);
}

/* 
 * Function: control action computation
 * ---------------------------
 * computes the control action as e = K*u
 * the K gain matrix is computed offline from the solution of the
 * Discrete Linear Quadratic Regulator problem
 * Implements u = 1 * K * e + 0 * u
 */
void dlqr_control(gsl_vector* sp, gsl_vector* x, gsl_matrix* K, gsl_vector* u)
{
	gsl_vector* e = gsl_vector_calloc(SIZE_X);
	gsl_vector_memcpy(e, sp);
	
 	gsl_vector_sub(e, x);

	gsl_blas_dgemv(CblasNoTrans, 1, K, e, 0, u);
}

/* 
 * Function: Linearized system equation
 * ---------------------------
 * x(k+1) = A * x(k) + B * u(k)
 * Allocations done to avoid overwriting during add operation
 */
void quad_linear_model(gsl_vector* u, gsl_matrix* A, gsl_matrix* B, gsl_vector* x)
{

	gsl_vector *Bu = gsl_vector_calloc(SIZE_X);
	gsl_vector *Ax = gsl_vector_calloc(SIZE_X);

	gsl_blas_dgemv(CblasNoTrans, 1, B, u, 0, Bu);
	gsl_blas_dgemv(CblasNoTrans, 1, A, x, 0, Ax);

	gsl_vector_add(Ax, Bu);
	
	gsl_vector_memcpy(x, Ax);
	
}

