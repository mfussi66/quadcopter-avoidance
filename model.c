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
void compute_error(Vector* setpoint, Vector* state, Vector *result)
{
	Vector* temp = gsl_vector_alloc_from_vector(setpoint, 0, SIZE_X, 1);
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
void dlqr_control(Vector* sp, Vector* x, Matrix* K, Vector* u)
{
	Vector* e = gsl_vector_calloc(SIZE_X);
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
void quad_linear_model(Vector* u, Matrix* A, Matrix* B, Vector* x)
{

	Vector *Bu = gsl_vector_calloc(SIZE_X);
	Vector *Ax = gsl_vector_calloc(SIZE_X);

	gsl_blas_dgemv(CblasNoTrans, 1, B, u, 0, Bu);
	gsl_blas_dgemv(CblasNoTrans, 1, A, x, 0, Ax);

	gsl_vector_add(Ax, Bu);
	
	gsl_vector_memcpy(x, Ax);
	
}

