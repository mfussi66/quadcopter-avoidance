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

/* 
 * Function: Laser scanner initializer
 * ---------------------------
 * Initializes the array of traces taking aperture,
 * number of beams and initial pose of the quadcopter
 */
void init_laser_scanner(Trace* tr, int n, double aperture, double* init_pose)
{
	double angle = aperture / (n - 1);
	
	for(int i = 1; i < n -1; i++)
	{
		tr[i].x = init_pose[3] + 100 * cos(angle * i);
		tr[i].y = init_pose[4] - 12 + 100 * sin(angle * i);
		tr[i].z = init_pose[5] + 100 * 0.0;
	}
	
}

void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double aperture)
{
	double angle = aperture / (5 - 1);
	double d;
	int obj_found;
	for (int a = 0; a < 5; a++)
	{
		obj_found = 0;
		d = 50;
		while (d < 100 || obj_found == 0)
		{
			
			tr[a].x = pose[3] + d * cos(angle * (a + 1));
			tr[a].y = pose[4] - d * sin(angle * (a + 1));
			tr[a].z = pose[5] - 0 * sin(angle * (a + 1));
		
			d +=5;
			
			if (getpixel(bmp, tr[a].x, tr[a].y) == makecol(0,255,0))
				obj_found = 1;
		}
		
	}
}

double deg2rad(double n)
{
	return n * M_PI / 180;	
}


double rad2deg(double n)
{
	return n * 180 / M_PI;	
}
