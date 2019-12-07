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
		
		printf("beam %d, x: %f, y: %f \n", i,tr[i].x,tr[i].y);
	}
	
}

void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double aperture)
{
	double angle = aperture / (5 - 1);
	int col = makecol(0, 255, 0);
	int d_min = 2;
	int d_max = 5;
	int d_step = 1;
	int obj_found;
	Trace temp;
	
	for (int a = 0; a < 5; a++)
	{
		obj_found = 0;
		
		for(int d = d_min; d < d_max; d += d_step)
		{
			temp.x = ENV_OFFSET_X + ENV_SCALE * (pose[3] + (double)d * cos(angle * (a + 1)));
			temp.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + (double)d * sin(angle * (a + 1)));
			temp.z = pose[5] - 0 * sin(angle * (a + 1));
			
			temp.x = temp.x;
			temp.y = temp.y;
			
			if (getpixel(bmp, (int)temp.x, (int)temp.y) == col)
			{
				printf("Beam %d found: x: %f y: %f \n", a, temp.x, temp.y);
				obj_found = 1;
				break;
			}
		}
		//printf("beam %d, x: %f, y: %f \n", a, temp.x,temp.y);
		tr[a].x = temp.x;
		tr[a].y = temp.y;
		tr[a].z = temp.z;
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
