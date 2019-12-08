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
	
	for(int i = 0; i < n; i++)
	{
		tr[i].x = 0.0;
		tr[i].y = 0.0;
		tr[i].z = 0.0;
		
		//printf("beam %d, x: %f, y: %f \n", i,tr[i].x,tr[i].y);
	}
	
}

void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double spread, double n)
{
	int angle = spread / (n);
	int col = makecol(0, 255, 0);
	int i = 0;
	int obj_found;
	Trace temp;
	
	for (int a = angle; a <= spread; a += angle)
	{
		obj_found = 0;
		
		for(int d = BEAM_DMIN; d <= BEAM_DMAX; d += BEAM_DSTEP)
		{	
			temp.x = ENV_OFFSET_X + ENV_SCALE * (pose[3] + d * sin(deg2rad(a)));
			temp.y = ENV_OFFSET_Y - OFFSET_LASER - ENV_SCALE * (pose[4] + d * cos(deg2rad(a)));
			temp.z = pose[5] - 0 * sin(deg2rad(a));
			
			//printf("d: %d, a: %d x: %f, y: %f \n", d, a, temp.x,temp.y);
			//printf("dist x: %lf y: %lf \n", d * cos(deg2rad(a)), d * sin(deg2rad(a)));
			
			if (getpixel(bmp, (int)temp.x, (int)temp.y) ==  COL_GREEN)
			{
				//printf("Beam %d found col %d: x: %f y: %f \n", i + 1, col, temp.x, temp.y);
				obj_found = 1;
				break;
			}
		}
		//printf("beam %d, a: %d x: %f, y: %f \n", i, a, temp.x,temp.y);
		if (i < n)
		{
			tr[i].x = (temp.x - ENV_OFFSET_X) / ENV_SCALE - pose[3];
			tr[i].y = (temp.y - ENV_OFFSET_Y + OFFSET_LASER) / (- ENV_SCALE) - pose[4];
			tr[i].z = temp.z;
			if (obj_found)
				printf("Beam %d: a: %d xd: %f yd: %f \n", i + 1, a, tr[i].x, tr[i].y);
			i++;
		}
	}
	//printf("<<<<<<\n");
}

double deg2rad(double n)
{
	return n * M_PI / 180;	
}


double rad2deg(double n)
{
	return n * 180 / M_PI;	
}
