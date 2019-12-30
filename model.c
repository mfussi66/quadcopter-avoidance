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
 * Function: scalar absolut error computation
 * ---------------------------
 * Computes error between setpoint and measured state
 * allocates a new vector so that gsl_vector_sub() does not
 * overwrite the first argument
 */
double compute_error_scabs(Vector* v1, Vector* v2, int idx)
{
	return fabs(gsl_vector_get(v1, idx) - gsl_vector_get(v2, idx));
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

/* 
 * Function: Get distances from laser scanner
 * ---------------------------
 * Computes distances from obstacles simulating a laser scanner,
 * for each beam at a specific aperture looks for green pixels at a certain
 * increasing distance step
 */
void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double spread, double n)
{
	int angle = spread / (n);
	int col = makecol(0, 255, 0);
	int i = 0;
	int obj_found;
	double yaw = pose[2];
	Trace temp;
	
	for (int a = angle - 90; a <= spread - 90; a += angle)
	{
		obj_found = 0;
		
		for(int d = BEAM_DMIN; (d <= BEAM_DMAX) && (obj_found == 0); d += BEAM_DSTEP)
		{	
			temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3] + d * cos(deg2rad(a) + yaw));
			temp.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + d * sin(deg2rad(a) + yaw));
			temp.z = pose[5] - 0 * sin(deg2rad(a));
			
			//printf("d: %d, a: %d x: %f, y: %f \n", d, a, temp.x,temp.y);
			//printf("dist x: %lf y: %lf \n", d * cos(deg2rad(a)), d * sin(deg2rad(a)));
			
			if (getpixel(bmp, (int)temp.x, (int)temp.y) ==  COL_GREEN)
			{
				//printf("Beam %d found: x: %f y: %f \n", i, temp.x, temp.y);
				obj_found = 1;
			}
		}
		//printf("beam %d, a: %d x: %f, y: %f \n", i, a, temp.x,temp.y);
		if (i < n)
		{
			tr[i].x = (temp.x - ENV_OFFSET_X - OFFSET_LASER) / ENV_SCALE - pose[3];
			tr[i].y = (temp.y - ENV_OFFSET_Y) / (- ENV_SCALE) - pose[4];
			tr[i].z = temp.z;
// 			if (obj_found)
// 				printf("Beam %d: xd: %.1f yd: %.1f \n", i, tr[i].x, tr[i].y);
			i++;
		}
	}

}

void compute_repulsive_force(Trace* tr, int n, double* pose, double *rep_force_body)
{
	Trace force_sum;
	
	force_sum.x = 0.0;
	force_sum.y = 0.0;
	force_sum.z = 0.0;
	
	double ampli_sum = 0.0;
	double angle_sum = 0.0;
	double yaw = pose[2];
	double tr_ampli = 0.0;
	double tr_angle = 0.0;
	
	for(int i = 0; i < n; i++)
	{
		printf("%d: (x:%.2f, y:%.2f)\n",i, tr[i].x, tr[i].y);
		tr_angle = atan2(tr[i].y, tr[i].x);
		printf("   tr_angle %f\n", rad2deg(tr_angle));
		force_sum.x += BEAM_DMAX * cos(tr_angle) - tr[i].x;
		force_sum.y += BEAM_DMAX * sin(tr_angle) - tr[i].y;
		force_sum.z += 0.0;
	}
	
	tr_ampli = sqrt(force_sum.x * force_sum.x + force_sum.y * force_sum.y);
	tr_angle = atan2(force_sum.y, force_sum.x);

	rep_force_body[0] = force_sum.x * cos(yaw) - force_sum.y * sin(yaw);
	rep_force_body[1] = force_sum.y * sin(yaw) + force_sum.y * cos(yaw);
	rep_force_body[2] = force_sum.z;
	
	printf("(am:%.2f, an:%.2f)\n", tr_ampli, rad2deg(atan2(rep_force_body[1], rep_force_body[0])));
}

int chk_collisions(double* pose, Obstacle* obs, int n_obs)
{
	double x = pose[3];
	double y = pose[4];
	
	Obstacle env;
	
	env.x1 = 0.0;
	env.y1 = 0.0;
	env.x2 = 520 / ENV_SCALE;
	env.y2 = 590 / ENV_SCALE;
	
	int result = 0;

	//check collision with env boundaries
	if(x <= env.x1 || x >= env.x2 ||
		y <= env.y1 || y >= env.y2)
		result = 1;

	// check collision with obstacles
	for (int i = 0; i < n_obs; i++)
	{
		if(x >= obs[i].x1 && x <= obs[i].x2 &&
			y <= obs[i].y1 && y >= obs[i].y2)
			result = 1;
	}
	
	return result;
}


/* 
 * Function: Utility functions for angle conversion
 * ---------------------------
 */
double deg2rad(double n)
{
	return n * M_PI / 180;	
}


double rad2deg(double n)
{
	return n * 180 / M_PI;	
}
