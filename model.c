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
void compute_error(double* sp, double* x, double* e, double* e_old, int size)
{
	
	for (int i = 0; i < size; i++)
	{
		e_old[i] = e[i];
		e[i] = sp[i] - x[i];
	}
}

void rotate_error(double* e, double yaw)
{
double e_tmp[3] = {0.0};
	
	memcpy(e_tmp, e, sizeof(double) * 3);
	
	//printf("e_0:%f, e_1:%f\n", e[0], e[1]);
	
	e[0] = e_tmp[0] * cos(yaw) + e_tmp[1] * sin(yaw);
	e[1] = - e_tmp[0] * sin(yaw) + e_tmp[1] * cos(yaw);
	
	//printf("e0:%f, e1:%f\n", e[0], e[1]);
}

/* 
 * Function: compute euclidean distance
 * ---------------------------
 * 
 */
double compute_pos_dist(double* v1, double* v2)
{
	double x = v1[0] - v2[0];
	double y =v1[1] - v2[1];	
	//double z = gsl_vector_get(v1, 5) - gsl_vector_get(v2, 5);
	
	return sqrt(pow2(x) + pow2(y));
	//return sqrt(pow2(x) + pow2(y) + pow2(z));
	
}

/* 
 * Function: setpoint getter and computation
 * ---------------------------
 * 
 */
void compute_setpoint(double* sp, WPoint* wp, double* pose, double alt, int wp_size, int* wp_flags)
{
	
WPoint xy_setpoint;

	if (wp_size >= MAX_WPOINTS) return;
	
	for(int i = 1; i <= wp_size; i++)
	{
		if(wp_flags[i] == 0)
		{
			printf("WP %f %f \n", wp[i].x, wp[i].y);
			xy_setpoint.x = (wp[i].x - ENV_OFFSET_X) / ENV_SCALE;
			xy_setpoint.y = (ENV_OFFSET_Y - wp[i].y) / ENV_SCALE;
			break;
		}	
	}
	
	sp[0] = xy_setpoint.x;
	sp[1] = xy_setpoint.y;
	sp[2] = alt;
}


double compute_yaw_ref(double yaw, WPoint* sp, double yaw_laser, double gain)
{
	
double yaw_ref;
double a_r;

	a_r = yaw;

	yaw_ref = a_r + gain * (yaw_laser);
	
	//printf("yaw: %f rel:%f d:%f\n", rad2deg(a_r), rad2deg(yaw_laser), rad2deg(yaw_ref));
		
	return yaw_ref;
	
}

void pid_xy_control(double* e, double* e_prev, double* rp_sp, double yaw)
{
		
	// roll
	rp_sp[0] = e[1] * 1e-6 + 3e-4 * (e[1] - e_prev[1]) / (TP_MODEL / 1000.0);
	// pitch
	rp_sp[1] = e[0] * 1e-6  + 3e-4 * (e[0] - e_prev[0]) / (TP_MODEL / 1000.0);

}

void pid_rpy_alt_control(double* e, double* e_prev, double* u)
{
	u[0] = e[0] * 0.001 + 0.001 * (e[0] - e_prev[0]) / (TP_MODEL / 1000.0);
	u[1] = e[1] * 0.001 + 0.001 * (e[1] - e_prev[1]) / (TP_MODEL / 1000.0);
	u[2] = e[2] * 0.0001 + 0.004 * (e[2] - e_prev[2]) / (TP_MODEL / 1000.0);
	u[3] = 0.0;
	
//	printf("u0: %lf, u1: %lf\n", u[0], u[1] * 1e7);
}

/* 
 * Function: control action computation
 * ---------------------------
 * computes the control action as e = K*u
 * the K gain matrix is computed offline from the solution of the
 * Discrete Linear Quadratic Regulator problem
 * Implements u = 1 * K * e + 0 * u
 */
void dlqr_control(Vector* sp, Vector* x, Matrix* K, Vector* u, double yaw)
{
	Vector* e = gsl_vector_calloc(SIZE_X);
	double yaw_a = gsl_vector_get(x, 2);
	double e_body[3] = {0.0};
	
	gsl_vector_memcpy(e, sp);
	
 	gsl_vector_sub(e, x);
	
	e_body[0] = gsl_vector_get(e, 3) * cos(yaw_a) + gsl_vector_get(e, 4) * sin(yaw_a);
	e_body[1] = gsl_vector_get(e, 3) * (-sin(yaw_a)) + gsl_vector_get(e, 4) * cos(yaw_a);
	e_body[2] = gsl_vector_get(e, 5);
	
	printf("ex:%f ey:%f \n", e_body[0], e_body[1]);
	
// 	e_body[0] = gsl_vector_get(e, 3);
// 	e_body[1] = gsl_vector_get(e, 4);
	gsl_vector_set(e, 3, e_body[0]);
	gsl_vector_set(e, 4, e_body[1]);
	
	gsl_blas_dgemv(CblasNoTrans, 1, K, e, 0, u);
	
	for (int i = 0; i < 4; i++)
		printf("u%d %f\n",i, gsl_vector_get(u, i));	
	
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

void lin_model(double* u, double* x, double yaw_sp)
{
	double X_old[SIZE_X] = {0.0};
	double X_new[SIZE_X] = {0.0};
	double xy_tmp[2] = {0.0};
	
	memcpy(X_old, x, sizeof(double) * SIZE_X);

	X_new[6] = X_old[6] + u[0] / Jxx;							// p
	X_new[0] = X_old[0] + X_new[6] * (TP_MODEL / 1000.0); 		// roll
	
	X_new[7] = X_old[7] + u[1] / Jyy;							// q
	X_new[1] = X_old[1] + X_new[7] * (TP_MODEL / 1000.0);		// pitch

	X_new[8] = X_old[8] + u[2] / Jzz;							// r
	X_new[2] = X_old[2] + X_new[8] * (TP_MODEL / 1000.0);		// yaw
	
	X_new[9] = X_old[9] + 9.81 * X_new[1];						// u
	X_new[3] =  X_new[9] * (TP_MODEL / 1000.0);		// x

	X_new[10] = X_old[10] + 9.81 * X_new[0];					// v
	X_new[4] =  X_new[10] * (TP_MODEL / 1000.0);		// y

	X_new[11] = X_old[11] + u[3] / M;							// w
	X_new[5] = X_old[5] + X_new[11] * (TP_MODEL / 1000.0);		// z
	
	
	xy_tmp[0] = X_new[3];
	xy_tmp[1] = X_new[4];
	X_new[3] = xy_tmp[0] * cos(X_new[2]) - xy_tmp[1] * sin(X_new[2]);
	X_new[4] = + xy_tmp[0] * sin(X_new[2]) + xy_tmp[1] * cos(X_new[2]);

	X_new[3] = X_old[3] + X_new[3];
	X_new[4] = X_old[4] + X_new[4];

//	printf("p: (%f, %f) \n", xy_tmp[0], xy_tmp[1]);
	memcpy(x, X_new, sizeof(double) * SIZE_X);
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
		tr[i].x = BEAM_DMIN;
		tr[i].y = BEAM_DMIN;
		tr[i].z = BEAM_DMIN;
		tr[i].theta = 0.0;
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
	int i = 0;
	float d = BEAM_DMIN;
	double yaw = pose[2];
	Trace trace_mt = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};
	Trace trace_px = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};
	
	for (int a = angle - 90; a <= spread - 90; a += angle)
	{	
		d = BEAM_DMIN;
		while(d <= BEAM_DMAX)
		{
			trace_mt.x = d * cos(deg2rad((double)a) + yaw);
			trace_mt.y = d * sin(deg2rad((double)a) + yaw);

			trace_px.x = ENV_OFFSET_X + ENV_SCALE * (pose[3] + trace_mt.x);
			trace_px.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + trace_mt.y);

			tr[i].x = trace_mt.x;
			tr[i].y = trace_mt.y;
			tr[i].theta = deg2rad((double)a) + yaw;

			if (getpixel(bmp, (int)trace_px.x, (int)trace_px.y) == makecol(0, 255, 0))
				break;

			d += BEAM_DSTEP;
		}
		//printf("%d a: %f (%.2lf, %.2lf)\n", i,rad2deg(atan2(tr[i].y, tr[i].x)), trace_px.x, trace_px.y);
		if(i < n) i++;
	}
	//printf("----\n");
}

void compute_repulsive_force(Trace* tr, int n, double* pose, double *rep_force_body)
{
	Trace force_sum;
	double trace_min = 1;
	double trace_tmp;
	
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
		
// 		if(sqrt(pow2(tr[i].x) + pow2(tr[i].y)) < 1)
// 		{
// 			tr_angle = atan2(tr[i].y, tr[i].x);
// 			force_sum.x += BEAM_DMAX * cos(tr_angle) - tr[i].x;
// 			force_sum.y += BEAM_DMAX * sin(tr_angle) - tr[i].y;
// 			force_sum.z += 0.0;
// 		}
			trace_tmp = sqrt(pow2(tr[i].x) + pow2(tr[i].y));
			if(trace_tmp < trace_min)
			{
				trace_min = trace_tmp;
				tr_angle = tr[i].theta;
				force_sum.x = BEAM_DMAX * cos(tr_angle) - tr[i].x;
				force_sum.y = BEAM_DMAX * sin(tr_angle) - tr[i].y;
				force_sum.z = 0.0; 
			}
	}
	
	tr_ampli = sqrt(force_sum.x * force_sum.x + force_sum.y * force_sum.y);
	tr_angle = atan2(force_sum.y, force_sum.x);
	
	if (tr_ampli > 0)
	{
		rep_force_body[0] = force_sum.x * cos(yaw) + force_sum.y * sin(yaw);
		rep_force_body[1] = - force_sum.x * sin(yaw) + force_sum.y * cos(yaw);
		rep_force_body[2] = force_sum.z;

// 		rep_force_body[0] = force_sum.x;
// 		rep_force_body[1] = force_sum.y;
// 		rep_force_body[2] = force_sum.z;

		tr_ampli = sqrt(pow2(rep_force_body[0]) + pow2(rep_force_body[1]));
		
		rep_force_body[0] = rep_force_body[0] / tr_ampli; 
		rep_force_body[1] = rep_force_body[1] / tr_ampli; 
		printf("rfb_x: %f, rfb_y: %f\n",rep_force_body[0], rep_force_body[1]);
	}
	else
	{
		rep_force_body[0] = 0.0;
		rep_force_body[1] = 0.0;
		rep_force_body[2] = 0.0;
	}	
}


void compute_histogram(Trace* tr, int n, double* hist)
{
double tr_angle = 0.0;

	for(int i = 0; i < n; i++)
	{
		tr_angle = tr[i].theta;
		
		hist[i] = sqrt(pow2(BEAM_DMAX * cos(tr_angle) - tr[i].x) + 
					   pow2(BEAM_DMAX * sin(tr_angle) - tr[i].y));
		//printf("%d: (%f, %f) %f\n", i,tr[i].x,tr[i].y, hist[i]);
	}
}

void find_valleys(double* hist, Valley* valleys, int size, int* v_size, int threshold)
{
	
int start_found = 0;
int j = 0;

	for(int i = 0; i < size; i++)
	{
		valleys[i].start = 0;
		valleys[i].end = size - 1;
	}

	for(int i = 0; i < size; i++)
	{
		if (hist[i] < threshold && start_found == 0)
		{
			valleys[j].start = i;
			start_found = 1;
		}
		else if(hist[i] >= threshold && start_found == 1)
		{
			valleys[j].end = i;
			start_found = 0;
			j++;
		}
	}
	
	*v_size = j;
}

double compute_heading(Trace* tr, Valley* v, int v_size, double* pose, WPoint* sp)
{
int j = 0;
double dist[2];
double a_start;
int i_start;
double a_end;
int i_end;
int i_nearest = 0;
int i_nearest_tmp = 0;
double a_nearest = M_PI;
double a_nearest_tmp = M_PI;
double theta_rel = 0.0;

	dist[0]= sp->x - pose[3];
	dist[1]= sp->y - pose[4];

	for(int i = 0; i < v_size; i++)
	{
		i_start = v[i].start;
		a_start = tr[i_start].theta - pose[2];
		//a_start = atan2(tr[j].y - dist[1], tr[j].x - dist[0]);
		
		i_end = v[i].end;
		a_end = tr[i_end].theta - pose[2];
		//a_end = atan2(tr[j].y - dist[1], tr[j].x - dist[0]);
		
		if(fabs(a_end) > fabs(a_start))
		{
			i_nearest = i_end;
			a_nearest_tmp = a_end;
		}
		else
		{
			i_nearest = i_start;
			a_nearest_tmp = a_start;
		}
		
		if(a_nearest_tmp <= a_nearest)
		{
			i_nearest = i_nearest_tmp;
			a_nearest = a_nearest_tmp;
		}
	}
	
	i_start = v[i_nearest].start;
	i_end = v[i_nearest].end;
	
	theta_rel = (atan2(tr[i_start].y, tr[i_start].x) + atan2(tr[i_end].y, tr[i_end].x)) / 2;
	
	printf("(%f, %f)\n", rad2deg(atan2(tr[i_start].y, tr[i_start].x)), rad2deg(atan2(tr[i_end].y, tr[i_end].x)));
	printf("(%d, %d) %f\n", i_start, i_end, rad2deg(theta_rel));
	printf("---\n");
	
	return theta_rel;
	
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

double pow2(double n)
{
	return n * n;
}
