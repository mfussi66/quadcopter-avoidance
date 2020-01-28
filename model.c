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
/* 
 * Function: error rotation
 * ---------------------------
 * Rotates the error vector towards the body frame on
 * the yaw axis
 */
void rotate_error(double* e, double yaw)
{
double e_tmp[3] = {0.0};
	
	memcpy(e_tmp, e, sizeof(double) * 3);
	
	//printf("e_0:%f, e_1:%f\n", e[0], e[1]);
	
	e[0] = e_tmp[0] * cos(yaw) + e_tmp[1] * sin(yaw);
	e[1] = - e_tmp[0] * sin(yaw) + e_tmp[1] * cos(yaw);
	e[2] = e[2];
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
 * Gets the next waypoint in the vector
 */
void next_setpoint(double* sp, WPoint* wp, int wp_size, int* wp_flags)
{
	
WPoint xy_setpoint;

	if (wp_size >= MAX_WPOINTS) return;
	
	for(int i = 1; i <= wp_size; i++)
	{
		if(wp_flags[i] == 0)
		{
			//printf("WP %f %f \n", wp[i].x, wp[i].y);
			xy_setpoint.x = (wp[i].x - ENV_OFFSET_X) / ENV_SCALE;
			xy_setpoint.y = (ENV_OFFSET_Y - wp[i].y) / ENV_SCALE;
			break;
		}	
	}
	
	sp[0] = xy_setpoint.x;
	sp[1] = xy_setpoint.y;
}


/* 
 * Function: Position PID
 * ---------------------------
 * Implements a simple PD controller for position
 * control
 */
void pid_xyz_control(double* e, double* e_prev, double dt, double* u, double* vel_sp, double* p, double* d)
{
	// X velocity
	vel_sp[0] = e[0] * p[0]  + d[0] * (e[0] - e_prev[0]) / dt;
	// Y velocity
	vel_sp[1] = e[1] * p[1] + d[1] * (e[1] - e_prev[1]) / dt;

	// Vertical Thrust
	u[3] = e[2] * p[2] + d[2] * (e[2] - e_prev[2]) / dt;
}

/* 
 * Function: Position PID
 * ---------------------------
 * Implements a simple P controller for velocity control
 */
void pid_vel_control(double* e, double* e_prev, double dt, double* rp_sp, double* p, double* d)
{
	// roll
	rp_sp[0] = e[1] * p[6] + d[6] * (e[1] - e_prev[1]) / dt;
	// pitch
	rp_sp[1] = e[0] * p[7] + d[7] * (e[0] - e_prev[0]) / dt;
}


/* 
 * Function: Forces PD control
 * ---------------------------
 * Implements a PD control that gives the control 
 * outputs to the system: RPY torques and vertical thrust
 */
void pid_rpy_alt_control(double* e, double* e_prev, double dt, double* u, double* p, double* d)
{
	// tau roll
	u[0] = e[0] * p[3] + d[3] * (e[0] - e_prev[0]) / dt;
	// tau pitch
	u[1] = e[1] * p[4] + d[4] * (e[1] - e_prev[1]) / dt;
	// tau yaw
	u[2] = e[2] * p[5] + d[5] * (e[2] - e_prev[2]) / dt;
	
	// vertical thrust
	//u[3] = 0.0;
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

/* 
 * Function: Linearized system equation
 * ---------------------------
 * Implements the linearized quadcopter dynamic equations
 * Before position integration the step is rotated in the
 * world frame to accumulate correctly
 */
void lin_model(double* u, double* x, double dt)
{
	double X_old[SIZE_X] = {0.0};
	double X_new[SIZE_X] = {0.0};
	double xy_tmp[2] = {0.0};
	
	memcpy(X_old, x, sizeof(double) * SIZE_X);

	X_new[6] = X_old[6] + u[0] / Jxx;							// p
	X_new[0] = X_old[0] + X_new[6] * dt; 		// roll
	
	X_new[7] = X_old[7] + u[1] / Jyy;							// q
	X_new[1] = X_old[1] + X_new[7] * dt;		// pitch

	X_new[8] = X_old[8] + u[2] / Jzz;							// r
	X_new[2] = X_old[2] + X_new[8] * dt;		// yaw
	
	X_new[9] = X_old[9] + 9.81 * X_new[1];						// u
	X_new[3] =  X_new[9] * dt;					// x

	X_new[10] = X_old[10] + 9.81 * X_new[0];					// v
	X_new[4] =  X_new[10] * dt;				// y

	X_new[11] = X_old[11] + u[3] / M;							// w
	X_new[5] = X_old[5] + X_new[11] * dt;		// z
	
	
	xy_tmp[0] = X_new[3];
	xy_tmp[1] = X_new[4];
	X_new[3] = xy_tmp[0] * cos(X_new[2]) - xy_tmp[1] * sin(X_new[2]);
	X_new[4] = + xy_tmp[0] * sin(X_new[2]) + xy_tmp[1] * cos(X_new[2]);

	X_new[3] = X_old[3] + X_new[3];
	X_new[4] = X_old[4] + X_new[4];

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
		tr[i].x = BEAM_DMAX;
		tr[i].y = BEAM_DMAX;
		tr[i].z = BEAM_DMAX;
		tr[i].theta = 0.0;
		tr[i].d = BEAM_DMAX;
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

			tr[i].d = d;
			tr[i].x = trace_mt.x;
			tr[i].y = trace_mt.y;
			tr[i].theta = deg2rad((double)a) + yaw;
			
			if (getpixel(bmp, (int)trace_px.x, (int)trace_px.y) == makecol(0, 255, 0))
				break;

			d += BEAM_DSTEP;
		}
		//printf("theta %d: %f\n",i,tr[i].theta);
		//printf("%d a: %f (%.2lf, %.2lf)\n", i,rad2deg(atan2(tr[i].y, tr[i].x)), trace_px.x, trace_px.y);
		if(i < n) i++;
	}
	//printf("^^^^\n");
}

/* 
 * Function: Get the shortest beam
 * ---------------------------
 * Takes the array of beams of the laser reader and returns 
 * the shortest beam
 */
Trace get_shortest_beam(Trace* tr, double threshold)
{
Trace force_max;
double trace_min = BEAM_DMAX;
force_max.x = 0.0;
force_max.y = 0.0;
force_max.z = 0.0;

	for(int i = 0; i < N_BEAMS; i++)
	{
		if(tr[i].d < trace_min)
		{
			memcpy(&force_max, &tr[i], sizeof(Trace));
 			trace_min = tr[i].d;
		}
	}

	return force_max;
}

void compute_repulsive_force(Trace* tr, double* pose, double *rep_forces, double ampli_thr)
{

double yaw = pose[2];
double tr_ampli = 0.0;
double tr_angle = 0.0;
double rotated_vel[2] = {0.0};
double rep_force_body[3] = {0.0};

	//tr_ampli = sqrt(tr->x * tr->x + tr->y * tr->y);
	//tr_angle = atan2_safe(tr->y, tr->x);	
	tr_ampli = tr->d;
	tr_angle = tr->theta;
	
	if (tr_ampli > ampli_thr)
	{
		rep_force_body[0] = tr->x * cos(tr_angle) + tr->y * sin(tr_angle);
		rep_force_body[1] = - tr->x * sin(tr_angle) + tr->y * cos(tr_angle);
		rep_force_body[2] = tr->z;

		tr_ampli = sqrt(pow2(rep_force_body[0]) + pow2(rep_force_body[1]));
		
		rep_force_body[0] = rep_force_body[0] / tr_ampli; 
		rep_force_body[1] = rep_force_body[1] / tr_ampli; 
		
		rotated_vel[0] = pose[9] * cos(tr_angle) + pose[10] * sin(tr_angle);
		rotated_vel[1] = - pose[9] * sin(tr_angle) + pose[10] * cos(tr_angle);
		
		//printf("rfb_x: %f, rfb_y: %f\n",rep_force_body[0], rep_force_body[1]);
	}
	else
	{
		rep_force_body[0] = 0.0;
		rep_force_body[1] = 0.0;
		rep_force_body[2] = 0.0;
	}

	//printf("rv: (%f, %f) a: %f\n",rotated_vel[0], rotated_vel[1], rad2deg(tr_angle + M_PI));
	
	rep_forces[0] = - 1e-6 * rep_force_body[1] - 1e-6 * rotated_vel[1]; //tau_roll
	rep_forces[1] = - 1e-7 * rep_force_body[0] - 0 * rotated_vel[0]; //tau_pitch

	printf("rf: (%f, %f) a: %f\n",rep_forces[0] * 1e9, rep_forces[1] * 1e9, rad2deg(tr_angle));
	printf("---\n");
	
}

/* 
 * Function: Set avoidance mode
 * ---------------------------
 * Sets the appropriate obstacle avoidance mode according
 * to the angle and amplitude of the shortest beam wrt to 
 * the drone
 */
int set_avoid_mode(Trace* tr, double* pose, WPoint* target, int* turn_dir)
{
int i_nearest = 0;
double a_nearest = M_PI;
double direct = atan2_safe(target->y - pose[4], target->x - pose[3]);
double dist = sqrt(pow2(target->x - pose[3]) + pow2(target->y - pose[4]));
Trace min_trace;
double angle_diff = 0.0;
int mode = 0;

	min_trace = get_shortest_beam(tr, 1);
		
	// take the beam closest to the setpoint
	for(int i = 0; i < N_BEAMS; i++)
	{
		if(fabs(anglediff_safe(direct, tr[i].theta)) < fabs(a_nearest))
		{
			i_nearest = i;
			a_nearest = anglediff_safe(direct, tr[i].theta);
		}
	}
	
	angle_diff = min_trace.theta - pose[2];
	
	if (angle_diff < 0)
		*turn_dir = 1;
	else
		*turn_dir = -1;

	// Emergency avoidance mode
	if(min_trace.d <= 0.6)
	{
		return 1;
	}
	
	// If the the goal is closer than the shortest beam
	if(dist <= tr[i_nearest].d)
	{
		return 0;
	}
	
	// Sets the primary avoidance modes
	if(min_trace.d <= 1)
	{
		// if the obstacle if more or less in front of the quadc.
		if(fabs(angle_diff) <= deg2rad(30))
		{
			//if(get_uniform_num() >= 0.5) {*turn_dir = 1;}
			//else {*turn_dir = -1;}
			return 1;
		}
		// rotate 1
		if(fabs(angle_diff) > deg2rad(30))
			return 2;
	}

	return mode;
	
}

/* 
 * Function: Set velocity setpoints
 * ---------------------------
 * Sets the appropriate velocity setpoints to avoid 
 * obstacles according to the modes chosen
 */
void compute_avoid_sp(double* pose, int mode, double *vel_sp, double *yaw_sp, int turn_dir)
{
double vel_rot[2] = {0.0};

	if (mode == 2)
	{	
		//*yaw_sp = pose[2];
		vel_rot[1] = turn_dir * 0.5;
		vel_rot[0] = turn_dir * 0.5;
		vel_sp[0] = vel_rot[0] * cos(0) - vel_rot[1] * sin(0);
		vel_sp[1] = vel_rot[0] * sin(0) + vel_rot[1] * cos(0);
		
	}
	else if (mode == 3)
	{	
		//*yaw_sp = pose[2];
		vel_rot[1] = -0.3;
		vel_rot[0] = -0.3;
		vel_sp[0] = vel_rot[0] * cos(0) - vel_rot[1] * sin(0);
		vel_sp[1] = vel_rot[0] * sin(0) + vel_rot[1] * cos(0);		
	}
	else if(mode == 1)
	{
		*yaw_sp = pose[2];
		vel_rot[1] = turn_dir * 0.3;
		vel_rot[0] = 0.0;
		vel_sp[0] = vel_rot[0] * cos(0) - vel_rot[1] * sin(0);
		vel_sp[1] = vel_rot[0] * sin(0) + vel_rot[1] * cos(0);
	}
	
}

/*
int compute_avoid_sp(Trace* tr, double* pose, WPoint* target, double ampli_thr, double *vel_sp, double *yaw_sp)
{
double tr_ampli = tr->d;
double tr_angle = tr->theta;
double vel_rot[2] = {0.0};
double rot_dir = -1;
double sp_dir = atan2_safe(target->y - pose[4], target->x - pose[3]);

int mode = 0;

	if (tr_ampli > ampli_thr)
	{	
		if(tr_angle - pose[2] >= -M_PI/2 && tr_angle - pose[2] <= 0) 
			rot_dir = 1;
		else if(tr_angle - pose[2] <= M_PI/2 && tr_angle - pose[2] > 0)
			rot_dir = -1;

		vel_rot[1] = rot_dir * 0.3;		// velocity y-axis BODY (roll)
		
		// if the obstacle if more or less in front of the quadc.
		if(tr_angle - pose[2] >= -M_PI/7 && tr_angle - pose[2] <= M_PI/7)
		{
			printf("stopping pitch vel\n");
			*yaw_sp = pose[2];
			vel_rot[0] = 0.0;
			vel_sp[0] = vel_rot[0] * cos(0) - vel_rot[1] * sin(0);
			vel_sp[1] = vel_rot[0] * sin(0) + vel_rot[1] * cos(0);
			mode = 2;
		}
		else
		{
			vel_rot[0] = 0.1;
			vel_sp[0] = vel_rot[0] * cos(0) - vel_rot[1] * sin(0);
			vel_sp[1] = vel_rot[0] * sin(0) + vel_rot[1] * cos(0);
			mode = 1;
		}
	}
	else
	{

		vel_rot[0] = 0;	// velocity x-axis BODY
		vel_sp[1] = 0;	// velocity y-axis BODY
		mode = 0;
	}
	
	return mode;
	
}
*/



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

	// initialize valleys values
	for(int i = 0; i < size; i++)
	{
		valleys[i].start = 0;
		valleys[i].end = size - 1;
	}

	for(int i = 0; i < size; i++)
	{
		
		if (hist[i] < threshold && start_found == 0) //save start of valley
		{
			//printf("found valley start:%d h:%f\n", i, hist[i]);
			valleys[j].start = i;
			start_found = 1;
		}
		else if(hist[i] >= threshold && start_found == 1) // save end of valley
		{
			//printf("found valley end:%d h:%f\n ", i - 1, hist[i]);
			valleys[j].end = i - 1;
			start_found = 0;
			j++;
		}
		else if(i == size - 1 && start_found == 1) //save the end of valley at end
		{
			//printf("found valley end:%d h:%f\n ", i, hist[i]);
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
int candidate_valley = 0;
double a_nearest = M_PI;
double a_nearest_tmp = M_PI;
double theta_rel = 0.0;
double yaw_ref = 0.0;

	yaw_ref = atan2_safe(sp->y - pose[4],sp->x - pose[3]);
	dist[0]= sp->x - pose[3];
	dist[1]= sp->y - pose[4];

	for(int i = 0; i < v_size; i++)
	{
		i_start = v[i].start;
		a_start = anglediff_safe(yaw_ref, tr[i_start].theta);
		
		i_end = v[i].end;
		a_end = anglediff_safe(yaw_ref, tr[i_end].theta);

		printf("v%d: (%d, %d) (%f, %f)\n",i, i_start, i_end, rad2deg(a_start), rad2deg(a_end));
		
		if(fabs(a_end) < fabs(a_start))
		{
			i_nearest_tmp = i_end;
			a_nearest_tmp = a_end;
		}
		else
		{
			i_nearest_tmp = i_start;
			a_nearest_tmp = a_start;
		}
		
		if(a_nearest_tmp <= a_nearest)
		{
			candidate_valley = i;
			i_nearest = i_nearest_tmp;
			a_nearest = a_nearest_tmp;
		}
	}
	
	i_start = v[candidate_valley].start;
	i_end = v[candidate_valley].end;
	printf("%d (%d, %d)\n",candidate_valley, i_start, i_end);
		
	theta_rel = (tr[i_start].theta + tr[i_end].theta) / 2;
	
	printf("(%f - %f)\n",rad2deg(theta_rel), rad2deg(pose[2]));
	printf("---\n");
	
	return theta_rel;
	
}

/* 
 * Function: Check collision
 * ---------------------------
 * checks if the baricenter of the quadcopter is inside the 
 * environment or inside an obstalce
 */
int chk_collisions(double* pose, Obstacle* obs, int n_obs)
{
	double x = pose[3];
	double y = pose[4];
	
	Obstacle env;
	
	env.x1 = 0.0;
	env.y1 = 0.0;
	env.x2 = (WIDTH_SCREEN - 266) / ENV_SCALE;
	env.y2 = (HEIGHT_SCREEN - 5) / ENV_SCALE;
	
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
 * Function: Init controller gains
 * ---------------------------
 */
void init_gains(double* p, double* d, double* p_df, double* d_df)
{
	
	// proportional gains
	p[0] = 1.5e-1;	// x
	p[1] = 1.5e-1;	// y
	p[2] = 1e-4;	// z
	p[3] = 1e-3;	// roll
	p[4] = 1e-3;	// pitch
	p[5] = 1e-4;	// yaw
	p[6] = 7e-2;	// u
	p[7] = 7e-2;	// v
	p[8] = 1e-2;	// w
	
	// derivative gains
	d[0] = 0;		// x
	d[1] = 0;		// y
	d[2] = 1e-3;	// z
	d[3] = 1e-3;	// roll
	d[4] = 1e-3;	// pitch
	d[5] = 8e-3;	// yaw
	d[6] = 0;		// u
	d[7] = 0;		// v
	d[8] = 0;		// w	
	
	memcpy(p_df, p, sizeof(double) * SIZE_PID);
	memcpy(d_df, d, sizeof(double) * SIZE_PID);
}

/* 
 * Function: Modify controller gains
 * ---------------------------
 */
void adjust_gain(double *p, double *d, int mode, int updown)
{
int idx = mode - 2;
	
	if(idx < 0 || idx >= SIZE_PID) return;
	
	if(updown == 0)
	{
		p[idx] *= 2;
		//d[idx] *= 2;
	}
	else if(updown == 1)
	{
		p[idx] /= 2;
		//d[idx] /= 2;
	}
	
}

/* 
 * Function: Reset gains to default values
 * ---------------------------
 */
void reset_gains(double* p, double* d, double* p_df, double* d_df)
{
	memcpy(p, p_df, sizeof(double) * SIZE_PID);
	memcpy(d, d_df, sizeof(double) * SIZE_PID);
}


/* 
 * Helper Functions
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

double atan2_safe(double y, double x)
{
double r = atan2(y, x);

	if(r < 0) return (2 * M_PI + r);
	
	return r;
}

double anglediff_safe(double sp, double theta)
{
double res = sp - theta;

	if (res < M_PI && res > -M_PI)
	{
		res = sp - theta;
	}
	else if (res > M_PI)
	{
		res -= 2*M_PI;
	}
	else if (res <= -M_PI)
	{
		res += 2*M_PI;
	}
	
	return res;
	
}

double get_uniform_num()
{
	
	return  rand() / RAND_MAX;
	
}
