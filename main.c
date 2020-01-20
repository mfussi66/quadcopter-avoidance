/* --- REAL TIME MAIN --- */

#include "task.h"
#include "model.h"
#include "graphics.h"
#include "customdata.h"

/* global vars */

double arr_state[SIZE_X] = {0.0};
double arr_old_state[SIZE_X] = {0.0};
double arr_forces[SIZE_U] = {0.0}; 
double arr_repulsive_forces[SIZE_U] = {0.0};
double arr_error[SIZE_X] = {0.0};

Trace arr_traces[N_BEAMS];
Trace arr_old_traces[N_BEAMS];

WPoint waypoints[5] = {{-9999}, {-9999}};

Obstacle arr_obstacles[OBS_NUM];

double altitude_sp = 10.0;
float yawref = 0.0;

int collision = 0;
int gfx_initialized = 0;
int key_initialized = 0;
int gfx_enabled = 0;
int key_enabled = 0;
int key_mode = 0;
int waypoints_num = -1;

int QR_changed = 0;
double Q_coeff = 1;
double R_coeff = 1;

double Q_coeff_old = 1;
double R_coeff_old = 1;

int tp_changed = 0;
int selected_thread = 99;
int sel_thread_old_tp = 0;
int thread_periods[THREAD_MAX_NUM] = {0};

int start_sim = 0;
int end_sim = 0;

int Lmouse_clicked = 0;
int Rmouse_clicked = 0;
WPoint Rmouse;
WPoint Lmouse;

BITMAP* buffer_gfx;

char scan;

/* Semaphores */

pthread_mutex_t mux_forces;
pthread_mutex_t mux_rep_forces;
pthread_mutex_t mux_state;
pthread_mutex_t mux_gfx;
pthread_mutex_t mux_points;
pthread_mutex_t mux_laser;

pthread_mutexattr_t muxattr;

/* tasks  declaration */
void* lqr_task (void* arg);
void* lin_model_task (void* arg);
void* key_task (void* arg);
void* gfx_task (void* arg);
void* plt_task (void* arg);
void* laser_task (void* arg);
void* point_task (void* arg);
void* gains_task (void* arg);

/* main thread */

int main (int argc, char **argv) 
{

struct sched_param sched_mod, sched_gfx, sched_lqr, sched_pnt;
struct sched_param sched_plt, sched_key, sched_lsr, sched_gains;

struct task_par tp_mod, tp_gfx, tp_lqr, tp_plt;
struct task_par tp_key, tp_lsr, tp_pnt, tp_gains;

pthread_attr_t attr_mod, attr_gfx, attr_lqr, attr_plt;
pthread_attr_t attr_key, attr_lsr, attr_pnt, attr_gains;

pthread_t tid_mod, tid_gfx, tid_lqr, tid_plt; 
pthread_t tid_key, tid_lsr, tid_pnt, tid_gains;

int ret = 0;

	mutex_create (mux_forces, muxattr, 0, 100);
	mutex_create (mux_rep_forces, muxattr, 0, 100);
	mutex_create (mux_state, muxattr, 0, 100);
	mutex_create (mux_points, muxattr, 0, 100);
	mutex_create (mux_gfx, muxattr, 0, 100);
	mutex_create (mux_laser, muxattr, 0, 100);
	
	pthread_mutexattr_destroy (&muxattr);
	
	//Create Graphics Thread
	start_allegro();
	
	set_task_params(&tp_gfx, 1, TP_GFX, TP_GFX, 7);
	ret = thread_create (&tp_gfx, &sched_gfx, attr_gfx, &tid_gfx, gfx_task);
	thread_periods[0] = TP_GFX;

	//Create Keyboard Thread
	set_task_params(&tp_key, 2, TP_KEY, TP_KEY, 6);
	ret = thread_create (&tp_key, &sched_key, attr_key, &tid_key, key_task);
	thread_periods[1] = TP_KEY;

	//Create Waypoint selection Thread
	set_task_params(&tp_pnt, 7, TP_POINT, TP_POINT, 2);
	ret = thread_create (&tp_pnt, &sched_pnt, attr_pnt, &tid_pnt, point_task);
	thread_periods[2] = TP_POINT;

	//Create Dynamic Model Thread
	set_task_params(&tp_mod, 3, TP_MODEL, TP_MODEL, 20);
	ret = thread_create (&tp_mod, &sched_mod, attr_mod, &tid_mod, lin_model_task);
	thread_periods[3] = TP_MODEL;

	//Create LQR Thread
	set_task_params(&tp_lqr, 4, TP_LQR, TP_LQR, 19);
	ret = thread_create (&tp_lqr, &sched_lqr, attr_lqr, &tid_lqr, lqr_task);
	thread_periods[4] = TP_LQR;	

	//Create Laser scan Thread
	set_task_params(&tp_lsr, 6, TP_LSR, TP_LSR, 16);
	ret = thread_create (&tp_lsr, &sched_lsr, attr_lsr, &tid_lsr, laser_task);
	thread_periods[5] = TP_LSR;

	//Create Plotting Thread
	set_task_params(&tp_plt, 5, TP_PLOTS, TP_PLOTS, 5);
	ret = thread_create (&tp_plt, &sched_plt, attr_plt, &tid_plt, plt_task);
	thread_periods[6] = TP_PLOTS;

	//Create Gains Computation Thread
	set_task_params(&tp_gains, 8, TP_GAINS, TP_GAINS, 10);
	ret = thread_create (&tp_gains, &sched_gains, attr_gains, &tid_gains, gains_task);
	thread_periods[7] = TP_GAINS;
	
	pthread_join (tid_pnt, NULL);
	pthread_join (tid_gains, NULL);
	pthread_join (tid_plt, NULL);
	pthread_join (tid_lsr, NULL);
	pthread_join (tid_lqr, NULL);
	pthread_join (tid_mod, NULL);
	pthread_join (tid_key, NULL);
	pthread_join (tid_gfx, NULL);

	close_allegro();

	pthread_mutex_destroy(&mux_points);
	pthread_mutex_destroy(&mux_forces);
	pthread_mutex_destroy(&mux_state);
	pthread_mutex_destroy(&mux_laser);
	pthread_mutex_destroy(&mux_gfx);
	pthread_mutex_destroy(&mux_rep_forces);

	return 0;

}


void* lin_model_task(void* arg)
{
struct task_par* tp;

	tp = (struct task_par *)arg;
	set_period (tp);

	gsl_vector* lqr_forces = gsl_vector_calloc(SIZE_U);
	gsl_vector* rep_forces = gsl_vector_calloc(SIZE_U);
	gsl_vector* forces = gsl_vector_calloc(SIZE_U);
	gsl_vector* state = gsl_vector_calloc(SIZE_X);
	gsl_matrix* A = gsl_matrix_calloc(SIZE_X, SIZE_X);
	gsl_matrix* B = gsl_matrix_calloc(SIZE_X, SIZE_U);
	
	do{

		if (deadline_miss (tp))
			printf ("DEADLINE MISS: model_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 3, &tp_changed);
        
	}while(!start_sim  && !end_sim);
	
	if(start_sim)
	{
		// Set initial state from waypoints array	
		gsl_vector_set(state, 3, (waypoints[0].x - ENV_OFFSET_X) / ENV_SCALE);
		gsl_vector_set(state, 4, (ENV_OFFSET_Y - waypoints[0].y) / ENV_SCALE);
		gsl_vector_set(state, 5, 0);
		memcpy(arr_state, state->data, sizeof(double) * SIZE_X);
		
		read_matrix_file("A.bin", A);
		read_matrix_file("B.bin", B);
		
		printf("Dynamic model task started\n");
	}
	
    do{
		
		// Lock forces to store in internal variable
		pthread_mutex_lock (&mux_forces);
		pthread_mutex_lock (&mux_rep_forces);
	
		memcpy(lqr_forces->data, arr_forces, sizeof(double) * SIZE_U);
		memcpy(rep_forces->data, arr_repulsive_forces, sizeof(double) * SIZE_U);
		
// 		printf("a_f (%.2f, %.2f, %.2f)\n", arr_forces[0], arr_forces[1], arr_forces[2]);
// 		printf("r_f (%.2f, %.2f, %.2f)\n", arr_repulsive_forces[0], arr_repulsive_forces[1], arr_repulsive_forces[2]);
// 		printf("---\n");
		
		gsl_vector_memcpy(forces, lqr_forces);
		gsl_vector_add(forces, rep_forces);
		
		pthread_mutex_unlock (&mux_forces);		
		pthread_mutex_unlock(&mux_rep_forces);
        
		// Lock state and compute new state from transition function
        pthread_mutex_lock (&mux_state);
        
		memcpy(arr_old_state, arr_state, sizeof(double) * SIZE_X);
		
		quad_linear_model(forces, A, B, state);

		memcpy(arr_state, state->data, sizeof(double) * SIZE_X);

		collision = chk_collisions(arr_state, arr_obstacles, OBS_NUM);
		if (collision ==1) 
			printf("Collision detected!!\n");
		
		pthread_mutex_unlock (&mux_state);
		
		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: model_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 3, &tp_changed);

	}while(!end_sim && collision == 0);
	
	// free memory allocated for GSL variables
	gsl_vector_free(forces);
	gsl_vector_free(state);
	gsl_matrix_free(A);
	gsl_matrix_free(B);

    printf("model task closed\n");
    
	pthread_exit(0);

}

/*
 * Task
 * -----------------
 * Used for computing error and
 * control action
 */
void* lqr_task(void* arg)
{

struct task_par* tp;

gsl_vector* state = gsl_vector_calloc(SIZE_X);
gsl_vector* forces = gsl_vector_calloc(SIZE_U);
gsl_vector* setpoint = gsl_vector_calloc(SIZE_X);
//gsl_vector* altitude_sp = gsl_vector_calloc(SIZE_X);
gsl_vector* yaw_sp = gsl_vector_calloc(SIZE_X);
gsl_matrix* K = gsl_matrix_calloc(SIZE_U, SIZE_X);
gsl_vector_view state_view;

int waypoint_flags[MAX_WPOINTS] = {0};
int waypoint_idx = 1;
double dist = 0.0;
	
	tp = (struct task_par *)arg;
	
	set_period (tp);

	read_matrix_file("K.bin", K);
	
	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lqr_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 4, &tp_changed);
		
	}while(!start_sim && !end_sim);
    
	if(start_sim)
	{
		printf("LQR Controller task started\n");
		
		pthread_mutex_lock (&mux_state);
		state_view = gsl_vector_view_array(arr_state, SIZE_X);
		compute_setpoint(setpoint, waypoints, altitude_sp, &state_view.vector, waypoints_num, waypoint_flags);
		pthread_mutex_unlock (&mux_state);
	}
	
	do{
		
		pthread_mutex_lock (&mux_state);
		state_view = gsl_vector_view_array(arr_state, SIZE_X);

		dist = compute_pos_dist(setpoint, &state_view.vector);
		if(dist <= 0.3 && waypoint_idx < waypoints_num)
		{
			printf("Waypoint %d reached\n", waypoint_idx);
			waypoint_flags[waypoint_idx] = 1;
			waypoint_idx++;
			compute_setpoint(setpoint, waypoints, altitude_sp, &state_view.vector, waypoints_num, waypoint_flags);
		}
		
		state_view = gsl_vector_view_array(arr_state, SIZE_X);
		dlqr_control(setpoint, &state_view.vector, K, forces);
		pthread_mutex_unlock (&mux_state);
		
		pthread_mutex_lock (&mux_forces);
		memcpy(arr_forces, forces->data, sizeof(double) * SIZE_U);
		
// 		printf("tau_r %f, tau_p %f, tau_y: %f, f_t %f\n", 
// 			   arr_state[9], arr_state[10], arr_state[11], arr_forces[3]);
// 		
		pthread_mutex_unlock (&mux_forces);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lqr_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 4, &tp_changed);
		
	}while(!end_sim  && collision == 0);

	// free memory allocated for GSL variables
	gsl_vector_free(state);
	gsl_vector_free(forces);
	gsl_vector_free(setpoint);
	gsl_matrix_free(K);
    
    printf("lqr task closed\n");
    
	pthread_exit(0);
	
}

void* laser_task(void* arg)
{
struct task_par* tp;

double aperture;
int n_traces;
Trace laser_traces[N_BEAMS];
Trace old_traces[N_BEAMS];
double initialpose[6] = {0.0};
double old_pose[6] = {0.0};
double pose[6] = {0.0};
double rep_forces_xyz[3] = {0.0};
double rep_forces[SIZE_U] = {0.0};
double rep_force_angle = 0.0;
double rep_force_ampli = 0.0;
	
	tp = (struct task_par *)arg;
	
	aperture = 150;
	n_traces = N_BEAMS;
	
	set_period (tp);
		
	init_laser_scanner(laser_traces, N_BEAMS, aperture, initialpose);
	init_laser_scanner(old_traces, N_BEAMS, aperture, initialpose);
	
	do{

		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lsr_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 5, &tp_changed);

	}while(!start_sim && !end_sim);
    
    printf("Laser Scanner task started\n");
    
	do{
		
		pthread_mutex_lock(&mux_state);
		memcpy(old_pose, pose, sizeof(double) * SIZE_Y);
		memcpy(pose, arr_state, sizeof(double) * SIZE_Y);
		pthread_mutex_unlock(&mux_state);
		
		memcpy(old_traces, laser_traces, sizeof(Trace) * N_BEAMS);
		
		pthread_mutex_lock(&mux_gfx);
		get_laser_distances(buffer_gfx, laser_traces, pose, aperture, N_BEAMS);
		pthread_mutex_unlock(&mux_gfx);

		pthread_mutex_lock(&mux_rep_forces);
		compute_repulsive_force(laser_traces, N_BEAMS, pose, rep_forces_xyz);
		
		rep_forces[0] = 0.004 * rep_forces_xyz[1]; //tau_roll
		rep_forces[1] = 0.0005 * rep_forces_xyz[0]; //tau_pitch
		
		memcpy(arr_repulsive_forces, rep_forces, sizeof(double) * SIZE_U);
		pthread_mutex_unlock(&mux_rep_forces);
/*
  		printf("rep force (x:%f, y:%f)\n", rep_forces[0], rep_forces[1]);
 		printf("---\n");*/
 		
		pthread_mutex_lock(&mux_gfx);
		//draw_laser_points(buffer_gfx, old_traces, laser_traces, old_pose, pose);
		draw_laser_traces(buffer_gfx, old_traces, laser_traces, old_pose, pose);
		pthread_mutex_unlock(&mux_gfx);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lsr_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 5, &tp_changed);
		
	}while(!end_sim && collision == 0);

    printf("Laser Scanner task closed\n");
    
	pthread_exit(0);
	
}

/*
 * Task: Main graphics thread
 * -----------------
 * Used for drawing model
 * 
 */
void* gfx_task(void* arg)
{
	
struct task_par* tp;

int gui_color =  makecol(0, 255, 0);
int ret_obs = 0;
double old_pose[SIZE_Y] = {0.0};
double curr_pose[SIZE_Y] = {0.0};
WPoint curr_waypoints[MAX_WPOINTS];
WPoint old_waypoints[MAX_WPOINTS];

BITMAP* quad;
BITMAP* quad_bg;
BITMAP* expl;

	tp = (struct task_par *)arg;
	
	set_period (tp);
	
	for(int i = 0; i < MAX_WPOINTS; i++)
	{
		curr_waypoints[i].x= -9999;
		curr_waypoints[i].y = -9999;
	}

	buffer_gfx = create_bitmap(SCREEN_W, SCREEN_H);
	
	clear_to_color(buffer_gfx, 0);

	build_gui(buffer_gfx, font, COL_GREEN);
	
	ret_obs = gen_obstacles(arr_obstacles, OBS_NUM);
	
	if (ret_obs == -1)
		printf("Impossible to load obstacles!\n");
		
	quad = load_bmp("bmp/quad.bmp", NULL);
	quad_bg = load_bmp("bmp/black.bmp", NULL);
	expl = load_bmp("bmp/expl.bmp", NULL);

	if(quad == NULL)
		printf("Drone sprite not found!\n");
	
	gfx_enabled = 1;
	
	show_mouse(buffer_gfx);
	
	do{
		scare_mouse();
		draw_obstacles(buffer_gfx, arr_obstacles, OBS_NUM, COL_GREEN);
		
		pthread_mutex_lock(&mux_points);
		memcpy(old_waypoints, curr_waypoints, sizeof(WPoint) * MAX_WPOINTS);
		memcpy(curr_waypoints, waypoints, sizeof(WPoint) * MAX_WPOINTS);
		draw_waypoints(buffer_gfx, old_waypoints, curr_waypoints, waypoints_num);
		pthread_mutex_unlock(&mux_points);

		draw_periods(buffer_gfx, thread_periods, THREAD_MAX_NUM, selected_thread);
		
		pthread_mutex_lock(&mux_gfx);		
		blit(buffer_gfx,screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);

		pthread_mutex_unlock(&mux_gfx);
		unscare_mouse();		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: gfx_task()\n");

		wait_for_period (tp);
		
		eval_period(tp, thread_periods, &selected_thread, 0, &tp_changed);
		
	}while(!start_sim && !end_sim);
	
	show_mouse(NULL);
	
	do{
		pthread_mutex_lock(&mux_state);		
		memcpy(old_pose, curr_pose, sizeof(double) * SIZE_Y);
		memcpy(curr_pose, arr_state, sizeof(double) * SIZE_Y);
		pthread_mutex_unlock(&mux_state);
	
		build_gui(buffer_gfx, font, COL_GREEN);
		draw_obstacles(buffer_gfx, arr_obstacles, OBS_NUM, COL_GREEN);
		
		draw_waypoints(buffer_gfx, curr_waypoints, old_waypoints, waypoints_num);
		
		draw_periods(buffer_gfx, thread_periods, THREAD_MAX_NUM, selected_thread);
		
		if (collision)
			draw_quad(buffer_gfx, expl, quad_bg, old_pose, curr_pose);
		else if (quad != NULL)
			draw_quad(buffer_gfx, quad, quad_bg, old_pose, curr_pose);
		else
			draw_pose(buffer_gfx, old_pose, curr_pose);
			
        pthread_mutex_lock(&mux_gfx);
		blit(buffer_gfx,screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
		pthread_mutex_unlock(&mux_gfx);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: gfx_task()\n");
		
		wait_for_period (tp);
		
		eval_period(tp, thread_periods, &selected_thread, 0, &tp_changed);
        
	}while(!end_sim);
	
	printf("Closing graphics task\n");

	clear_to_color(buffer_gfx, 0);
	
	draw_exit_screen(buffer_gfx, gui_color);
	
	blit(buffer_gfx,screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
	
	usleep(2e6);

	gfx_enabled = 0;
	
	pthread_exit(0);
}

/*
 * Task
 * -----------------
 * Used for plotting
 * 
 */
void* plt_task(void* arg)
{
	
struct task_par* tp;

double plt_buf_Roll[PLT_DATA_SIZE] = {0.0};
double plt_buf_Pitch[PLT_DATA_SIZE] = {0.0};
double plt_buf_Yaw[PLT_DATA_SIZE] = {0.0};
double plt_buf_X[PLT_DATA_SIZE] = {0.0};
double plt_buf_Y[PLT_DATA_SIZE] = {0.0};
double plt_buf_Z[PLT_DATA_SIZE] = {0.0};

double new_pose[SIZE_Y] = {0.0};

	tp = (struct task_par *)arg;
	
	set_period (tp);
	
	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: plt_task()\n");
		
		wait_for_period (tp);
		
		eval_period(tp, thread_periods, &selected_thread, 6, &tp_changed);
		
	}while(!start_sim && !end_sim);
    
    printf("Plotting task started\n");

	do{
			
		pthread_mutex_lock(&mux_state);
        
		memcpy(new_pose, arr_state, sizeof(double) * SIZE_Y);
	
		pthread_mutex_unlock(&mux_state);
        
		shift_and_append(plt_buf_X, PLT_DATA_SIZE, new_pose[3]);
		shift_and_append(plt_buf_Z, PLT_DATA_SIZE, new_pose[5]);
		shift_and_append(plt_buf_Y, PLT_DATA_SIZE, new_pose[4]);
		
		shift_and_append(plt_buf_Roll, PLT_DATA_SIZE, new_pose[0]);
		shift_and_append(plt_buf_Pitch, PLT_DATA_SIZE, new_pose[1]);
		shift_and_append(plt_buf_Yaw, PLT_DATA_SIZE, new_pose[2]);
    
		pthread_mutex_lock(&mux_gfx);
				
		rectfill(buffer_gfx, 696, 286, PLT_XPOS_XCOORD - 1, PLT_XPOS_YCOORD - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 696, 391, PLT_YPOS_XCOORD - 1, PLT_YPOS_YCOORD - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 696, 496, PLT_ZPOS_XCOORD - 1, PLT_ZPOS_YCOORD - 1, makecol(0,0,0));
		
		rectfill(buffer_gfx, 580 + 1, 285 + 1, 680 - 1, 385 - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 580 + 1, 390 + 1, 680 - 1, 490 - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 580 + 1, 495 + 1, 680 - 1, 595 - 1, makecol(0,0,0));
		
		update_plot(buffer_gfx, plt_buf_Roll, PLT_XPOS_XCOORD - 115, PLT_XPOS_YCOORD, 25);		
		update_plot(buffer_gfx, plt_buf_Pitch, PLT_YPOS_XCOORD - 115, PLT_YPOS_YCOORD, 25);
		update_plot(buffer_gfx, plt_buf_Yaw, PLT_ZPOS_XCOORD - 115, PLT_ZPOS_YCOORD, 10);
		
		update_plot(buffer_gfx, plt_buf_X, PLT_XPOS_XCOORD, PLT_XPOS_YCOORD, PLT_SCALE_Y);		
		update_plot(buffer_gfx, plt_buf_Y, PLT_YPOS_XCOORD, PLT_YPOS_YCOORD, PLT_SCALE_Y);
		update_plot(buffer_gfx, plt_buf_Z, PLT_ZPOS_XCOORD, PLT_ZPOS_YCOORD, PLT_SCALE_Y);
		
		pthread_mutex_unlock(&mux_gfx);

		if (deadline_miss (tp))
			printf ("DEADLINE MISS: plt_task()\n");

		wait_for_period(tp);

		eval_period(tp, thread_periods, &selected_thread, 6, &tp_changed);

	}while(!end_sim);
	
	pthread_exit(0);	
}


/*
 * Task
 * -----------------
 * Used to select waypoints
 * 
 */
void* point_task(void* arg)
{
struct task_par* tp;

	tp = (struct task_par *)arg;

	set_period (tp);
	
	for(int i = 0; i < MAX_WPOINTS; i++)
	{
		waypoints[i].x = -9999;
		waypoints[i].y = -9999;
	}
	
	printf("Waypoint task started\n");

	printf("Click on starting point\n");
	
	do{
		
		if(Lmouse_clicked)
		{
			//printf("L clicked\n");
			pthread_mutex_lock(&mux_points);
			add_waypoint(buffer_gfx, waypoints, &waypoints_num, Lmouse);
			pthread_mutex_unlock(&mux_points);
		}
		
		if(Rmouse_clicked)
		{
			//printf("R clicked\n");
			pthread_mutex_lock(&mux_points);
			pthread_mutex_lock(&mux_gfx);
			del_waypoint(buffer_gfx, waypoints, &waypoints_num, Rmouse);
			pthread_mutex_unlock(&mux_points);
			pthread_mutex_unlock(&mux_gfx);
		}
		
		Lmouse_clicked = 0;
		Rmouse_clicked = 0;

		if (deadline_miss(tp))
			printf ("DEADLINE MISS: point_task()\n");

		wait_for_period (tp);
		
		eval_period(tp, thread_periods, &selected_thread, 2, &tp_changed);

	}while(!start_sim  && !end_sim);
	
	printf("Waypoints selection completed: found %d goals\n", waypoints_num);
	
	usleep(1e6);

	pthread_exit(0);

}

/*
 * Task
 * -----------------
 * Used for solve the ARE to get the gains
 * of the LQR controller
 */
void* gains_task(void* arg)
{
struct task_par* tp;

	tp = (struct task_par *)arg;
	set_period (tp);
	
	do
	{
		

		if (deadline_miss(tp))		
			printf ("DEADLINE MISS: gains_task()\n");

		wait_for_period (tp);	
		eval_period(tp, thread_periods, &selected_thread, 7, &tp_changed);
	
	}while(0);
	
	pthread_exit(0);
	
}


/*
 * Task
 * -----------------
 * Used for keyboard events
 * 
 */
void* key_task(void* arg)
{
struct task_par* tp;

	tp = (struct task_par *)arg;
	set_period (tp);

	key_enabled = 1;

	printf("Keyboard task started\n");

	do{
		scan = 0;
		
		// Check for pressed key
		if (keypressed())
			scan = readkey() >> 8;

		if(key[KEY_ESC])
			end_sim = 1;

		if(key[KEY_SPACE])
		{
			start_sim = 1;
			key_mode = 0;
		}

		if(key[KEY_ENTER])
		{
			if (key_mode == 1)
			{
				if (selected_thread < 99 && thread_periods[selected_thread] != sel_thread_old_tp)
				{
					tp_changed = 1;
				}
				else if(selected_thread < 99 && 
					thread_periods[selected_thread] == sel_thread_old_tp)
				{
					cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
					tp_changed = 0;
				}
			}
			else if (key_mode >= 2)
			{	
				printf("Q coeff: %lf - R coeff: %lf\n", Q_coeff, R_coeff);
				QR_changed = 1;
			}
			key_mode = 0;
		}
		
		
		if(key[KEY_0])
		{	
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 0);
		}
		if(key[KEY_1])
		{
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 1);
		}
		if(key[KEY_2])
		{
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 2);
		}
		if(key[KEY_3])
		{
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 3);
		}
		if(key[KEY_4])
		{
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 4);
		}
		
		if(key[KEY_5])
		{
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 5);
		}
			
		if(key[KEY_6])
		{
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 6);
		}

		if(key[KEY_7])
		{
			if (key_mode != 1) {key_mode = 1; R_coeff = R_coeff_old; Q_coeff = Q_coeff_old;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 7);
		}
		
		if(key[KEY_Q])
		{
			if(key_mode != 2)
			{
				printf("Select the new gain for the Q weights matrix\n");
				key_mode = 2;
				Q_coeff_old = Q_coeff;
			}
		}

		if(key[KEY_R])
		{
			if(key_mode != 3)
			{
				printf("Select the new gain for the R weights matrix\n");
				key_mode = 3;
				R_coeff_old = R_coeff;
			}
		}

		if(key[KEY_BACKSPACE])
		{
			if (key_mode == 1)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
			}
			else if (key_mode == 2)
			{
				Q_coeff = Q_coeff_old;
				printf("Q coeff reverted to: %lf\n", Q_coeff);
			}
			else if (key_mode == 3)
			{
				R_coeff = R_coeff_old;
				printf("R coeff reverted to: %lf\n", R_coeff);
			}
			
			key_mode = 0;
		}

		if(key[KEY_UP])
		{
			if(key_mode == 1)
				modify_thread_tp(&selected_thread, thread_periods, 10);
			else if(key_mode == 2)
			{
				Q_coeff *= 10;
				printf("Q coeff: %f\n", Q_coeff);
			}
			else if(key_mode == 3)
			{
				R_coeff *= 10;
				printf("R coeff: %f\n", R_coeff);
			}
		}
		
		if(key[KEY_DOWN])
		{
			if(key_mode == 1)
				modify_thread_tp(&selected_thread, thread_periods, -10);
			else if(key_mode == 2)
			{
				Q_coeff /= 10;
				printf("Q coeff: %f\n", Q_coeff);
			}
			else if(key_mode == 3)
			{
				R_coeff /= 10;
				printf("R coeff: %f\n", R_coeff);
			}
		}
		
		// Check left mouse click
		if(mouse_b & 1)
		{
			Lmouse_clicked = 1;
			Lmouse.x = (double)mouse_x;
			Lmouse.y = (double)mouse_y;
		}

		// Check right mouse click
		if(mouse_b & 2)
		{
			Rmouse_clicked = 1;
			Rmouse.x = (double)mouse_x;
			Rmouse.y = (double)mouse_y;
		}
		
		if (deadline_miss(tp))		
		printf ("DEADLINE MISS: key_task()\n");

		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 1, &tp_changed);

	}while(!end_sim);

	if(end_sim)
	{
		printf("ESCAPE key was pressed: Closing simulation\n");
		key_enabled = 0;
	}

	pthread_exit(0);

}
