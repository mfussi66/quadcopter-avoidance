/* --- REAL TIME MAIN --- */

#include "task.h"
#include "model.h"
#include "graphics.h"
#include "customdata.h"

/* global vars */

double arr_state[SIZE_X] = {0.0};
double arr_old_state[SIZE_X] = {0.0};
double arr_forces[SIZE_U] = {0.0}; 
double arr_repulsive_forces[2] = {0.0};
double arr_error[SIZE_X] = {0.0};
double arr_sp_uvw[3] = {0.0};
double yawref = 0.0;

double P_gains[SIZE_PID] = {0.0};
double D_gains[SIZE_PID] = {0.0};
double P_gains_df[SIZE_PID] = {0.0};
double D_gains_df[SIZE_PID] = {0.0};

Trace arr_traces[N_BEAMS];
Trace arr_old_traces[N_BEAMS];

WPoint waypoints[5] = {{-9999}, {-9999}};

Obstacle arr_obstacles[OBS_NUM];
int avoid = 0;

double altitude_sp = 5;

WPoint target = {-9999, -9999};

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
pthread_mutex_t mux_vel_avoid;
pthread_mutex_t mux_state;
pthread_mutex_t mux_gfx;
pthread_mutex_t mux_points;
pthread_mutex_t mux_laser;
pthread_mutex_t mux_rpy_sp;
pthread_mutex_t mux_gains;
pthread_mutex_t mux_alt;

pthread_mutexattr_t muxattr;

/* tasks  declaration */
void* lin_model_task (void* arg);
void* key_task (void* arg);
void* gfx_task (void* arg);
void* plt_task (void* arg);
void* laser_task (void* arg);
void* point_task (void* arg);

/* main thread */

int main (int argc, char **argv) 
{

struct sched_param sched_mod, sched_gfx, sched_pnt;
struct sched_param sched_plt, sched_key, sched_lsr;

struct task_par tp_mod, tp_gfx, tp_plt;
struct task_par tp_key, tp_lsr, tp_pnt;

pthread_attr_t attr_mod, attr_gfx, attr_plt;
pthread_attr_t attr_key, attr_lsr, attr_pnt;

pthread_t tid_mod, tid_gfx, tid_plt; 
pthread_t tid_key, tid_lsr, tid_pnt;

int ret = 0;

	mutex_create (mux_forces, muxattr, 0, 100);
	mutex_create (mux_vel_avoid, muxattr, 0, 100);
	mutex_create (mux_state, muxattr, 0, 100);
	mutex_create (mux_points, muxattr, 0, 100);
	mutex_create (mux_gfx, muxattr, 0, 100);
	mutex_create (mux_laser, muxattr, 0, 100);
	mutex_create (mux_rpy_sp, muxattr, 0, 100);
	mutex_create (mux_gains, muxattr, 0, 100);
	mutex_create (mux_alt, muxattr, 0, 100);
	
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

	//Create Laser scan Thread
	set_task_params(&tp_lsr, 6, TP_LSR, TP_LSR, 16);
	ret = thread_create (&tp_lsr, &sched_lsr, attr_lsr, &tid_lsr, laser_task);
	thread_periods[4] = TP_LSR;

	//Create Plotting Thread
	set_task_params(&tp_plt, 7, TP_PLOTS, TP_PLOTS, 5);
	ret = thread_create (&tp_plt, &sched_plt, attr_plt, &tid_plt, plt_task);
	thread_periods[5] = TP_PLOTS;

	
	pthread_join (tid_pnt, NULL);
	pthread_join (tid_lsr, NULL);
	pthread_join (tid_plt, NULL);
	pthread_join (tid_mod, NULL);
	pthread_join (tid_key, NULL);
	pthread_join (tid_gfx, NULL);
 
	close_allegro();

	pthread_mutex_destroy(&mux_points);
	pthread_mutex_destroy(&mux_rpy_sp);
	pthread_mutex_destroy(&mux_forces);
	pthread_mutex_destroy(&mux_state);
	pthread_mutex_destroy(&mux_laser);
	pthread_mutex_destroy(&mux_gfx);
	pthread_mutex_destroy(&mux_vel_avoid);
	pthread_mutex_destroy(&mux_alt);

	return 0;

}

void* lin_model_task(void* arg)
{
struct task_par* tp;

double forces[SIZE_U] = {0.0};
double state[SIZE_X] = {0.0};

int wp_flags[MAX_WPOINTS] = {0};
int wp_idx = 1;
double dist = 0.0;

double error_xyz[3] = {0.0};
double error_xyz_old[3] = {0.0};
double error_vel[3] = {0.0};
double error_vel_old[3] = {0.0};
double error_rpy[3] = {0.0};
double error_rpy_old[3] = {0.0};
double setpoint_rpy[3] = {0.0};
double setpoint_xyz[3] = {0.0};
double setpoint_uvw[3] = {0.0};
double xyz[3] = {0.0};
double uvw[3] = {0.0};
double rpy[3] = {0.0};
double rep_forces[2] = {0.0};
double total_forces[SIZE_U] = {0.0};

	tp = (struct task_par *)arg;
	
	set_period (tp);
	
	init_gains(P_gains, D_gains, P_gains_df, D_gains_df);
	
	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: model_task()\n");

		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 3, &tp_changed);

	}while(!start_sim && !end_sim);

	if(start_sim)
	{
		// Set initial state from waypoints array
		state[3] = (waypoints[0].x - ENV_OFFSET_X) / ENV_SCALE;
		state[4] = (ENV_OFFSET_Y - waypoints[0].y) / ENV_SCALE;
		state[5] = 0;
		next_setpoint(setpoint_xyz, waypoints, waypoints_num, wp_flags);
		setpoint_rpy[2] = atan2(setpoint_xyz[1] - state[4], setpoint_xyz[0] - state[3]);
		target.x = setpoint_xyz[0];
		target.y = setpoint_xyz[1];
		yawref = setpoint_rpy[2];
		memcpy(arr_state, state, sizeof(double) * SIZE_X);
		printf("Dynamic model task started\n");
	}
	
    do{
		for(int i = 0; i < 3; i++) rpy[i] = state[i];
		for(int i = 0; i < 3; i++) xyz[i] = state[i + 3];
		for(int i = 0; i < 3; i++) uvw[i] = state[i + 9];

		dist = compute_pos_dist(setpoint_xyz, xyz);
		
		if(dist <= 0.15 && wp_idx >= waypoints_num)
		{
			setpoint_rpy[2] = state[2];
		}
		else if(dist <= 0.15 && wp_idx < waypoints_num)
		{
			printf("Waypoint %d reached\n", wp_idx);
			wp_flags[wp_idx] = 1;
			wp_idx++;
			next_setpoint(setpoint_xyz, waypoints, waypoints_num, wp_flags);
			setpoint_rpy[2] = atan2_safe(setpoint_xyz[1] - state[4], setpoint_xyz[0] - state[3]);
			target.x = setpoint_xyz[0];
			target.y = setpoint_xyz[1];
			yawref = setpoint_rpy[2];
		}
		
		// Copy altitude setpoint in case it has changed
		pthread_mutex_lock(&mux_alt);
		setpoint_xyz[2] = altitude_sp;
		pthread_mutex_unlock(&mux_alt);

		pthread_mutex_lock(&mux_gains);
		
		// [OUTER LOOP] POSITION/VELOCITY PID CONTROL
		if (avoid == 0)
		{
			compute_error(setpoint_xyz, xyz, error_xyz, error_xyz_old, 3);
			rotate_error(error_xyz, rpy[2]);
			pid_xyz_control(error_xyz, error_xyz_old, 
						   tp->period / 1000.0, forces, setpoint_uvw, P_gains, D_gains);
			
			compute_error(setpoint_uvw, uvw, error_vel, error_vel_old, 3);
			pid_vel_control(error_vel, error_vel_old, 
							tp->period / 1000.0, setpoint_rpy, P_gains, D_gains);
		}
		else
		{
			pthread_mutex_lock(&mux_vel_avoid);
			memcpy(setpoint_uvw, arr_sp_uvw, sizeof(double) * 3);
			pthread_mutex_unlock(&mux_vel_avoid);
			compute_error(setpoint_uvw, uvw, error_vel, error_vel_old, 3);
			pid_vel_control(error_vel, error_vel_old, 
							tp->period / 1000.0, setpoint_rpy, P_gains, D_gains);
		}
		
		// [INNER LOOP] ROLL PITCH YAW PID CONTROL
		compute_error(setpoint_rpy, rpy, error_rpy, error_rpy_old, 3);
		error_rpy[2] = anglediff_safe(setpoint_rpy[2], rpy[2]);

		if (avoid  == 0)
			setpoint_rpy[2] = atan2_safe(setpoint_xyz[1] - state[4], setpoint_xyz[0] - state[3]);
		else
			setpoint_rpy[2] = yawref;
		
		pid_rpy_alt_control(error_rpy, error_rpy_old, 
							tp->period / 1000.0, forces, P_gains, D_gains);
		
		pthread_mutex_unlock(&mux_gains);
		
		// APPLY FORCES TO LINEAR SYSTEM
		lin_model(forces, state, tp->period / 1000.0);
		
		pthread_mutex_lock(&mux_state);
		memcpy(arr_old_state, arr_state, sizeof(double) * SIZE_X);
		memcpy(arr_state, state, sizeof(double) * SIZE_X);
		pthread_mutex_unlock(&mux_state);
		
		printf("sp:%f ,Z:%f e:%f\n", setpoint_xyz[2], state[5], error_xyz[2]);
		
		// Save forces to plot
		pthread_mutex_lock(&mux_forces);
		memcpy(arr_forces, forces, sizeof(double) * SIZE_U);
		pthread_mutex_unlock(&mux_forces);
		
		collision = chk_collisions(arr_state, arr_obstacles, OBS_NUM);
		if (collision ==1) 
			printf("Collision detected!!\n");

			pthread_mutex_unlock(&mux_state);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: model_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 3, &tp_changed);

	}while(!end_sim && collision == 0);
	
    printf("model task closed\n");
    
	pthread_exit(0);

}

void* laser_task(void* arg)
{
struct task_par* tp;

double aperture;
int n_traces;
Trace laser_traces[N_BEAMS];
Trace old_traces[N_BEAMS];
double initialpose[SIZE_Y] = {0.0};
double old_state[SIZE_X] = {0.0};
double state[SIZE_X] = {0.0};
int mode = 0;
int avoid_orientation = 0;
	
	tp = (struct task_par *)arg;
	set_period (tp);	
	
	aperture = 153;
	n_traces = N_BEAMS;

	srand(time(NULL));
		
	init_laser_scanner(laser_traces, N_BEAMS, aperture, initialpose);
	init_laser_scanner(old_traces, N_BEAMS, aperture, initialpose);
	
	do{

		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lsr_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 4, &tp_changed);

	}while(!start_sim && !end_sim);
    
    printf("Laser Scanner task started\n");
    
	do{
		
		pthread_mutex_lock(&mux_state);
		memcpy(old_state, state, sizeof(double) * SIZE_X);
		memcpy(state, arr_state, sizeof(double) * SIZE_X);
		pthread_mutex_unlock(&mux_state);
		
		memcpy(old_traces, laser_traces, sizeof(Trace) * N_BEAMS);
		
		pthread_mutex_lock(&mux_gfx);
		get_laser_distances(buffer_gfx, laser_traces, state, aperture, N_BEAMS);
		pthread_mutex_unlock(&mux_gfx);
		
		mode = set_avoid_mode(laser_traces, state, &target, &avoid_orientation);

		if(mode > 0)
		{
			compute_avoid_sp(state, mode, arr_sp_uvw, &yawref, avoid_orientation);
			draw_msg(buffer_gfx, 3, WIDTH_SCREEN - 100,  HEIGHT_SCREEN/2 + 50);
			avoid = 1;
		}
		else
		{
			draw_msg(buffer_gfx, 2, WIDTH_SCREEN - 100,  HEIGHT_SCREEN/2 + 50);
			avoid = 0;
		}

		pthread_mutex_lock(&mux_gfx);
		draw_laser_traces(buffer_gfx, old_traces, laser_traces, old_state, state);
		pthread_mutex_unlock(&mux_gfx);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lsr_task()\n");
		
		wait_for_period (tp);
		eval_period(tp, thread_periods, &selected_thread, 4, &tp_changed);
		
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
		pthread_mutex_lock(&mux_points);
		scare_mouse();
		draw_obstacles(buffer_gfx, arr_obstacles, OBS_NUM, COL_GREEN);
		
		memcpy(old_waypoints, curr_waypoints, sizeof(WPoint) * MAX_WPOINTS);
		memcpy(curr_waypoints, waypoints, sizeof(WPoint) * MAX_WPOINTS);
		draw_waypoints(buffer_gfx, old_waypoints, curr_waypoints, waypoints_num);
		

		draw_periods(buffer_gfx, thread_periods, THREAD_MAX_NUM, selected_thread);
		draw_gains(buffer_gfx, P_gains, D_gains, key_mode);
		
		pthread_mutex_lock(&mux_gfx);
		blit(buffer_gfx,screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
		pthread_mutex_unlock(&mux_gfx);
		pthread_mutex_unlock(&mux_points);
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
		draw_gains(buffer_gfx, P_gains, D_gains, key_mode);
		
		if (collision)
		{
			draw_quad(buffer_gfx, expl, quad_bg, old_pose, curr_pose);
			draw_msg(buffer_gfx, 1, WIDTH_SCREEN/2, HEIGHT_SCREEN/2);
		}
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
	
	usleep(4e6);

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
double plt_buf_Z[PLT_DATA_SIZE] = {0.0};
double plt_buf_tauX[PLT_DATA_SIZE] = {0.0};
double plt_buf_tauY[PLT_DATA_SIZE] = {0.0};
double plt_buf_tauZ[PLT_DATA_SIZE] = {0.0};

double new_pose[SIZE_X] = {0.0};
double new_forces[SIZE_U] = {0.0};

	tp = (struct task_par *)arg;
	
	set_period (tp);
	
	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: plt_task()\n");
		
		wait_for_period (tp);
		
		eval_period(tp, thread_periods, &selected_thread, 5, &tp_changed);
		
	}while(!start_sim && !end_sim);
    
    printf("Plotting task started\n");

	do{
			
		pthread_mutex_lock(&mux_state);
		memcpy(new_pose, arr_state, sizeof(double) * SIZE_X);
		pthread_mutex_unlock(&mux_state);
		
		pthread_mutex_lock(&mux_forces);
		memcpy(new_forces, arr_forces, sizeof(double) * SIZE_U);
		pthread_mutex_unlock(&mux_forces);
        
		shift_and_append(plt_buf_tauX, PLT_DATA_SIZE, new_forces[0]);
		shift_and_append(plt_buf_tauY, PLT_DATA_SIZE, new_forces[1]);
		shift_and_append(plt_buf_tauZ, PLT_DATA_SIZE, new_forces[2]);
		
		shift_and_append(plt_buf_Roll, PLT_DATA_SIZE, new_pose[0]);
		shift_and_append(plt_buf_Pitch, PLT_DATA_SIZE, new_pose[1]);
		shift_and_append(plt_buf_Z, PLT_DATA_SIZE, new_pose[5]);
    
		pthread_mutex_lock(&mux_gfx);
					
		update_plot(buffer_gfx, plt_buf_tauX, PLT_11_XCOORD, PLT_11_YCOORD, 1e3);
		update_plot(buffer_gfx, plt_buf_tauY, PLT_21_XCOORD, PLT_21_YCOORD, 1e3);
		update_plot(buffer_gfx, plt_buf_tauZ, PLT_31_XCOORD, PLT_31_YCOORD, 1e2);
		
		update_plot(buffer_gfx, plt_buf_Roll, PLT_12_XCOORD, PLT_12_YCOORD, 1e3);
		update_plot(buffer_gfx, plt_buf_Pitch, PLT_22_XCOORD, PLT_22_YCOORD, 1e3);
		update_plot(buffer_gfx, plt_buf_Z, PLT_32_XCOORD , PLT_32_YCOORD, 1);

		pthread_mutex_unlock(&mux_gfx);

		if (deadline_miss (tp))
			printf ("DEADLINE MISS: plt_task()\n");

		wait_for_period(tp);

		eval_period(tp, thread_periods, &selected_thread, 5, &tp_changed);

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
		
		
		draw_msg(buffer_gfx, waypoints_num + 5, WIDTH_SCREEN - 100,  HEIGHT_SCREEN/2 + 50);
	
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
			if (waypoints_num >= 1) start_sim = 1;
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

			key_mode = 0;
		}
		
		
		if(key[KEY_0])
		{	
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 0);
		}
		if(key[KEY_1])
		{
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 1);
		}
		if(key[KEY_2])
		{
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 2);
		}
		if(key[KEY_3])
		{
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 3);
		}
		if(key[KEY_4])
		{
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 4);
		}
		
		if(key[KEY_5])
		{
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 5);
		}
			
		if(key[KEY_6])
		{
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 6);
		}

		if(key[KEY_7])
		{
			if (key_mode != 1) {key_mode = 1;}
			select_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp, 7);
		}
		
		if(key[KEY_X])
		{
			if(key_mode != 2)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 2;
			}
		}

		if(key[KEY_Y])
		{
			if(key_mode != 3)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 3;
			}
		}
		
		if(key[KEY_Z])
		{
			if(key_mode != 4)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 4;
			}
		}

		if(key[KEY_R])
		{
			if(key_mode != 5)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 5;
			}
		}
		
		if(key[KEY_P])
		{
			if(key_mode != 6)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 6;
			}
		}
		
		if(key[KEY_A])
		{
			if(key_mode != 7)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 7;
			}
		}
		
		if(key[KEY_U])
		{
			if(key_mode != 8)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 8;
			}
		}
		
		if(key[KEY_V])
		{
			if(key_mode != 9)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 9;
			}
		}
		
		if(key[KEY_W])
		{
			if(key_mode != 10)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 10;
			}
		}
		
		if(key[KEY_L])
		{
			if(key_mode != 11)
			{
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
				key_mode = 11;
			}
		}
		
		if(key[KEY_C])
		{
			reset_gains(P_gains, D_gains, P_gains_df, D_gains_df);
			key_mode = 0;
		}
		
		if(key[KEY_BACKSPACE])
		{
			if (key_mode == 1)
				cancel_thread_tp(&selected_thread, thread_periods, &sel_thread_old_tp);
			key_mode = 0;
		}

		if(key[KEY_UP])
		{
			if(key_mode == 1)
				modify_thread_tp(&selected_thread, thread_periods, 10);
			else if(key_mode > 1 && key_mode < 11)
			{
				pthread_mutex_lock(&mux_gains);
				adjust_gain(P_gains, D_gains, key_mode, 0);
				pthread_mutex_unlock(&mux_gains);
			}
			else if(key_mode == 11)
			{
				pthread_mutex_lock(&mux_alt);
				altitude_sp = altitude_sp * 1.2;
				pthread_mutex_unlock(&mux_alt);
			}
		}
		
		if(key[KEY_DOWN])
		{
			if(key_mode == 1)
				modify_thread_tp(&selected_thread, thread_periods, -10);
			else if(key_mode > 1 && key_mode < 11)
			{
				pthread_mutex_lock(&mux_gains);
				adjust_gain(P_gains, D_gains, key_mode, 1);
				pthread_mutex_unlock(&mux_gains);
			}
			else if(key_mode == 11)
			{
				pthread_mutex_lock(&mux_alt);
				altitude_sp = altitude_sp * 0.8;
				pthread_mutex_unlock(&mux_alt);
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
