/* --- REAL TIME MAIN --- */

#include "task.h"
#include "model.h"
#include "graphics.h"
#include "customdata.h"

/* global vars */

double arr_state[SIZE_X] = {0.0};
double arr_old_state[SIZE_X] = {0.0};
double arr_forces[SIZE_U] = {0.0}; 
double arr_error[SIZE_X] = {0.0};

float w_lift_off = 0.0;
float yawref = 0.0;

int gfx_initialized = 0;
int key_initialized = 0;
int gfx_enabled = 0;
int key_enabled = 0;

BITMAP* buffer_gfx;

char scan;

/* Semaphores */

pthread_mutex_t mux_forces;
pthread_mutex_t mux_state;
pthread_mutex_t mux_gfx;

pthread_mutexattr_t muxattr;

/* tasks  declaration */
void* lqr_task (void* arg);
void* lin_model_task (void* arg);
void* key_task (void* arg);
void* gfx_task (void* arg);
void* plt_task (void* arg);
void* laser_task (void* arg);

/* main thread */

int main (int argc, char **argv) 
{

struct sched_param sched_mod, sched_gfx, sched_lqr, sched_plt, sched_key, sched_lsr;
struct task_par tp_mod, tp_gfx, tp_lqr, tp_plt, tp_key, tp_lsr;
pthread_attr_t attr_mod, attr_gfx, attr_lqr, attr_plt, attr_key, attr_lsr;
pthread_t tid_mod, tid_gfx, tid_lqr, tid_plt, tid_key, tid_lsr;

int ret = 0;

    mutex_create (mux_forces, muxattr, 0, 100);
    mutex_create (mux_state, muxattr, 0, 100);
	mutex_create (mux_gfx, muxattr, 0, 100);
	 
	pthread_mutexattr_destroy (&muxattr);
	
	//Create Graphics Thread
    
    start_allegro();
    
	tp_gfx.arg = 1;
	tp_gfx.period = TP_GFX;
	tp_gfx.deadline = TP_GFX * 1;
	tp_gfx.priority = 7;
	tp_gfx.dmiss = 0;

	ret = thread_create (&tp_gfx, &sched_gfx, attr_gfx, &tid_gfx, gfx_task);

   	//Create Keyboard Thread
    
	tp_key.arg = 2;
	tp_key.period = TP_KEY;
	tp_key.deadline = TP_KEY * 1;
	tp_key.priority = 6;
	tp_key.dmiss = 0;

	ret = thread_create (&tp_key, &sched_key, attr_key, &tid_key, key_task);


    //Create Dynamic Model Thread
    
	tp_mod.arg = 3;
	tp_mod.period = TP_MODEL;
	tp_mod.deadline = TP_MODEL * 1;
	tp_mod.priority = 20;
	tp_mod.dmiss = 0;

	ret = thread_create (&tp_mod, &sched_mod, attr_mod, &tid_mod, lin_model_task);
	
    //Create LQR Thread
    
	tp_lqr.arg = 4;
	tp_lqr.period = TP_LQR;
	tp_lqr.deadline = TP_LQR * 1;
	tp_lqr.priority = 19;
	tp_lqr.dmiss = 0;

	ret = thread_create (&tp_lqr, &sched_lqr, attr_lqr, &tid_lqr, lqr_task);
	
	//Create LQR Thread
    
	tp_lsr.arg = 6;
	tp_lsr.period = TP_LSR;
	tp_lsr.deadline = TP_LSR * 1;
	tp_lsr.priority = 16;
	tp_lsr.dmiss = 0;

	ret = thread_create (&tp_lsr, &sched_lsr, attr_lsr, &tid_lsr, laser_task);
    
    //Create Plotting Thread
    
	tp_plt.arg = 5;
	tp_plt.period = TP_PLOTS;
	tp_plt.deadline = TP_PLOTS * 1;
	tp_plt.priority = 5;
	tp_plt.dmiss = 0;

	ret = thread_create (&tp_plt, &sched_plt, attr_plt, &tid_plt, plt_task);
    
    pthread_join (tid_plt, NULL);
    pthread_join (tid_lqr, NULL);
    pthread_join (tid_mod, NULL);
    pthread_join (tid_key, NULL);
	pthread_join (tid_gfx, NULL);
    
    close_allegro();
    /*
    pthread_attr_destroy (&attr_plt);
	pthread_attr_destroy (&attr_lqr);
    pthread_attr_destroy (&attr_mod);	
    pthread_attr_destroy (&attr_key);
    pthread_attr_destroy (&attr_gfx);
    */
    pthread_mutex_destroy(&mux_forces);
    pthread_mutex_destroy(&mux_state);
    pthread_mutex_destroy(&mux_gfx);
    
	return 0;
    
}


void* lin_model_task(void* arg)
{
struct task_par* tp;

	tp = (struct task_par *)arg;
	set_period (tp);

	gsl_vector_view forces;
	gsl_vector* state = gsl_vector_calloc(SIZE_X);

	gsl_matrix* A = gsl_matrix_calloc(SIZE_X, SIZE_X);
	gsl_matrix* B = gsl_matrix_calloc(SIZE_X, SIZE_U);

	do{

		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lin_model_task()\n");
		
		wait_for_period (tp);
        
	}while(key_enabled == 0 && gfx_enabled == 0);
    
    read_matrix_file("A.bin", A);
	read_matrix_file("B.bin", B);
    
    printf("Dynamic model task started\n");

    do{
		
		// Lock forces to store in internal variable
		pthread_mutex_lock (&mux_forces);
		
        forces = gsl_vector_view_array(arr_forces, SIZE_U);
        
		pthread_mutex_unlock (&mux_forces);
		
        // Lock state and compute new state from transition function
        pthread_mutex_lock (&mux_state);
        
		memcpy(arr_old_state, arr_state, sizeof(double) * SIZE_X);
		
		quad_linear_model(&forces.vector, A, B, state);

		memcpy(arr_state, state->data, sizeof(double) * SIZE_X);

		pthread_mutex_unlock (&mux_state);

		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: lin_model_task()\n");
		
		wait_for_period (tp);

	}while(scan != KEY_ESC);
	
	// free memory allocated for GSL variables
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
	gsl_matrix* K = gsl_matrix_calloc(SIZE_U, SIZE_X);
	gsl_vector_view state_view;
	tp = (struct task_par *)arg;
	
	set_period (tp);
	
	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lqr_task()\n");
		
		wait_for_period (tp);
	}while(key_enabled == 0 && gfx_enabled == 0);
    
    printf("LQR Controller task started\n");
	
    read_matrix_file("K.bin", K);
	
	gsl_vector_set(setpoint, 2, M_PI/4);
	gsl_vector_set(setpoint, 3, 10);
	gsl_vector_set(setpoint, 4, 10);
	gsl_vector_set(setpoint, 5, 10);
    
	do{
		
		pthread_mutex_lock (&mux_state);
		state_view = gsl_vector_view_array(arr_state, SIZE_X);

		dlqr_control(setpoint, &state_view.vector, K, forces);
		pthread_mutex_unlock (&mux_state);
		
		pthread_mutex_lock (&mux_forces);
		memcpy(arr_forces, forces->data, sizeof(double) * SIZE_U);
		pthread_mutex_unlock (&mux_forces);
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lqr_task()\n");
		
		wait_for_period (tp);
		
	}while(scan != KEY_ESC);

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
Trace laser_traces[5];
Trace old_traces[5];
double initialpose[6] = {0.0};
double old_pose[6] = {0.0};
double pose[6] = {0.0};
	
	tp = (struct task_par *)arg;
	
	aperture = 150;
	n_traces = 5;
	
	set_period (tp);
		
	init_laser_scanner(laser_traces, n_traces, aperture, initialpose);
	
	do{

		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lsr_task()\n");
		
		wait_for_period (tp);

	}while(key_enabled == 0 && gfx_enabled == 0);
    
    printf("Laser Scanner task started\n");
    
	do{
			
		pthread_mutex_lock(&mux_state);
		memcpy(old_pose, pose, sizeof(double) * SIZE_Y);
		memcpy(pose, arr_state, sizeof(double) * SIZE_Y);
		pthread_mutex_unlock(&mux_state);

		pthread_mutex_lock(&mux_gfx);
		memcpy(old_traces, laser_traces, sizeof(double) * 3 * n_traces);
		get_laser_distances(buffer_gfx, laser_traces, pose, aperture, n_traces);
	
		draw_laser_traces(buffer_gfx, old_traces, laser_traces, old_pose, pose);
		pthread_mutex_unlock(&mux_gfx);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lsr_task()\n");
		
		wait_for_period (tp);
		
	}while(scan != KEY_ESC);

    printf("Laser Scanner task closed\n");
    
	pthread_exit(0);
	
}

/*
 * Task
 * -----------------
 * Used for drawing model
 * 
 */
void* gfx_task(void* arg)
{
	
struct task_par* tp;

int gui_color =  makecol(0, 255, 0);
double old_pose[6] = {0.0};
double curr_pose[6] = {0.0};

BITMAP* quad;
BITMAP* quad_bg;

	tp = (struct task_par *)arg;
	
	set_period (tp);

    gfx_enabled = 1;

	buffer_gfx = create_bitmap(SCREEN_W, SCREEN_H);
	
	clear_to_color(buffer_gfx, 0);

	build_gui(buffer_gfx, font, COL_GREEN);
	
	quad = load_bmp("bmp/quad.bmp", NULL);
	quad_bg = load_bmp("bmp/black.bmp", NULL);
	
	if(quad == NULL)
		printf("Drone sprite not found!\n");
	
	do{
        
		pthread_mutex_lock(&mux_state);
		memcpy(old_pose, curr_pose, sizeof(double) * SIZE_Y);
		memcpy(curr_pose, arr_state, sizeof(double) * SIZE_Y);
		pthread_mutex_unlock(&mux_state); 
		
		if (quad != NULL)
			draw_quad(buffer_gfx, quad, quad_bg, old_pose, curr_pose);
		else
			draw_pose(buffer_gfx, old_pose, curr_pose);
		
        pthread_mutex_lock(&mux_gfx);

		blit(buffer_gfx,screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);
             
		pthread_mutex_unlock(&mux_gfx);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: gfx_task()\n");
		
		wait_for_period (tp);
        
	}while(key_enabled == 1);

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
	}while(key_enabled == 0 && gfx_enabled == 0);
    
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
		rectfill(buffer_gfx, 696, 286, PLT_XPOS_XCOORD - 1, PLT_XPOS_YCOORD - 1, 0);
		rectfill(buffer_gfx, 696, 391, PLT_YPOS_XCOORD - 1, PLT_YPOS_YCOORD - 1, 0);
		rectfill(buffer_gfx, 696, 496, PLT_ZPOS_XCOORD - 1, PLT_ZPOS_YCOORD - 1, 0);
		
		rectfill(buffer_gfx, 580 + 1, 285 + 1, 680 - 1, 385 - 1, 0);
		rectfill(buffer_gfx, 580 + 1, 390 + 1, 680 - 1, 490 - 1, 0);
		rectfill(buffer_gfx, 580 + 1, 495 + 1, 680 - 1, 595 - 1, 0);
		
		update_plot(buffer_gfx, plt_buf_Roll, PLT_XPOS_XCOORD - 115, PLT_XPOS_YCOORD);		
		update_plot(buffer_gfx, plt_buf_Pitch, PLT_YPOS_XCOORD - 115, PLT_YPOS_YCOORD);
		update_plot(buffer_gfx, plt_buf_Yaw, PLT_ZPOS_XCOORD - 115, PLT_ZPOS_YCOORD);
		
		update_plot(buffer_gfx, plt_buf_X, PLT_XPOS_XCOORD, PLT_XPOS_YCOORD);		
		update_plot(buffer_gfx, plt_buf_Y, PLT_YPOS_XCOORD, PLT_YPOS_YCOORD);
		update_plot(buffer_gfx, plt_buf_Z, PLT_ZPOS_XCOORD, PLT_ZPOS_YCOORD);
		
		pthread_mutex_unlock(&mux_gfx);

		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: plt_task()\n");

		wait_for_period (tp);

	}while(gfx_enabled && key_enabled);
	
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

	if (keypressed())
		scan = readkey() >> 8;

	if (deadline_miss(tp))		
	printf ("DEADLINE MISS: key_task()\n");

	wait_for_period (tp);

	}while(scan != KEY_ESC);

	if(scan == KEY_ESC)
	{
		printf("ESCAPE key was pressed: Closing simulation\n");
		key_enabled = 0;
	}

	pthread_exit(0);

}
