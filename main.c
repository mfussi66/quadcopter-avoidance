/* --- REAL TIME MAIN --- */

#include "task.h"
#include "model.h"
#include "graphics.h"
#include "customdata.h"

/* global vars */

double arr_state[SIZE_X] = {0.0};
double arr_forces[SIZE_U] = {0.0}; 
double arr_error[SIZE_X] = {0.0};

float w_lift_off = 0.0;
float yawref = 0.0;

int gfx_initialized = 0;
int gfx_closed = 0;

BITMAP* buffer_plt;
BITMAP* buffer_gfx;

/* Semaphores */

pthread_mutex_t mux_forces;
pthread_mutex_t mux_state;
pthread_mutex_t mux_plt;

pthread_mutexattr_t muxattr;

/* tasks  declaration */
void* lqr_task (void* arg);
void *lin_model_task (void *arg);
void *gfx_task (void *arg);
void *plt_task (void *arg);

/* main thread */

int main (int argc, char **argv) 
{

struct sched_param sched_mod, sched_gfx, sched_lqr, sched_plt;
struct task_par tp_mod, tp_gfx, tp_lqr, tp_plt;
pthread_attr_t attr_mod, attr_gfx, attr_lqr, attr_plt;
pthread_t tid_mod, tid_gfx, tid_lqr, tid_plt;

int ret = 0;

    mutex_create (mux_forces, muxattr, 0, 100);
    mutex_create (mux_state, muxattr, 0, 100);
	mutex_create (mux_plt, muxattr, 0, 100);
	 
	pthread_mutexattr_destroy (&muxattr);
	
	//Create Graphics Thread
    
	tp_gfx.arg = 4;
	tp_gfx.period = TPERIOD_GFX;
	tp_gfx.deadline = TPERIOD_GFX * 0.5;
	tp_gfx.priority = 31;
	tp_gfx.dmiss = 0;

	ret = thread_create (&tp_gfx, &sched_gfx, attr_gfx, &tid_gfx, gfx_task);


	//Create Plotting Thread
    
	tp_plt.arg = 2;
	tp_plt.period = TPERIOD_PLOTS;
	tp_plt.deadline = TPERIOD_PLOTS * 0.5;
	tp_plt.priority = 30;
	tp_plt.dmiss = 0;

	ret = thread_create (&tp_plt, &sched_plt, attr_plt, &tid_plt, plt_task);

    //Create Dynamic Model Thread
    
	tp_mod.arg = 1;
	tp_mod.period = TPERIOD_MODEL;
	tp_mod.deadline = TPERIOD_MODEL * 0.9;
	tp_mod.priority = 22;
	tp_mod.dmiss = 0;

	ret = thread_create (&tp_mod, &sched_mod, attr_mod, &tid_mod, lin_model_task);
	
    //Create LQR Thread
    
	tp_lqr.arg = 3;
	tp_lqr.period = TPERIOD_LQR;
	tp_lqr.deadline = TPERIOD_LQR * 0.5;
	tp_lqr.priority = 23;
	tp_lqr.dmiss = 0;

	ret = thread_create (&tp_lqr, &sched_lqr, attr_lqr, &tid_lqr, lqr_task);

	pthread_join (tid_mod, 0);
	pthread_join (tid_lqr, 0);
	pthread_join (tid_gfx, 0);
	pthread_join (tid_plt, 0);
		
    pthread_attr_destroy (&attr_mod);	
	pthread_attr_destroy (&attr_lqr);
	pthread_attr_destroy (&attr_gfx);
	pthread_attr_destroy (&attr_plt);
	return 0;
}


void *lin_model_task(void* arg)
{
struct task_par *tp;

	tp = (struct task_par *)arg;
	set_period (tp);

	gsl_vector *forces = gsl_vector_calloc(SIZE_U);
	gsl_vector *state = gsl_vector_calloc(SIZE_X);

	gsl_matrix *A = gsl_matrix_calloc(SIZE_X, SIZE_X);
	gsl_matrix *B = gsl_matrix_calloc(SIZE_X, SIZE_U);

	double val = 0.0;

	read_matrix_file("A.bin", A);
	read_matrix_file("B.bin", B);
	
	while(gfx_initialized == 0)
	{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lin_model_task()\n");
		
		wait_for_period (tp);
	}

	while ( keypressed() == 0)
    {		
		pthread_mutex_lock (&mux_forces);
		
		for(int i = 0; i < SIZE_U; i++)
			gsl_vector_set(forces,i, arr_forces[i]);

		pthread_mutex_unlock (&mux_forces);

		quad_linear_model(forces, A, B, state);
		
		pthread_mutex_lock (&mux_state);

		for(int i = 0; i < SIZE_X; i++)
		{
			arr_state[i] = gsl_vector_get(state, i);
		}
		
		pthread_mutex_unlock (&mux_state);
		
		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: lin_model_task()\n");
		
		wait_for_period (tp);

	}
	
	gsl_vector_free(state);
	gsl_vector_free(forces);
	gsl_matrix_free(A);
	gsl_matrix_free(B);
	
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
	struct task_par *tp;
	
	gsl_vector *state = gsl_vector_calloc(SIZE_X);
	gsl_vector *forces = gsl_vector_calloc(SIZE_U);
	gsl_vector *setpoint = gsl_vector_calloc(SIZE_X);
	gsl_matrix *K = gsl_matrix_calloc(SIZE_U, SIZE_X);
	
	tp = (struct task_par *)arg;
	
	set_period (tp);

	read_matrix_file("K.bin", K);
	
	gsl_vector_set(setpoint, 3, 1);
	gsl_vector_set(setpoint, 4, 1);
	gsl_vector_set(setpoint, 5, 1);
	
	while(gfx_initialized == 0)
	{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lqr_task()\n");
		
		wait_for_period (tp);
	}
	
	while(keypressed() == 0)
	{
		pthread_mutex_lock (&mux_state);
		
		for(int i = 0; i < SIZE_X; i++)
			gsl_vector_set(state, i, arr_state[i]);
			//printf("arr_state: %lf\n",arr_state[5]);
		
		pthread_mutex_unlock (&mux_state);
		
		dlqr_control(setpoint, state, K, forces);

		pthread_mutex_lock (&mux_forces);
		
		for(int i = 0; i < SIZE_U; i++)
			arr_forces[i] = gsl_vector_get(forces, i);
		
		pthread_mutex_unlock (&mux_forces);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lqr_task()\n");
		
		wait_for_period (tp);
	}
	
	gsl_vector_free(state);
	gsl_vector_free(forces);
	gsl_vector_free(setpoint);
	gsl_matrix_free(K);
	
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
	
struct task_par *tp;

int col;

	tp = (struct task_par *)arg;
	
	set_period (tp);
	
	start_allegro();
	
	pthread_mutex_lock(&mux_plt);
	buffer_gfx = create_bitmap(SCREEN_W, SCREEN_H);
	
	col = makecol(0, 255, 0);
	
	rect(buffer_gfx, 5, 5, 560, 595, col);
	
	pthread_mutex_unlock(&mux_plt);
	
	gfx_initialized = 1;
	
	while(keypressed() == 0)
	{
		pthread_mutex_lock(&mux_plt);
		
		blit(buffer_gfx,screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);

		pthread_mutex_unlock(&mux_plt);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: gfk_task()\n");
		
		wait_for_period (tp);
	}
	
	gfx_closed = 1;
	
	close_allegro ();
	
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
	
struct task_par *tp;

double plt_buf_Roll[GRAPH_DATA_SIZE] = {0.0};
double plt_buf_Pitch[GRAPH_DATA_SIZE] = {0.0};
double plt_buf_Yaw[GRAPH_DATA_SIZE] = {0.0};
double plt_buf_X[GRAPH_DATA_SIZE] = {0.0};
double plt_buf_Y[GRAPH_DATA_SIZE] = {0.0};
double plt_buf_Z[GRAPH_DATA_SIZE] = {0.0};

int col; 

	tp = (struct task_par *)arg;
	
	set_period (tp);
	
	while(gfx_initialized == 0)
	{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: plt_task()\n");
		
		wait_for_period (tp);
	}
	
	col = makecol(0, 255, 0);

	pthread_mutex_lock(&mux_plt);	
	buffer_plt = create_bitmap(SCREEN_W, SCREEN_H);

	rect(buffer_gfx, 695, 285, GRAPH_XPOS_XCOORD, GRAPH_XPOS_YCOORD, col);
	rect(buffer_gfx, 695, 390, GRAPH_YPOS_XCOORD, GRAPH_YPOS_YCOORD, col);
	rect(buffer_gfx, 695, 495, GRAPH_ZPOS_XCOORD, GRAPH_ZPOS_YCOORD, col);
	
	rect(buffer_gfx, 580, 285, 680, 385, col);
	rect(buffer_gfx, 580, 390, 680, 490, col);
	rect(buffer_gfx, 580, 495, 680, 595, col);
	
	textout_centre_ex(buffer_gfx, font, "R", 575, 335, col, -1);
	textout_centre_ex(buffer_gfx, font, "P", 575, 440, col, -1);
	textout_centre_ex(buffer_gfx, font, "Y", 575, 545, col, -1);
	
	textout_centre_ex(buffer_gfx, font, "X", 690, 335, col, -1);
	textout_centre_ex(buffer_gfx, font, "Y", 690, 440, col, -1);
	textout_centre_ex(buffer_gfx, font, "Z", 690, 545, col, -1);
	pthread_mutex_unlock(&mux_plt);
	
	while(keypressed() == 0 && gfx_closed ==0)
	{
		pthread_mutex_lock(&mux_state);
		
		shift_and_append(plt_buf_X, GRAPH_DATA_SIZE, arr_state[3]);
		shift_and_append(plt_buf_Z, GRAPH_DATA_SIZE, arr_state[5]);
		shift_and_append(plt_buf_Y, GRAPH_DATA_SIZE, arr_state[4]);
		
		shift_and_append(plt_buf_Roll, GRAPH_DATA_SIZE, arr_state[0]);
		shift_and_append(plt_buf_Pitch, GRAPH_DATA_SIZE, arr_state[1]);
		shift_and_append(plt_buf_Yaw, GRAPH_DATA_SIZE, arr_state[2]);
		
		pthread_mutex_unlock(&mux_state);
		
		pthread_mutex_lock(&mux_plt);
		rectfill(buffer_gfx, 696, 286, GRAPH_XPOS_XCOORD - 1, GRAPH_XPOS_YCOORD - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 696, 391, GRAPH_YPOS_XCOORD - 1, GRAPH_YPOS_YCOORD - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 696, 496, GRAPH_ZPOS_XCOORD - 1, GRAPH_ZPOS_YCOORD - 1, makecol(0,0,0));
		
		rectfill(buffer_gfx, 580 + 1, 285 + 1, 680 - 1, 385 - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 580 + 1, 390 + 1, 680 - 1, 490 - 1, makecol(0,0,0));
		rectfill(buffer_gfx, 580 + 1, 495 + 1, 680 - 1, 595 - 1, makecol(0,0,0));
		
		update_graph(buffer_gfx, plt_buf_Roll, GRAPH_XPOS_XCOORD - 115, GRAPH_XPOS_YCOORD);		
		update_graph(buffer_gfx, plt_buf_Pitch, GRAPH_YPOS_XCOORD - 115, GRAPH_YPOS_YCOORD);
		update_graph(buffer_gfx, plt_buf_Yaw, GRAPH_ZPOS_XCOORD - 115, GRAPH_ZPOS_YCOORD);
		
		update_graph(buffer_gfx, plt_buf_X, GRAPH_XPOS_XCOORD, GRAPH_XPOS_YCOORD);		
		update_graph(buffer_gfx, plt_buf_Y, GRAPH_YPOS_XCOORD, GRAPH_YPOS_YCOORD);
		update_graph(buffer_gfx, plt_buf_Z, GRAPH_ZPOS_XCOORD, GRAPH_ZPOS_YCOORD);
		
		pthread_mutex_unlock(&mux_plt);

		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: plt_task()\n");

		//readkey();
		wait_for_period (tp);

	}
	
	pthread_exit(0);	
}
