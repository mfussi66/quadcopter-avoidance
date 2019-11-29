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

/* Semaphores */

pthread_mutex_t mux_forces;
pthread_mutex_t mux_gfx;
pthread_mutex_t mux_state;

pthread_mutexattr_t muxattr;

/* tasks  declaration */
void* lqr_task (void* arg);
void *lin_model_task (void *arg);
void *gfx_task (void *arg);

/* main thread */

int main (int argc, char **argv) 
{

struct sched_param sched_mod, sched_gfx, sched_lqr;
struct task_par tp_mod, tp_gfx, tp_lqr;
pthread_attr_t attr_mod, attr_gfx, attr_lqr;
pthread_t tid_mod, tid_gfx, tid_lqr;

int ret = 0;

    mutex_create (mux_forces, muxattr, 0, 100);
    mutex_create (mux_state, muxattr, 0, 100);

	pthread_mutexattr_destroy (&muxattr);

	//Create Graphics Thread
    
	tp_gfx.arg = 2;
	tp_gfx.period = TPERIOD_GRAPHICS;
	tp_gfx.deadline = TPERIOD_GRAPHICS * 0.7;
	tp_gfx.priority = 30;
	tp_gfx.dmiss = 0;

	ret = thread_create (&tp_gfx, &sched_gfx, attr_gfx, &tid_gfx, gfx_task);

    //Create Dynamic Model Thread
    
	tp_mod.arg = 1;
	tp_mod.period = TPERIOD_MODEL;
	tp_mod.deadline = TPERIOD_MODEL * 0.8;
	tp_mod.priority = 22;
	tp_mod.dmiss = 0;

	ret = thread_create (&tp_mod, &sched_mod, attr_mod, &tid_mod, lin_model_task);
	
    //Create LQR Thread
    
	tp_lqr.arg = 3;
	tp_lqr.period = TPERIOD_LQR;
	tp_lqr.deadline = TPERIOD_LQR * 0.8;
	tp_lqr.priority = 23;
	tp_lqr.dmiss = 0;

	ret = thread_create (&tp_lqr, &sched_lqr, attr_lqr, &tid_lqr, lqr_task);

	pthread_join (tid_mod, 0);
	pthread_join (tid_lqr, 0);
	pthread_join (tid_gfx, 0);
	
    pthread_attr_destroy (&attr_mod);	
	pthread_attr_destroy (&attr_lqr);
	pthread_attr_destroy (&attr_gfx);
	
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

void *gfx_task(void* arg)
{
	
struct task_par *tp;

double arr_graph_X[GRAPH_DATA_SIZE] = {0.0};
double arr_graph_Y[GRAPH_DATA_SIZE] = {0.0};
double arr_graph_Z[GRAPH_DATA_SIZE] = {0.0};

int col = makecol8(0, 255, 0);

	tp = (struct task_par *)arg;
	
	set_period (tp);

	start_allegro();
	
	rect(screen, 50, 50, GRAPH_XPOS_XCOORD, GRAPH_XPOS_YCOORD, col);
	rect(screen, 50, 250, GRAPH_YPOS_XCOORD, GRAPH_YPOS_YCOORD, col);
	rect(screen, 50, 450, GRAPH_ZPOS_XCOORD, GRAPH_ZPOS_YCOORD, col);
	
	textout_centre_ex(screen, font, "X position", 100, 40, col,-1);
	textout_centre_ex(screen, font, "Y position", 100, 240, col, -1);
	textout_centre_ex(screen, font, "Z position", 100, 440, col, -1);
	
	gfx_initialized = 1;
	
	while(keypressed() == 0)
	{
		pthread_mutex_lock(&mux_state);
		
		shift_and_append(arr_graph_X, GRAPH_DATA_SIZE, arr_state[3]);
		rectfill(screen, 51, 51, 149, 149, makecol(0,0,0));
		update_graph(screen, arr_graph_X, GRAPH_XPOS_XCOORD, GRAPH_XPOS_YCOORD);
		
		shift_and_append(arr_graph_Y, GRAPH_DATA_SIZE, arr_state[4]);
		rectfill(screen, 51, 251, 149, 349, makecol(0,0,0));
		update_graph(screen, arr_graph_Y, GRAPH_YPOS_XCOORD, GRAPH_YPOS_YCOORD);
		
		shift_and_append(arr_graph_Z, GRAPH_DATA_SIZE, arr_state[5]);
		rectfill(screen, 51, 451, 149, 549, makecol(0,0,0));
		update_graph(screen, arr_graph_Z, GRAPH_ZPOS_XCOORD, GRAPH_ZPOS_YCOORD);

		pthread_mutex_unlock(&mux_state);
		
		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: gfx_task()\n");

		//readkey();
		wait_for_period (tp);

	}
	
	close_allegro ();
	
	pthread_exit(0);	
}
