/* --- REAL TIME MAIN --- */

#include "task.h"
#include "model.h"
#include "graphics.h"
#include "customdata.h"

/* global vars */

double arr_state[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double arr_forces[4] = {0,0,0,0}; 
double arr_error[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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
	tp_gfx.deadline = TPERIOD_GRAPHICS * 0.9;
	tp_gfx.priority = 21;
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
// 	
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

double cX[2] = {50, 125};
double cX_prev[2] = {50, 125};

double cY[2] = {50, 325};
double cY_prev[2] = {50, 325};

double cZ[2] = {50, 525};
double cZ_prev[2] = {50, 525};

int step = 1;
int colour = 0;

	tp = (struct task_par *)arg;
	set_period (tp);

	start_allegro();
	
	gfx_initialized = 1;
	
	while(keypressed() == 0)
	{
		colour = makecol(0, 255, 0);
		
		rect(screen, 50, 50, 750, 150, colour);
		rect(screen, 50, 250, 750, 350, colour);
		rect(screen, 50, 450, 750, 550, colour);
		
		pthread_mutex_lock(&mux_state);
		
		cZ_prev[0] = cZ[0];
		cZ_prev[1] = cZ[1];

		cX_prev[0] = cX[0];
		cX_prev[1] = cX[1];

		cY_prev[0] = cY[0];
		cY_prev[1] = cY[1];
			
		cZ[0] = 50 + step;
		cZ[1] = 50 + 75 - (int)(arr_state[5] * 50);

		cX[0] = 50 + step;
		cX[1] = 325 - (int)(arr_state[4] * 50);

		cY[0] = 50 + step;
		cY[1] = 525 - (int)(arr_state[3] * 50);		
		
		textout_centre_ex(screen,font,"Quadcopter X position",400, 40, colour,-1);
		textout_centre_ex(screen,font,"Quadcopter Y position",400, 240, colour, -1);
		textout_centre_ex(screen,font,"Quadcopter Z position",400, 440, colour, -1);
		
		fastline(screen, cZ_prev[0], cZ_prev[1],cZ[0],cZ[1], colour);
		fastline(screen, cY_prev[0], cY_prev[1],cY[0],cY[1], colour);
		fastline(screen, cX_prev[0], cX_prev[1],cX[0],cX[1], colour);
		
		//putpixel(screen, cZ[0], cZ[1], colour);
		pthread_mutex_unlock(&mux_state);
		
		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: gfx_task()\n");
		
		step ++;
		//readkey();
		wait_for_period (tp);

	}
	
	close_allegro ();
	
	pthread_exit(0);	
}
