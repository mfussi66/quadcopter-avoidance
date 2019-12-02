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
int key_initialized = 0;
int gfx_enabled = 0;
int key_enabled = 0;

BITMAP* buffer_plt;
BITMAP* buffer_gfx;

char scan;

/* Semaphores */

pthread_mutex_t mux_forces;
pthread_mutex_t mux_state;
pthread_mutex_t mux_plt;

pthread_mutexattr_t muxattr;

/* tasks  declaration */
void* lqr_task (void* arg);
void *lin_model_task (void *arg);
void *key_task (void *arg);
void *gfx_task (void *arg);
void *plt_task (void *arg);

/* main thread */

int main (int argc, char **argv) 
{

struct sched_param sched_mod, sched_gfx, sched_lqr, sched_plt, sched_key;
struct task_par tp_mod, tp_gfx, tp_lqr, tp_plt, tp_key;
pthread_attr_t attr_mod, attr_gfx, attr_lqr, attr_plt, attr_key;
pthread_t tid_mod, tid_gfx, tid_lqr, tid_plt, tid_key;

int ret = 0;

    mutex_create (mux_forces, muxattr, 0, 100);
    mutex_create (mux_state, muxattr, 0, 100);
	mutex_create (mux_plt, muxattr, 0, 100);
	 
	pthread_mutexattr_destroy (&muxattr);
	
	//Create Graphics Thread
    
	tp_gfx.arg = 1;
	tp_gfx.period = TP_GFX;
	tp_gfx.deadline = TP_GFX * 0.8;
	tp_gfx.priority = 31;
	tp_gfx.dmiss = 0;

	ret = thread_create (&tp_gfx, &sched_gfx, attr_gfx, &tid_gfx, gfx_task);

   	//Create Keyboard Thread
    
	tp_key.arg = 2;
	tp_key.period = TP_KEY;
	tp_key.deadline = TP_KEY * 0.9;
	tp_key.priority = 35;
	tp_key.dmiss = 0;

	ret = thread_create (&tp_key, &sched_key, attr_key, &tid_key, key_task);


    //Create Dynamic Model Thread
    
	tp_mod.arg = 3;
	tp_mod.period = TP_MODEL;
	tp_mod.deadline = TP_MODEL * 0.9;
	tp_mod.priority = 20;
	tp_mod.dmiss = 0;

	ret = thread_create (&tp_mod, &sched_mod, attr_mod, &tid_mod, lin_model_task);
	
    //Create LQR Thread
    
	tp_lqr.arg = 4;
	tp_lqr.period = TP_LQR;
	tp_lqr.deadline = TP_LQR * 0.9;
	tp_lqr.priority = 21;
	tp_lqr.dmiss = 0;

	ret = thread_create (&tp_lqr, &sched_lqr, attr_lqr, &tid_lqr, lqr_task);
    
    //Create Plotting Thread
    
	tp_plt.arg = 5;
	tp_plt.period = TP_PLOTS;
	tp_plt.deadline = TP_PLOTS * 0.9;
	tp_plt.priority = 38;
	tp_plt.dmiss = 0;

	ret = thread_create (&tp_plt, &sched_plt, attr_plt, &tid_plt, plt_task);
    
    pthread_join (tid_plt, 0);
    pthread_join (tid_lqr, 0);
    pthread_join (tid_mod, 0);
    pthread_join (tid_key, 0);
	pthread_join (tid_gfx, 0);

    pthread_attr_destroy (&attr_plt);
	pthread_attr_destroy (&attr_lqr);
    pthread_attr_destroy (&attr_mod);	
    pthread_attr_destroy (&attr_key);
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

	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lin_model_task()\n");
		
		wait_for_period (tp);
	}while(key_enabled == 0 && gfx_enabled == 0);
    
    read_matrix_file("A.bin", A);
	read_matrix_file("B.bin", B);
    
    printf("Dynamic model task started\n");

    do{
        		
        printf("Dynamic model is working\n");
               
		pthread_mutex_lock (&mux_forces);
		
		for(int i = 0; i < SIZE_U; i++)
			gsl_vector_set(forces,i, arr_forces[i]);

		pthread_mutex_unlock (&mux_forces);

		quad_linear_model(forces, A, B, state);
		
        printf("Dynamic model is working\n");
        
		pthread_mutex_lock (&mux_state);
        
        printf("Dynamic model is working\n");
        
		for(int i = 0; i < SIZE_X; i++)
		{
			arr_state[i] = gsl_vector_get(state, i);
		}
		
		pthread_mutex_unlock (&mux_state);

		if (deadline_miss (tp))		
			printf ("DEADLINE MISS: lin_model_task()\n");
		
		wait_for_period (tp);

	}while(scan != KEY_ESC);
	
    if (gfx_enabled == 0 && key_enabled == 0)
    {
        gsl_vector_free(state);
        gsl_vector_free(forces);
        gsl_matrix_free(A);
        gsl_matrix_free(B);
    }
    
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
	
	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: lqr_task()\n");
		
		wait_for_period (tp);
	}while(key_enabled == 0 && gfx_enabled == 0);
    
    printf("LQR Controller task started\n");
	
    read_matrix_file("K.bin", K);
	
	gsl_vector_set(setpoint, 3, 1);
	gsl_vector_set(setpoint, 4, 1);
	gsl_vector_set(setpoint, 5, 1);
    
	do{
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
	}while(scan != KEY_ESC);
	
    if(key_enabled == 0 && gfx_enabled == 0)
    {
        gsl_vector_free(state);
        gsl_vector_free(forces);
        gsl_vector_free(setpoint);
        gsl_matrix_free(K);
    }
    
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

    gfx_enabled = 1;
    
	pthread_mutex_lock(&mux_plt);
    
	buffer_gfx = create_bitmap(SCREEN_W, SCREEN_H);
	
	col = makecol(0, 255, 0);
	
	rect(buffer_gfx, 5, 5, 560, 595, col);
	
	pthread_mutex_unlock(&mux_plt);
    
	do{
		pthread_mutex_lock(&mux_plt);
		
		blit(buffer_gfx,screen, 0, 0, 0, 0, SCREEN_W, SCREEN_H);

		pthread_mutex_unlock(&mux_plt);
		
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: gfk_task()\n");
		
		wait_for_period (tp);
        
	}while(key_enabled == 1);
	
    close_allegro ();
    
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
	
	
	do{
		if (deadline_miss (tp))
			printf ("DEADLINE MISS: plt_task()\n");
		
		wait_for_period (tp);
	}while(key_enabled == 0 && gfx_enabled == 0);
    
    printf("Plotting task started\n");
	
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
	
	
	do{
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
    
    struct task_par *tp;
	

    
    tp = (struct task_par *)arg;
	
	set_period (tp);
    
    key_enabled = 1;

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
