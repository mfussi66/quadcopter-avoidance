/* Function file for RTS Project */

#include "task.h"

/* --- Functions--- */

/* Task Management */

void set_task_params(struct task_par *tp, int arg, int per, int dl, int prio)
{
	tp->arg = arg;
	tp->period = per;
	tp->deadline = dl;
	tp->priority = prio;
	tp->dmiss = 0;
	
}

void set_period (struct task_par *tp)
{
struct timespec t;
	
	clock_gettime (CLOCK_MONOTONIC, &t);

	time_copy (& (tp->at), t);
	time_copy (& (tp->dl), t);

	time_add_ms (& (tp->at), tp->period);
	time_add_ms (& (tp->dl), tp->deadline);
};

void wait_for_period (struct task_par *tp)
{
	clock_nanosleep (CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp->at), NULL);
	
	time_add_ms (& (tp->at), tp->period);
	time_add_ms (& (tp->dl), tp->period);
};

void time_add_ms(struct timespec *t, int ms)
{

	t->tv_sec += ms / 1000;
	t->tv_nsec += (ms % 1000) * 1000000;
	if (t->tv_nsec > 1000000000) 
    {
		t->tv_nsec -= 1000000000;
		t->tv_sec += 1;
	}

};

int time_cmp(struct timespec t1, struct timespec t2)
{	
	
	if (t1.tv_sec > t2.tv_sec) return 1;
	if (t1.tv_sec < t2.tv_sec) return -1;
	if (t1.tv_nsec > t2.tv_nsec) return 1;
	if (t1.tv_nsec < t2.tv_nsec) return -1;
	return 0;

};

void eval_period(struct task_par *tp, int* arr_tp, int* sel_t, int t_idx, int *change_flag)
{
	if(!*change_flag)
		return;
	
	if(*sel_t != t_idx)
		return;
	
	if(arr_tp[t_idx] == tp->period)
		return;
	
	struct timespec t;
	
	tp->period = arr_tp[t_idx];
	tp->deadline = arr_tp[t_idx];
	
	clock_gettime (CLOCK_MONOTONIC, &t);

	time_copy (& (tp->at), t);
	time_copy (& (tp->dl), t);

	time_add_ms (& (tp->at), tp->period);
	time_add_ms (& (tp->dl), tp->deadline);
	
	*change_flag = 0;
	*sel_t = 99;
	
	printf("Task %d period changed to %d\n", t_idx, tp->period);
	
};

void time_copy (struct timespec *td, struct timespec ts)
{

	td->tv_sec = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;

	return;

};

int deadline_miss (struct task_par *tp)
{
struct timespec now;
	
	clock_gettime (CLOCK_MONOTONIC, &now);

	if (time_cmp (now, tp->dl) > 0) {
		tp->dmiss++;
		return 1;
	}

	return 0;
};

int thread_create (struct task_par *tp, struct sched_param *sp, pthread_attr_t att, pthread_t *tid, void* task)
{
int ret = -1;

	pthread_attr_init (&att);
	pthread_attr_setinheritsched (&att, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&att, SCHED_FIFO);

	sp->sched_priority = tp->priority;
	pthread_attr_setschedparam (&att, sp);
	ret = pthread_create (tid, &att, task, tp);
	
	if (ret != 0)
    {
		printf ("!! - Error %d creating task() - !!\n", ret);
        printf("%s\n", strerror(ret));
    }
	
	return ret;

};

int mutex_create (pthread_mutex_t mux, pthread_mutexattr_t matt, int prot, int ceiling)
{

int result = -1;

	pthread_mutexattr_init(&matt);

	switch(prot)
	{
		case 1 :
			pthread_mutexattr_setprotocol (&matt, PTHREAD_PRIO_INHERIT);
		case 2 :
			pthread_mutexattr_setprotocol (&matt, PTHREAD_PRIO_PROTECT);
			pthread_mutexattr_setprioceiling (&matt, ceiling);
		default:
			pthread_mutexattr_setprotocol (&matt, PTHREAD_PRIO_NONE);
	}

	result = pthread_mutex_init (&mux, &matt);

	return result;

};

void select_thread_tp(int* sel_t, int* tp_arr, int* sel_t_old_tp, int t_idx)
{	
	if(t_idx >= THREAD_MAX_NUM)
	{
		printf("Thread %d selected does not exist\n", t_idx);
		*sel_t = 99;
		return;
	}

	*sel_t = t_idx;
	*sel_t_old_tp = tp_arr[*sel_t];
	printf("Sel t: %d - tp: %d\n", *sel_t, tp_arr[*sel_t]);
	
};

void cancel_thread_tp(int* sel_t, int* tp_arr, int* sel_t_old_tp)
{
	if(*sel_t >= 99) return;
	
	printf("Canc t: %d to tp %d\n", *sel_t, *sel_t_old_tp);
	
	tp_arr[*sel_t] = *sel_t_old_tp;
	*sel_t = 99;
}

void modify_thread_tp(int* sel_t, int* tp_arr, int val)
{
	if(*sel_t >= 99) return;
	
	if (tp_arr[*sel_t] + val <= 10 )
		tp_arr[*sel_t] = 10;
	else
		tp_arr[*sel_t] += val;
	
	printf("Mod t: %d p: %d \n", *sel_t, tp_arr[*sel_t]);
	
}

void shift_and_append (double *array, int size, double new_element)
{
	
	for(int i = 1; i < size; i++)
		array[i-1] = array[i];
	
	array[size-1] = new_element;
	
}

void write_to_file (const char* filename, const char* text, int toappend)
{
	FILE *f_pointer;

	if (toappend == 0)
	{
		f_pointer = fopen (filename, "w");
	}
	else
	{
		f_pointer = fopen (filename, "a");
	}

	fputs (text, f_pointer);
	
	fclose (f_pointer);

	return;

}

int read_matrix_file (const char* filename, gsl_matrix* M)
{
	FILE* ptr;
	int ret = 0;
	
	ptr = fopen(filename, "rb");
	
	ret = gsl_matrix_fread(ptr, M);
	
	if (ret !=0)
		printf("Error reading matrix file!\n");
	
	fclose(ptr);
	
	return ret;
}
