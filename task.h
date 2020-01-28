/* Header File for RTS project */

/* --- Include guard --- */

#ifndef TASK_H_INCLUDED_
#define TASK_H_INCLUDED_

/* --- Standard Libraries --- */

#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <string.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>

/* --- Definitions --- */

#define THREAD_MAX_NUM 6
/* --- Project functions --- */

struct task_par {
	int arg;				/* task argument */
	long wcet;				/* in microseconds */
	int period;				/* in milliseconds */
	int deadline;			/* relative (ms) */
	int priority;			/* in [0,99] */
	int dmiss;				/* n. of deadline misses */
	struct timespec at;		/* next activation time */
	struct timespec dl;		/* absolute deadline */
};

/* Task Management */

void set_task_params(struct task_par *tp, int arg, int per, int dl, int prio);

void set_period (struct task_par *tp);

void wait_for_period (struct task_par *tp);

void time_add_ms (struct timespec *t, int ms);

int time_cmp (struct timespec t1, struct timespec t2);

void eval_period(struct task_par *tp, int* arr_tp, int* sel_t, int t_idx, int *change_flag);

void time_copy (struct timespec *td, struct timespec ts);

int deadline_miss (struct task_par *tp);

void select_thread_tp(int* sel_t, int* tp_arr, int* sel_t_old_tp, int t_idx);

void cancel_thread_tp(int* sel_t, int* tp_arr, int* sel_t_old_tp);

void modify_thread_tp(int* sel_t, int* tp_arr, int val);

int thread_create (struct task_par *tp, struct sched_param *sp, pthread_attr_t att, pthread_t *tid, void* task);

int mutex_create (pthread_mutex_t mux, pthread_mutexattr_t matt, int prot, int ceiling);

/* Utilities */

void shift_and_append (double *array, int size, double new_element);

void write_to_file (const char* filename, const char* text, int toappend);

int read_matrix_file (const char* filename, gsl_matrix* M);

#endif /* TASK_H_INCLUDED_ */
