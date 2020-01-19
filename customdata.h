/* Data structures and constants definition */

#ifndef CUSTOMDATA_H
#define CUSTOMDATA_H

/* --- Project Constants --- */

#define NT 4
#define N_SAMPLES 2

#define SIZE_X 12
#define SIZE_U 4
#define SIZE_Y 6

#define ENV_OFFSET_X 20
#define ENV_OFFSET_Y 575
#define ENV_SCALE 10

#define MAX_WPOINTS 5
#define OBS_NUM 2

#define OFFSET_LASER 0
#define N_BEAMS 5
#define BEAM_DMIN 3
#define BEAM_DMAX 10
#define BEAM_DSTEP 0.01

#define COL_GREEN 48

#define PLT_FRAME_SIZE 100
#define PLT_DATA_SIZE 25
#define PLT_SCALE_X PLT_DATA_SIZE
#define PLT_STEP 4
#define PLT_SCALE_Y 1
#define PLT_XPOS_XCOORD 795
#define PLT_XPOS_YCOORD 385
#define PLT_YPOS_XCOORD 795
#define PLT_YPOS_YCOORD 490
#define PLT_ZPOS_XCOORD 795
#define PLT_ZPOS_YCOORD 595

/* --- Project Structures --- */

typedef struct
{
	double x;
	double y;
	double z;
} Trace;

typedef struct
{
	double x1;
	double y1;
	double x2;
	double y2;
} Obstacle;

typedef struct
{
	double x;
	double y;
} WPoint;

typedef struct
{
	int start;
	int end;
} Valley;

typedef struct servo_par {
	int mid;				/* motor id */
	float x_sp;			/* angular position set point */
	float xdot_sp;			/* angular speed set point */
	float x[3];			/* angular position array */
	float xdot[3];			/* angular speed array */
	float u[3];			         /* control action array */
	float e[N_SAMPLES];			/* error array */
	float integral;		     /* integral of error component */
	float derivative;		/* derivative of error component */
} servo_par;

typedef struct angular_body {
	float p;
	float q;
	float r;
} angular_body;

typedef struct angular {
	float r;
	float p;
	float y;
} angular;

typedef struct linear {
	float x;
	float y;
	float z;
} linear;

typedef struct vect6d {
	linear linear;
	angular angular;
} vect6d;



typedef struct status {
	vect6d pose;
	float traces[21];
} status;

typedef struct speeds {
	float w1;				/* Rotational speed motor 1 */
	float w2;				/* Rotational speed motor 2 */
	float w3;				/* Rotational speed motor 3 */
	float w4;				/* Rotational speed motor 4 */
} speeds;

#endif /* CUSTOMDATA_H_INCLUDED */
