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
#define ENV_SCALE 50

#define MAX_WPOINTS 5
#define OBS_NUM 2

#define OFFSET_LASER 0
#define N_BEAMS 9
#define BEAM_DMIN 1
#define BEAM_DMAX 5
#define BEAM_DSTEP 0.005

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

#define TP_GFX 33
#define TP_PLOTS 125
#define TP_LSR 30
#define TP_MODEL 20
#define TP_POS 10
#define TP_RPY 10
#define TP_KEY 100
#define TP_POINT 80
#define TP_GAINS 200

/* --- Project Structures --- */

typedef struct
{
	double x;
	double y;
	double z;
	double theta;
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

/* 
 * Function: Utility functions for angle conversion
 * ---------------------------
 */
double deg2rad(double n)
{
	return n * M_PI / 180;	
}


double rad2deg(double n)
{
	return n * 180 / M_PI;	
}

double pow2(double n)
{
	return n * n;
}

double atan2_safe(double y, double x)
{
double r = atan2(y, x);

	if(r < 0) return (2 * M_PI + r);
	
	return r;
	
}

#endif /* CUSTOMDATA_H_INCLUDED */
