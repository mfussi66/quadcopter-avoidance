/* Data structures and constants definition */

#ifndef CUSTOMDATA_H_INCLUDED_
#define CUSTOMDATA_H_INCLUDED_

/* --- Project Constants --- */

#define SERVER "169.254.170.240" 		/* localhost address */
#define BUFLEN 512  					/* Max length of buffer */
#define PORT 8888   					/* The port for listening */

#define TPERIOD_COMM 2500
#define TPERIOD_POS 100
#define TPERIOD_RPY 70
#define TPERIOD_GRAPHICS 250
#define TPERIOD_MODEL 10
#define TPERIOD_LQR 100

#define NT 4
#define N_SAMPLES 2
/*
#define kp 0.09
#define ki 0.00002
#define kd 0.0 */

#define SIZE_X 12
#define SIZE_U 4
#define SIZE_Y 6

#define mass		0.42
#define WH 		    617658 //4106
#define gravity 	9.81
#define Kf 		    2.7 * 0.000000000001
#define Km 		6.63 * 0.00000000000001
#define L 		0.3677
#define Ixx		0.0005
#define Iyy		0.0005
#define Izz		0.001

#define kp_roll 	100
#define kp_pitch 	100
#define kp_yaw 		100

#define kd_roll 	1000
#define kd_pitch 	1000
#define kd_yaw	 	1000

#define kp_x	 	0.01
#define kp_y 		0.01
#define kp_z 		0.01

#define kd_x 		0.1
#define kd_y 		0.1
#define kd_z 		0.185

/* --- Project Structures --- */

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
