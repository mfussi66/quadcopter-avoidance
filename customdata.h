/* --------------------------
         REAL TIME SYSTEMS
         OBSTACLE AVOIDANCE
           Mattia Fussi

 COMMON CONSTANTS AND STRUCTS
 -------------------------- */

#ifndef CUSTOMDATA_H
#define CUSTOMDATA_H

/* --- Project Constants --- */

#define SIZE_X 12
#define SIZE_U 4
#define SIZE_Y 6
#define SIZE_PID 9

#define WIDTH_SCREEN 1366
#define HEIGHT_SCREEN 768

#define ENV_OFFSET_X 20
#define ENV_OFFSET_Y 743
#define ENV_SCALE 50

#define MAX_WPOINTS 5
#define OBS_NUM 6

#define OFFSET_LASER 0
#define APERTURE 150
#define N_BEAMS 15
#define ANGLE_RES 10
#define BEAM_DMIN 0.4
#define BEAM_DMAX 5
#define BEAM_DSTEP 0.02

#define COL_GREEN 48

#define PLT_FRAME_SIZE 100
#define PLT_DATA_SIZE 25
#define PLT_SCALE_X PLT_DATA_SIZE
#define PLT_STEP 4
#define PLT_SCALE_Y 1
#define PLT_12_XCOORD WIDTH_SCREEN - 5
#define PLT_12_YCOORD HEIGHT_SCREEN - 215
#define PLT_22_XCOORD WIDTH_SCREEN - 5
#define PLT_22_YCOORD HEIGHT_SCREEN - 110
#define PLT_32_XCOORD WIDTH_SCREEN - 5
#define PLT_32_YCOORD HEIGHT_SCREEN - 5
#define PLT_11_XCOORD WIDTH_SCREEN - 120
#define PLT_11_YCOORD HEIGHT_SCREEN - 215
#define PLT_21_XCOORD WIDTH_SCREEN - 120
#define PLT_21_YCOORD HEIGHT_SCREEN - 110
#define PLT_31_XCOORD WIDTH_SCREEN - 120
#define PLT_31_YCOORD HEIGHT_SCREEN - 5

#define TP_GFX 33
#define TP_PLOTS 80
#define TP_LSR 30
#define TP_MODEL 20
#define TP_KEY 60
#define TP_POINT 120

/* --- Project Structures --- */

typedef struct {
  double x;
  double y;
  double z;
  double d;
  double theta;
} Trace;

typedef struct {
  double x1;
  double y1;
  double x2;
  double y2;
} Obstacle;

typedef struct {
  double x;
  double y;
} WPoint;

typedef struct {
  double mass;
  double J_xx;
  double J_yy;
  double J_zz;
} DynParam;

#endif // CUSTOMDATA_H
