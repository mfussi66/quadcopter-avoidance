/* --------------------------
         REAL TIME SYSTEMS
         OBSTACLE AVOIDANCE
           Mattia Fussi

   MODEL and CONTROL SOURCE
 -------------------------- */

#include "model.h"

/* Model */

/*
 * Function: error computation
 * ---------------------------
 * Computes error between setpoint and measured state
 * allocates a new vector so that gsl_vector_sub() does not
 * overwrite the first argument
 */
void compute_error(double* sp, double* x, double* e, double* e_old, int size) {
  for (int i = 0; i < size; i++) {
    e_old[i] = e[i];
    e[i] = sp[i] - x[i];
  }
}
/*
 * Function: error rotation
 * ---------------------------
 * Rotates the error vector towards the body frame on
 * the yaw axis
 */
void rotate_error(double* e, double yaw) {
  double e_tmp[3] = {0.0};

  memcpy(e_tmp, e, sizeof(double) * 3);

  // printf("e_0:%f, e_1:%f\n", e[0], e[1]);

  e[0] = e_tmp[0] * cos(yaw) + e_tmp[1] * sin(yaw);
  e[1] = -e_tmp[0] * sin(yaw) + e_tmp[1] * cos(yaw);
  e[2] = e[2];
  // printf("e0:%f, e1:%f\n", e[0], e[1]);
}

/*
 * Function: compute euclidean distance
 * ---------------------------
 *
 */
double compute_pos_dist(double* v1, double* v2) {
  double x = v1[0] - v2[0];
  double y = v1[1] - v2[1];
  // double z = gsl_vector_get(v1, 5) - gsl_vector_get(v2, 5);

  return sqrt(pow2(x) + pow2(y));
  // return sqrt(pow2(x) + pow2(y) + pow2(z));
}

/*
 * Function: setpoint getter and computation
 * ---------------------------
 * Gets the next waypoint in the vector
 */
void next_setpoint(double* sp, WPoint* wp, int wp_size, int* wp_flags) {
  WPoint xy_setpoint;

  if (wp_size >= MAX_WPOINTS) return;

  for (int i = 1; i <= wp_size; i++) {
    if (wp_flags[i] == 0) {
      // printf("WP %f %f \n", wp[i].x, wp[i].y);
      xy_setpoint.x = (wp[i].x - ENV_OFFSET_X) / ENV_SCALE;
      xy_setpoint.y = (ENV_OFFSET_Y - wp[i].y) / ENV_SCALE;
      break;
    }
  }

  sp[0] = xy_setpoint.x;
  sp[1] = xy_setpoint.y;
}

/*
 * Function: Position PID
 * ---------------------------
 * Implements a simple PD controller for position
 * control
 */
void pid_xyz_control(double* e, double* e_prev, double dt, double* u,
                     double* vel_sp, double* p, double* d, int avoid) {
  if (avoid == 0) {
    // X velocity
    vel_sp[0] = e[0] * p[0] + d[0] * (e[0] - e_prev[0]) / dt;
    // Y velocity
    vel_sp[1] = e[1] * p[1] + d[1] * (e[1] - e_prev[1]) / dt;
  }
  // Vertical Thrust
  u[3] = e[2] * p[2] + d[2] * (e[2] - e_prev[2]) / dt;
}

/*
 * Function: Position PID
 * ---------------------------
 * Implements a simple P controller for velocity control
 */
void pid_vel_control(double* e, double* e_prev, double dt, double* rp_sp,
                     double* p, double* d) {
  // roll
  rp_sp[0] = e[1] * p[6] + d[6] * (e[1] - e_prev[1]) / dt;
  // pitch
  rp_sp[1] = e[0] * p[7] + d[7] * (e[0] - e_prev[0]) / dt;
}

/*
 * Function: Forces PD control
 * ---------------------------
 * Implements a PD control that gives the control
 * outputs to the system: RPY torques and vertical thrust
 */
void pid_rpy_control(double* e, double* e_prev, double dt, double* u, double* p,
                     double* d) {
  // tau roll
  u[0] = e[0] * p[3] + d[3] * (e[0] - e_prev[0]) / dt;
  // tau pitch
  u[1] = e[1] * p[4] + d[4] * (e[1] - e_prev[1]) / dt;
  // tau yaw
  u[2] = e[2] * p[5] + d[5] * (e[2] - e_prev[2]) / dt;
}

/*
 * Function: Linearized system equation
 * ---------------------------
 * Implements the linearized quadcopter dynamic equations
 * Before position integration the step is rotated in the
 * world frame to accumulate correctly
 */
void lin_model(double* u, double* x, double dt) {
  double deltax, deltay;

  x[6] += u[0] / Jxx * dt;     // p
  x[0] += x[6] * dt * 0.5;     // roll

  x[7] += u[1] / Jyy * dt;     // q
  x[1] += x[7] * dt * 0.5;     // pitch

  x[8] += u[2] / Jzz * dt;     // r
  x[2] += x[8] * dt * 0.5;     // yaw

  x[9] += 9.81 * x[1];         // u
  x[10] += 9.81 * x[0];        // v
  x[11] += u[3] / M * dt;      // w

  deltax = x[9] * dt;          // deltax
  deltay = x[10] * dt;         // deltay

  x[3] += deltax * cos(x[2]) - deltay * sin(x[2]); // x
  x[4] += deltax * sin(x[2]) + deltay * cos(x[2]); // y
  x[5]  += x[11] * dt * 0.5;   // z
}

/*
 * Function: Laser scanner initializer
 * ---------------------------
 * Initializes the array of traces taking aperture,
 * number of beams and initial pose of the quadcopter
 */
void init_laser_scanner(Trace* tr, int n, double aperture, double* init_pose) {
  double angle = aperture / (n - 1);

  for (int i = 0; i < n; i++) {
    tr[i].x = BEAM_DMAX;
    tr[i].y = BEAM_DMAX;
    tr[i].z = BEAM_DMAX;
    tr[i].theta = 0.0;
    tr[i].d = BEAM_DMAX;
    // printf("beam %d, x: %f, y: %f \n", i,tr[i].x,tr[i].y);
  }
}

/*
 * Function: Get distances from laser scanner
 * ---------------------------
 * Computes distances from obstacles simulating a laser scanner,
 * for each beam at a specific aperture looks for green pixels at a certain
 * increasing distance step
 */
void get_laser_distances(BITMAP* bmp, Trace* tr, double* pose, double spread,
                         double n) {
  float d = BEAM_DMIN;
  double yaw = pose[2];
  Trace trace_mt = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};
  Trace trace_px = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};

  for (int i = 1; i <= N_BEAMS; i++) {
    d = BEAM_DMIN;
    while (d <= BEAM_DMAX) {
      trace_mt.x = d * cos(deg2rad((double)ANGLE_RES * i) + yaw - M_PI / 2);
      trace_mt.y = d * sin(deg2rad((double)ANGLE_RES * i) + yaw - M_PI / 2);

      trace_px.x = ENV_OFFSET_X + ENV_SCALE * (pose[3] + trace_mt.x);
      trace_px.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + trace_mt.y);

      tr[i - 1].d = d;
      tr[i - 1].x = trace_mt.x;
      tr[i - 1].y = trace_mt.y;
      tr[i - 1].theta = deg2rad((double)ANGLE_RES * i) + yaw - M_PI / 2;

      if (getpixel(bmp, (int)trace_px.x, (int)trace_px.y) == makecol(0, 255, 0))
        break;

      d += BEAM_DSTEP;
    }
  }
}

/*
 * Function: Get the shortest beam
 * ---------------------------
 * Takes the array of beams of the laser reader and returns
 * the shortest beam
 */
Trace get_shortest_beam(Trace* tr, double threshold) {
  Trace force_max;
  int i_max = 0;
  double trace_min = BEAM_DMAX;
  force_max.x = 0.0;
  force_max.y = 0.0;
  force_max.z = 0.0;

  for (int i = 0; i < N_BEAMS; i++) {
    if (tr[i].d < trace_min) {
      i_max = i;
      trace_min = tr[i].d;
    }
  }

  force_max.d = tr[i_max].d;
  force_max.x = tr[i_max].x;
  force_max.y = tr[i_max].y;
  force_max.theta = tr[i_max].theta;

  return force_max;
}

/*
 * Function: Set avoidance mode
 * ---------------------------
 * Sets the appropriate obstacle avoidance mode according
 * to the angle and amplitude of the shortest beam wrt to
 * the drone
 */
int set_avoid_mode(Trace* tr, double* pose, WPoint* target, int* turn_dir,
                   int old_mode, int old_turn_dir) {
  int i_nearest = 0;
  double a_nearest = M_PI;
  double direct = atan2_safe(target->y - pose[4], target->x - pose[3]);
  double dist = sqrt(pow2(target->x - pose[3]) + pow2(target->y - pose[4]));
  Trace min_trace;
  double angle_diff = 0.0;
  int mode = 0;

  min_trace = get_shortest_beam(tr, 1);

  // take the beam closest to the setpoint
  for (int i = 0; i < N_BEAMS; i++) {
    if (fabs(anglediff_safe(direct, tr[i].theta)) < fabs(a_nearest)) {
      i_nearest = i;
      a_nearest = anglediff_safe(direct, tr[i].theta);
    }
  }

  angle_diff = anglediff_safe(min_trace.theta, pose[2]);

  // printf("min %d %f\n", i_nearest, rad2deg(angle_diff));

  if (angle_diff < 0)
    *turn_dir = 1;
  else
    *turn_dir = -1;

  // Emergency avoidance mode
  if (min_trace.d <= 0.6) {
    return 1;
  }

  if (old_mode == 1 && min_trace.d < 1) {
    *turn_dir = old_turn_dir;
    return 1;
  }

  if (old_mode == 4 && min_trace.d < 1.5) {
    *turn_dir = old_turn_dir;
    return 4;
  }

  if (old_mode == 3 && min_trace.d < 1.5) {
    *turn_dir = old_turn_dir;
    return 3;
  }

  // If the the goal is closer than the shortest beam
  if (dist <= tr[i_nearest].d) {
    return 0;
  }

  // Sets the primary avoidance modes
  if (old_mode == 0 && min_trace.d < 1) {
    // if the obstacle if more or less in front of the quadc.
    if (fabs(angle_diff) <= deg2rad(20)) {
      return 3;
    }
    // rotate 1
    else if (fabs(angle_diff) > deg2rad(20)) {
      return 4;
    }
  }

  return mode;
}

/*
 * Function: Set velocity setpoints
 * ---------------------------
 * Sets the appropriate velocity setpoints to avoid
 * obstacles according to the modes chosen
 */
void compute_avoid_sp(double* pose, int rn, int mode, double* vel_sp,
                      double* yaw_sp, int turn_dir) {
  double vel_rot[2] = {0.0};

  if (mode == 4) {  //*yaw_sp = pose[2];
    vel_sp[1] = turn_dir * 0.3;
    vel_sp[0] = 0.0;
  } else if (mode == 1) {
    //*yaw_sp = pose[2];
    vel_sp[1] = turn_dir * 0.5;
    vel_sp[0] = 0.0;
  } else if (mode == 3) {
    *yaw_sp = pose[2];
    vel_sp[1] = rn * 0.3;
    vel_sp[0] = 0.0;
  }
}

/*
 * Function: Check collision
 * ---------------------------
 * checks if the baricenter of the quadcopter is inside the
 * environment or inside an obstalce
 */
int chk_collisions(double* pose, Obstacle* obs, int n_obs) {
  double x = pose[3];
  double y = pose[4];

  Obstacle env;

  env.x1 = 0.0;
  env.y1 = 0.0;
  env.x2 = (WIDTH_SCREEN - 266) / ENV_SCALE;
  env.y2 = (HEIGHT_SCREEN - 5) / ENV_SCALE;

  int result = 0;

  // check collision with env boundaries
  if (x <= env.x1 || x >= env.x2 || y <= env.y1 || y >= env.y2) result = 1;

  // check collision with obstacles
  for (int i = 0; i < n_obs; i++) {
    if (x >= obs[i].x1 && x <= obs[i].x2 && y <= obs[i].y1 && y >= obs[i].y2)
      result = 1;
  }

  return result;
}

/*
 * Function: Init controller gains
 * ---------------------------
 */
void init_gains(double* p, double* d, double* p_df, double* d_df) {
  // proportional gains
  p[0] = 1.5e-1;  // x
  p[1] = 1.5e-1;  // y
  p[2] = 1e-4;    // z
  p[3] = 1e-6;    // roll
  p[4] = 1e-6;    // pitch
  p[5] = 1e-6;    // yaw
  p[6] = 7e-2;    // u
  p[7] = 7e-2;    // v
  p[8] = 1e-2;    // w

  // derivative gains
  d[0] = 0;     // x
  d[1] = 0;     // y
  d[2] = 6e-3;  // z
  d[3] = 1e-3;  // roll
  d[4] = 1e-3;  // pitch
  d[5] = 8e-3;  // yaw
  d[6] = 0;     // u
  d[7] = 0;     // v
  d[8] = 0;     // w

  memcpy(p_df, p, sizeof(double) * SIZE_PID);
  memcpy(d_df, d, sizeof(double) * SIZE_PID);
}

/*
 * Function: Modify controller gains
 * ---------------------------
 */
void adjust_gain(double* p, double* d, int mode, int updown) {
  int idx = mode - 2;

  if (idx < 0 || idx >= SIZE_PID) return;

  if (updown == 0) {
    p[idx] *= 2;
    // d[idx] *= 2;
  } else if (updown == 1) {
    p[idx] /= 2;
    // d[idx] /= 2;
  }
}

/*
 * Function: Reset gains to default values
 * ---------------------------
 */
void reset_gains(double* p, double* d, double* p_df, double* d_df) {
  memcpy(p, p_df, sizeof(double) * SIZE_PID);
  memcpy(d, d_df, sizeof(double) * SIZE_PID);
}

/*
 * Helper Functions
 * ---------------------------
 */
double deg2rad(double n) { return n * M_PI / 180.0; }

double rad2deg(double n) { return n * 180.0 / M_PI; }

double pow2(double n) { return n * n; }

double atan2_safe(double y, double x) {
  double r = atan2(y, x);

  if (r < 0) return (2 * M_PI + r);

  return r;
}

double anglediff_safe(double sp, double theta) {
  double res = sp - theta;

  if (res < M_PI && res > -M_PI) {
    res = sp - theta;
  } else if (res > M_PI) {
    res -= 2 * M_PI;
  } else if (res <= -M_PI) {
    res += 2 * M_PI;
  }

  return res;
}

double get_uniform_num() {
  double n = rand();

  return n / RAND_MAX;
}
