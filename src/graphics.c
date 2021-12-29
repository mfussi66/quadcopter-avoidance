/* --------------------------
         REAL TIME SYSTEMS
         OBSTACLE AVOIDANCE
           Mattia Fussi

          GRAPHICS SOURCE
 -------------------------- */

#include "graphics.h"

/* Graphics */

/*
 * Function:Init allegro engine and screen
 * ---------------------------
 */
void start_allegro(int mode) {
  allegro_init();

  set_color_depth(8);

  set_gfx_mode(mode, WIDTH_SCREEN, HEIGHT_SCREEN, 0, 0);
  install_keyboard();
  install_mouse();

  // enable_hardware_cursor();

  clear_to_color(screen, 0);

  printf("Allegro correctly initialized\n");

  return;
}

/*
 * Function: Closes allegro
 * ---------------------------
 */
void close_allegro(void) {
  allegro_exit();

  printf("Graphics: Allegro is closed\n");
}

/*
 * Function: Draws gui
 * ---------------------------
 * Draws all the static gui objects
 */
void build_gui(BITMAP* bmp, FONT* font, int col) {
  // Environment frame
  rect(bmp, 5, 5, WIDTH_SCREEN - 266, HEIGHT_SCREEN - 5, col);
  rect(bmp, 6, 6, WIDTH_SCREEN - 267, HEIGHT_SCREEN - 6, col);
  rect(bmp, 7, 7, WIDTH_SCREEN - 268, HEIGHT_SCREEN - 7, col);
  rect(bmp, 8, 8, WIDTH_SCREEN - 269, HEIGHT_SCREEN - 8, col);

  // Rectangles that contain the plots
  rect(bmp, WIDTH_SCREEN - 105, HEIGHT_SCREEN - 315, WIDTH_SCREEN - 5,
       HEIGHT_SCREEN - 215, col);
  rect(bmp, WIDTH_SCREEN - 105, HEIGHT_SCREEN - 210, WIDTH_SCREEN - 5,
       HEIGHT_SCREEN - 110, col);
  rect(bmp, WIDTH_SCREEN - 105, HEIGHT_SCREEN - 105, WIDTH_SCREEN - 5,
       HEIGHT_SCREEN - 5, col);
  rect(bmp, WIDTH_SCREEN - 220, HEIGHT_SCREEN - 315, WIDTH_SCREEN - 120,
       HEIGHT_SCREEN - 215, col);
  rect(bmp, WIDTH_SCREEN - 220, HEIGHT_SCREEN - 210, WIDTH_SCREEN - 120,
       HEIGHT_SCREEN - 110, col);
  rect(bmp, WIDTH_SCREEN - 220, HEIGHT_SCREEN - 105, WIDTH_SCREEN - 120,
       HEIGHT_SCREEN - 5, col);

  // Plots labels
  textout_centre_ex(bmp, font, "R", WIDTH_SCREEN - 110, PLT_12_YCOORD - 50, col,
                    -1);
  textout_centre_ex(bmp, font, "P", WIDTH_SCREEN - 110, PLT_22_YCOORD - 50, col,
                    -1);
  textout_centre_ex(bmp, font, "Z", WIDTH_SCREEN - 110, PLT_32_YCOORD - 50, col,
                    -1);
  textout_centre_ex(bmp, font, "Tau_X", WIDTH_SCREEN - 242, PLT_11_YCOORD - 50,
                    col, -1);
  textout_centre_ex(bmp, font, "Tau_Y", WIDTH_SCREEN - 242, PLT_21_YCOORD - 50,
                    col, -1);
  textout_centre_ex(bmp, font, "Tau_Z", WIDTH_SCREEN - 242, PLT_31_YCOORD - 50,
                    col, -1);

  // Editable data
  textout_ex(bmp, font, "Select with number/letter", 1141, 10, col, -1);
  textout_ex(bmp, font, "Change with Up/Down arrows", 1141, 20, col, -1);
  textout_ex(bmp, font, "PERIODS", 1141, 35, col, -1);
  textout_ex(bmp, font, "[0] Graphics", 1141, 45, col, -1);
  textout_ex(bmp, font, "[1] KeyBoard", 1141, 55, col, -1);
  textout_ex(bmp, font, "[2] Waypoints", 1141, 65, col, -1);
  textout_ex(bmp, font, "[3] Model", 1141, 75, col, -1);
  textout_ex(bmp, font, "[4] Laser", 1141, 85, col, -1);
  textout_ex(bmp, font, "[5] Plots", 1141, 95, col, -1);
  textout_ex(bmp, font, "[ENTER] confirm period", 1141, 105, col, -1);
  textout_ex(bmp, font, "[BACKSPACE] cancel period", 1141, 115, col, -1);

  textout_ex(bmp, font, "GAINS", 1141, 153, col, -1);
  textout_ex(bmp, font, "Kp      Kd", 1261, 153, col, -1);
  textout_ex(bmp, font, "[X] Xpos", 1141, 165, col, -1);
  textout_ex(bmp, font, "[Y] Ypos", 1141, 175, col, -1);
  textout_ex(bmp, font, "[Z] Altitude", 1141, 185, col, -1);
  textout_ex(bmp, font, "[R] Roll", 1141, 195, col, -1);
  textout_ex(bmp, font, "[P] Pitch", 1141, 205, col, -1);
  textout_ex(bmp, font, "[A] Yaw", 1141, 215, col, -1);
  textout_ex(bmp, font, "[U] Xvel", 1141, 225, col, -1);
  textout_ex(bmp, font, "[V] Yvel", 1141, 235, col, -1);
  textout_ex(bmp, font, "[W] Zvel", 1141, 245, col, -1);
  textout_ex(bmp, font, "[C] Reset gains to default", 1141, 255, col, -1);
}

/*
 * Function: Show fancy exit text
 * ---------------------------
 */
void draw_exit_screen(BITMAP* bmp, int col) {
  textout_centre_ex(bmp, font, "_________________________", WIDTH_SCREEN / 2,
                    HEIGHT_SCREEN / 2 - 13, col, -1);
  textout_centre_ex(bmp, font, "   Simulation stopped    ", WIDTH_SCREEN / 2,
                    HEIGHT_SCREEN / 2, col, -1);
  textout_centre_ex(bmp, font, "  Thank you for playing  ", WIDTH_SCREEN / 2,
                    HEIGHT_SCREEN / 2 + 10, col, -1);
  textout_centre_ex(bmp, font, "_________________________", WIDTH_SCREEN / 2,
                    HEIGHT_SCREEN / 2 + 15, col, -1);
}

/*
 * Function: Generate obstacles
 * ---------------------------
 * Generates a vector with obstacle coordinates
 */
int gen_obstacles(Obstacle* arr_obstacles, int n_obs) {
  size_t n = (uint)n_obs;

  if (n == 0) return -1;
  
  arr_obstacles[0].x1 = (5 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[0].y1 = (-250 + ENV_OFFSET_Y) / ENV_SCALE;
  arr_obstacles[0].x2 = (250 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[0].y2 = (-400 + ENV_OFFSET_Y) / ENV_SCALE;

  arr_obstacles[1].x1 = (360 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[1].y1 = (-100 + ENV_OFFSET_Y) / ENV_SCALE;
  arr_obstacles[1].x2 = (560 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[1].y2 = (-200 + ENV_OFFSET_Y) / ENV_SCALE;

  arr_obstacles[2].x1 = (400 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[2].y1 = (-400 + ENV_OFFSET_Y) / ENV_SCALE;
  arr_obstacles[2].x2 = (450 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[2].y2 = (-450 + ENV_OFFSET_Y) / ENV_SCALE;

  arr_obstacles[3].x1 = (650 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[3].y1 = (-550 + ENV_OFFSET_Y) / ENV_SCALE;
  arr_obstacles[3].x2 = (700 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[3].y2 = (-600 + ENV_OFFSET_Y) / ENV_SCALE;

  arr_obstacles[4].x1 = (550 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[4].y1 = (-400 + ENV_OFFSET_Y) / ENV_SCALE;
  arr_obstacles[4].x2 = (650 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[4].y2 = (-450 + ENV_OFFSET_Y) / ENV_SCALE;

  arr_obstacles[5].x1 = (800 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[5].y1 = (-150 + ENV_OFFSET_Y) / ENV_SCALE;
  arr_obstacles[5].x2 = (950 - ENV_OFFSET_X) / ENV_SCALE;
  arr_obstacles[5].y2 = (-450 + ENV_OFFSET_Y) / ENV_SCALE;

  // 	arr_obstacles[6].x1 = (800 - ENV_OFFSET_X) / ENV_SCALE;
  // 	arr_obstacles[6].y1 = (-550 + ENV_OFFSET_Y) / ENV_SCALE;
  // 	arr_obstacles[6].x2 = (1000 - ENV_OFFSET_X) / ENV_SCALE;
  // 	arr_obstacles[6].y2 = (-650 + ENV_OFFSET_Y) / ENV_SCALE;
  //
  // 	arr_obstacles[7].x1 = (400 - ENV_OFFSET_X) / ENV_SCALE;
  // 	arr_obstacles[7].y1 = (-550 + ENV_OFFSET_Y) / ENV_SCALE;
  // 	arr_obstacles[7].x2 = (480 - ENV_OFFSET_X) / ENV_SCALE;
  // 	arr_obstacles[7].y2 = (-650 + ENV_OFFSET_Y) / ENV_SCALE;

  return 0;
}

/*
 * Function: Draws obstacles
 * ---------------------------
 */
void draw_obstacles(BITMAP* bmp, Obstacle* obs, int n_obs, int col) {
  int X_env[4] = {0};

  for (int i = 0; i < n_obs; i++) {
    X_env[0] = (int)(ENV_OFFSET_X + ENV_SCALE * (obs[i].x1));
    X_env[1] = (int)(ENV_OFFSET_Y - ENV_SCALE * (obs[i].y1));
    X_env[2] = (int)(ENV_OFFSET_X + ENV_SCALE * (obs[i].x2));
    X_env[3] = (int)(ENV_OFFSET_Y - ENV_SCALE * (obs[i].y2));

    rectfill(bmp, X_env[0], X_env[1], X_env[2], X_env[3], col);
  }
}

/*
 * Function: Add waypoint
 * ---------------------------
 * Adds the clicked environment waypoint to the
 * array of goals if it's inside the environment and
 * not on an obstacle
 */
void add_waypoint(BITMAP* bmp, WPoint* array, int* num, WPoint point) {
  Obstacle env;
  int n = *num + 1;

  env.x1 = 5 + 5;
  env.y1 = 5 + 5;
  env.x2 = (WIDTH_SCREEN - 266 - 5);
  env.y2 = (HEIGHT_SCREEN - 5 - 5);

  if (n >= MAX_WPOINTS) {
    printf("Max n. of waypoints reached (5)\nPress SPACE to start\n");
    return;
  }

  if (getpixel(bmp, point.x, point.y) == COL_GREEN) {
    printf("Cannot place waypoint on obstacle\n");
    return;
  }

  if (point.x < env.x1 || point.x > env.x2 || point.y < env.y1 ||
      point.y > env.y2) {
    printf("Cannot place waypoint outside environment\n");
    return;
  }

  if (n < MAX_WPOINTS - 1) printf("Click on waypoint %d\n", n + 1);

  array[n].x = point.x;
  array[n].y = point.y;

  *num = n;
}

/*
 * Function: Deletes a waypoint
 * ---------------------------
 * Deletes a waypoint by checking which is the closest
 * from the clicked point
 */
void delete_waypoint(BITMAP* bmp, WPoint* array, int* num, WPoint point) {
  int clicked_col;
  int n = *num;
  int min_dist_idx;
  int j = 0;
  double dist = 0;
  double xy_dist[2] = {0};
  double curr_max_dist = 0;
  WPoint tmp_new_array[MAX_WPOINTS];

  if (n < 0) return;

  curr_max_dist = sqrt(ENV_OFFSET_X * ENV_OFFSET_X + ENV_OFFSET_Y * ENV_OFFSET_Y) + 10;
  clicked_col = getpixel(bmp, (int)point.x, (int)point.y);
  memcpy(tmp_new_array, array, sizeof(WPoint) * MAX_WPOINTS);

  if (clicked_col == makecol(255, 0, 0) ||
      clicked_col == makecol(0, 255, 255)) {
    for (int i = 0; i < MAX_WPOINTS; i++) {
      xy_dist[0] = point.x - tmp_new_array[i].x;
      xy_dist[1] = point.y - tmp_new_array[i].y;

      dist = sqrt(xy_dist[0] * xy_dist[0] + xy_dist[1] * xy_dist[1]);

      if (dist < curr_max_dist) {
        curr_max_dist = dist;
        min_dist_idx = i;
      }
    }

    tmp_new_array[min_dist_idx].x = -9999;
    tmp_new_array[min_dist_idx].y = -9999;

    for (int i = 0; i < MAX_WPOINTS; i++) {
      if (tmp_new_array[i].x > -9999) {
        array[j].x = tmp_new_array[i].x;
        array[j].y = tmp_new_array[i].y;
        j++;
      }
    }

    for (int i = j; i < MAX_WPOINTS; i++) {
      array[i].x = -9999;
      array[i].y = -9999;
    }

    printf("Point %d deleted\n", min_dist_idx);

    *num = n - 1;
  }
}

/*
 * Function: Draw laser beams
 * ---------------------------
 */
void draw_laser_traces(BITMAP* bmp, Trace* old, Trace* new, double* old_pose,
                       double* pose) {
  int red = makecol(255, 0, 0);
  int blk = makecol(0, 0, 0);
  Trace temp;
  double X[3] = {0.0};
  double old_X[3] = {0.0};

  X[0] = (ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3]));
  X[1] = (ENV_OFFSET_Y - ENV_SCALE * (pose[4]));

  old_X[0] = (ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (old_pose[3]));
  old_X[1] = (ENV_OFFSET_Y - ENV_SCALE * (old_pose[4]));

  for (int i = 0; i < N_BEAMS; i++) {
    temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (old_pose[3] + old[i].x);
    temp.y = ENV_OFFSET_Y - ENV_SCALE * (old_pose[4] + old[i].y);
    // temp.z = old_pose[5] + old[i].z;
    if (getpixel(bmp, temp.x, temp.y) < 0) continue;
    fastline(bmp, old_X[0], old_X[1], temp.x, temp.y, blk);

    temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3] + new[i].x);
    temp.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + new[i].y);
    // temp.z = pose[5] + new[i].z;
    if (getpixel(bmp, temp.x, temp.y) < 0) continue;
    fastline(bmp, X[0], X[1], temp.x, temp.y, red);
  }
  // printf("---\n");
}

/*
 * Function: Draw laser beams as points
 * ---------------------------
 */
void draw_laser_points(BITMAP* bmp, Trace* old, Trace* new, double* old_pose,
                       double* pose) {
  int red = makecol(255, 0, 0);
  int blk = makecol(0, 0, 0);
  Trace temp = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};
  Trace temp_old = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};

  for (int i = 0; i < N_BEAMS; i++) {
    temp_old.x =
        ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (old_pose[3] + old[i].x);
    temp_old.y = ENV_OFFSET_Y - ENV_SCALE * (old_pose[4] + old[i].y);
    // temp_old.z = old_pose[5] + old[i].z;
    if (getpixel(bmp, (int)temp_old.x, (int)temp_old.y) < 0) continue;
    putpixel(bmp, (int)temp_old.x, (int)temp_old.y, blk);

    temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3] + new[i].x);
    temp.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + new[i].y);
    // temp.z = pose[5] + new[i].z;
    if (getpixel(bmp, (int)temp.x, (int)temp.y) < 0) continue;
    putpixel(bmp, (int)temp.x, (int)temp.y, red);
  }
  // printf("---\n");
}

/*
 * Function: Draw quadcopter actor
 * ---------------------------
 * Draws and rotates the quadcopter sprite
 */
void draw_quad(BITMAP* bmp, BITMAP* quad, BITMAP* bg, double* old,
               double* new) {
  int old_x = ENV_OFFSET_X + (int)(ENV_SCALE * old[3]);
  int old_y = ENV_OFFSET_Y - (int)(ENV_SCALE * old[4]);
  fixed old_yaw = ftofix(-old[2] / (2 * M_PI) * 256);

  int x = ENV_OFFSET_X + (int)(ENV_SCALE * new[3]);
  int y = ENV_OFFSET_Y - (int)(ENV_SCALE * new[4]);
  fixed yaw = ftofix(-new[2] / (2 * M_PI) * 256);

  rotate_sprite(bmp, bg, old_x - bg->w / 2, old_y - bg->h / 2, old_yaw);
  rotate_sprite(bmp, quad, x - bg->w / 2, y - bg->h / 2, yaw);
}

/*
 * Function: Draws square
 * ---------------------------
 * Draws a rotating square as two triangles
 * if the quadcopter bitmap is not available
 */
void draw_pose(BITMAP* bmp, double* old, double* new) {
  int old_x = ENV_OFFSET_X + (int)(ENV_SCALE * old[3]);
  int old_y = ENV_OFFSET_Y - (int)(ENV_SCALE * old[4]);

  int x = ENV_OFFSET_X + (int)(ENV_SCALE * new[3]);
  int y = ENV_OFFSET_Y - (int)(ENV_SCALE * new[4]);

  int tr1_1_x = old_x - 12;
  int tr1_1_y = old_y - 12;

  int tr1_2_x = old_x - 12;
  int tr1_2_y = old_y + 12;

  int tr1_3_x = old_x + 12;
  int tr1_3_y = old_y - 12;

  int tr2_1_x = old_x + 12;
  int tr2_1_y = old_y + 12;

  triangle(bmp, tr1_1_x, tr1_1_y, tr1_2_x, tr1_2_y, tr1_3_x, tr1_3_y,
           makecol(0, 0, 0));
  triangle(bmp, tr2_1_x, tr2_1_y, tr1_2_x, tr1_2_y, tr1_3_x, tr1_3_y,
           makecol(0, 0, 0));

  tr1_1_x = x - 12;
  tr1_1_y = y - 12;

  tr1_2_x = x - 12;
  tr1_2_y = y + 12;

  tr1_3_x = x + 12;
  tr1_3_y = y - 12;

  tr2_1_x = x + 12;
  tr2_1_y = y + 12;

  triangle(bmp, tr1_1_x, tr1_1_y, tr1_2_x, tr1_2_y, tr1_3_x, tr1_3_y,
           COL_GREEN);
  triangle(bmp, tr2_1_x, tr2_1_y, tr1_2_x, tr1_2_y, tr1_3_x, tr1_3_y,
           COL_GREEN);
}

/*
 * Function: Print tasks' periods
 * ---------------------------
 * Draws to screen the periods of all the tasks
 * Highlights the selected one
 */
void draw_periods(BITMAP* bmp, int* tp, int size, int sel) {
  char text[4];

  rectfill(bmp, 1328, 40, 1361, 115, makecol(0, 0, 0));

  for (int i = 0; i < size; i++) {
    sprintf(text, "%d", tp[i]);

    if (i == sel)
      textout_ex(bmp, font, text, 1329, 45 + 10 * i, makecol(255, 165, 0), -1);
    else
      textout_ex(bmp, font, text, 1329, 45 + 10 * i, COL_GREEN, -1);
  }
}

/*
 * Function: Draws selected waypoints
 * ---------------------------
 */
void draw_waypoints(BITMAP* bmp, WPoint* old_wpoints, WPoint* wpoints,
                    int size) {
  int blk = makecol(0, 0, 0);
  int red = makecol(255, 0, 0);

  // Draw black on old waypoints
  for (int i = 0; i < MAX_WPOINTS; i++) {
    if (old_wpoints[i].x <= -9999) continue;
    circlefill(bmp, old_wpoints[i].x, old_wpoints[i].y, 4, blk);
  }

  if (size < 0) return;
  if (size >= MAX_WPOINTS) return;

  for (int i = 0; i < MAX_WPOINTS; i++) {
    if (i == 0 && wpoints[i].x > -9999)
      circlefill(bmp, wpoints[i].x, wpoints[i].y, 4, makecol(0, 255, 255));
    else if (i > 0 && wpoints[i].x > -9999)
      circlefill(bmp, wpoints[i].x, wpoints[i].y, 4, red);
  }
}

/*
 * Function: Plots update
 * ---------------------------
 * Updates the plot of the variable of interest
 */
void update_plot(BITMAP* bmp, double* data, int coord_x, int coord_y,
                 double scale) {
  int x = coord_x - (PLT_STEP * PLT_DATA_SIZE);
  int y = coord_y - (data[0] * scale) - PLT_FRAME_SIZE / 2;

  int x_prev = x;
  int y_prev = y;

  rectfill(bmp, coord_x - 100, coord_y - 100, coord_x - 1, coord_y - 1,
           makecol(0, 0, 0));

  for (int i = 0; i < PLT_DATA_SIZE; i++) {
    x = x + PLT_STEP;

    y = coord_y - (data[i] * scale) - PLT_FRAME_SIZE / 2;

    if (y <= (coord_y - PLT_FRAME_SIZE)) y = coord_y - PLT_FRAME_SIZE + 1;

    if (y >= coord_y) y = coord_y - 1;

    if (y_prev <= (coord_y - PLT_FRAME_SIZE))
      y_prev = coord_y - PLT_FRAME_SIZE + 1;

    if (y_prev >= coord_y) y_prev = coord_y - 1;

    if (getpixel(bmp, x_prev, y_prev) < 0) continue;
    if (getpixel(bmp, x, y) < 0) continue;

    fastline(bmp, x_prev, y_prev, x, y, COL_GREEN);

    x_prev = x;
    y_prev = y;
  }
}

/*
 * Function: Print controller gains
 * ---------------------------
 * Draws to screen the gains of the controllers
 * Highlights the selected one
 */
void draw_gains(BITMAP* bmp, double* p, double* d, int sel) {
  char text[20];

  rectfill(bmp, 1238, 164, 1361, 216, makecol(0, 0, 0));

  for (int i = 0; i < SIZE_PID; i++) {
    sprintf(text, "%.1e %.1e", p[i], d[i]);

    if (i == sel - 2)
      textout_ex(bmp, font, text, 1239, 165 + 10 * i, makecol(255, 165, 0), -1);
    else
      textout_ex(bmp, font, text, 1239, 165 + 10 * i, COL_GREEN, -1);
  }
}

void draw_altitude(BITMAP* bmp, double alt, double sp) {
  char text[30];

  sprintf(text, "[L] ALTITUDE %.2f - %.2f", sp, alt);

  rectfill(bmp, 1135, 130, 1361, 145, makecol(0, 0, 0));

  textout_ex(bmp, font, text, 1141, 135, COL_GREEN, -1);
}

/*
 * Function: Prints message
 * ---------------------------
 * Draws a text on a screen location according to a selected
 * mode
 */
void draw_msg(BITMAP* bmp, int mode, int x, int y) {
  char text[20];

  if (mode == 0) return;

  if (mode == 1) {
    textout_centre_ex(bmp, font, "!!! Collision detected !!!", x, y,
                      makecol(0, 0, 0), makecol(255, 0, 0));
    textout_centre_ex(bmp, font, "Press [ESC] to exit simulation", x + 8, y + 8,
                      makecol(0, 0, 0), makecol(255, 0, 0));
  } else if (mode == 2) {
    rectfill(bmp, WIDTH_SCREEN - 260, y - 4, WIDTH_SCREEN - 5, y + 10,
             makecol(0, 0, 0));
    textout_centre_ex(bmp, font, "Status: Reaching goal", x, y, COL_GREEN, -1);
  } else if (mode == 4) {
    rectfill(bmp, WIDTH_SCREEN - 260, y - 4, WIDTH_SCREEN - 5, y + 10,
             makecol(0, 0, 0));
    textout_centre_ex(bmp, font, "Click on Starting point", x, y, COL_GREEN,
                      -1);
  } else if (mode > 4 && mode < 9) {
    sprintf(text, "Click on GOAL n.%d", mode - 4);

    rectfill(bmp, WIDTH_SCREEN - 260, y - 4, WIDTH_SCREEN - 5, y + 10,
             makecol(0, 0, 0));
    textout_centre_ex(bmp, font, text, x, y, COL_GREEN, -1);
  } else if (mode == 9) {
    rectfill(bmp, WIDTH_SCREEN - 260, y - 4, WIDTH_SCREEN - 5, y + 10,
             makecol(0, 0, 0));
    textout_centre_ex(bmp, font, "Press [SPACE] to start", x, y, COL_GREEN, -1);
  } else if (mode > 9) {
    sprintf(text, "Avoiding: MODE %d", mode - 9);

    rectfill(bmp, WIDTH_SCREEN - 260, y - 4, WIDTH_SCREEN - 5, y + 10,
             makecol(0, 0, 0));
    textout_centre_ex(bmp, font, text, x, y, makecol(255, 165, 0), -1);
  }
}
