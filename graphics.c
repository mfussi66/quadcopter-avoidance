/* Function file for RTS Project */

#include "graphics.h"

/* Graphics */ 

void start_allegro (void)
{

	allegro_init();
    
    set_color_depth(8);
    
	set_gfx_mode (GFX_AUTODETECT_FULLSCREEN, 1366, 768, 0, 0);
    install_keyboard();
	install_mouse();
	
	//enable_hardware_cursor();
    
    clear_to_color(screen, 0);
    
	printf ("Allegro correctly initialized\n");

	return;

}

void close_allegro(void)
{
	
	allegro_exit();
    
    printf("Graphics: Allegro is closed\n");
}

void build_gui(BITMAP* bmp, FONT* font, int col)
{
	// Environment window
	rect(bmp, 5, 5, 560, 595, col);

	// Plots coordinates
	rect(bmp, 695, 285, PLT_XPOS_XCOORD, PLT_XPOS_YCOORD, col);
	rect(bmp, 695, 390, PLT_YPOS_XCOORD, PLT_YPOS_YCOORD, col);
	rect(bmp, 695, 495, PLT_ZPOS_XCOORD, PLT_ZPOS_YCOORD, col);
	rect(bmp, 580, 285, 680, 385, col);
	rect(bmp, 580, 390, 680, 490, col);
	rect(bmp, 580, 495, 680, 595, col);
	
	// Plots labels
	textout_centre_ex(bmp, font, "R", 575, 335, col, -1);
	textout_centre_ex(bmp, font, "P", 575, 440, col, -1);
	textout_centre_ex(bmp, font, "Y", 575, 545, col, -1);
	textout_centre_ex(bmp, font, "X", 690, 335, col, -1);
	textout_centre_ex(bmp, font, "Y", 690, 440, col, -1);
	textout_centre_ex(bmp, font, "Z", 690, 545, col, -1);
	
	textout_ex(bmp,font, "Select with number/letter", 575, 10, col, -1);
	textout_ex(bmp,font, "Change with Up/Down arrows", 575, 20, col, -1);
	textout_ex(bmp,font, "PERIODS", 575, 35, col, -1);
	textout_ex(bmp,font, "[0] Graphics", 575, 45, col, -1);
	textout_ex(bmp,font, "[1] KeyBoard", 575, 55, col, -1);
	textout_ex(bmp,font, "[2] Waypoints", 575, 65, col, -1);
	textout_ex(bmp,font, "[3] Model", 575, 75, col, -1);
	textout_ex(bmp,font, "[4] Control", 575, 85, col, -1);
	textout_ex(bmp,font, "[5] Laser Scan", 575, 95, col, -1);
	textout_ex(bmp,font, "[6] Plots", 575, 105, col, -1);
	textout_ex(bmp,font, "[7] LQR iter", 575, 115, col, -1);
	textout_ex(bmp,font, "[ENTER] confirm period", 575, 125, col, -1);
	textout_ex(bmp,font, "[BACKSPACE] cancel period", 575, 135, col, -1);
	
	textout_ex(bmp,font, "GAINS", 575, 153, col, -1);
	textout_ex(bmp,font, "Kp      Kd", 680, 153, col, -1);
	textout_ex(bmp,font, "[X] Xpos", 575, 165, col, -1);
	textout_ex(bmp,font, "[Y] Ypos", 575, 175, col, -1);
	textout_ex(bmp,font, "[Z] Altitude", 575, 185, col, -1);
	textout_ex(bmp,font, "[R] Roll", 575, 195, col, -1);
	textout_ex(bmp,font, "[P] Pitch", 575, 205, col, -1);
	textout_ex(bmp,font, "[W] Yaw", 575, 215, col, -1);
	textout_ex(bmp,font, "[C] Reset gains to default", 575, 225, col, -1);	
}

void draw_exit_screen(BITMAP* bmp, int col)
{
	textout_centre_ex(bmp, font, "_________________________", 400, 287, col, -1);
	textout_centre_ex(bmp, font, "   Simulation stopped    ", 400, 300, col, -1);
	textout_centre_ex(bmp, font, "  Thank you for playing  ", 400, 310, col, -1);
	textout_centre_ex(bmp, font, "_________________________", 400, 315, col, -1);	
}

int gen_obstacles(Obstacle* arr_obstacles, int n_obs)
{
	size_t n = (uint)n_obs;

	if (n == 0)
		return -1;

	arr_obstacles[0].x1 = (5 - ENV_OFFSET_X) / ENV_SCALE;
	arr_obstacles[0].y1 = (-250 + ENV_OFFSET_Y) / ENV_SCALE;
	arr_obstacles[0].x2 = (250 - ENV_OFFSET_X) / ENV_SCALE;
	arr_obstacles[0].y2 = (-400 + ENV_OFFSET_Y) / ENV_SCALE;
	
	arr_obstacles[1].x1 = (360 - ENV_OFFSET_X) / ENV_SCALE;
	arr_obstacles[1].y1 = (-100 + ENV_OFFSET_Y) / ENV_SCALE;
	arr_obstacles[1].x2 = (560 - ENV_OFFSET_X) / ENV_SCALE;
	arr_obstacles[1].y2 = (-200 + ENV_OFFSET_Y) / ENV_SCALE;
	
	return 0;
	
}

void draw_obstacles(BITMAP* bmp, Obstacle* obs, int n_obs, int col)
{
	int X_env[4] = {0}; 
	
	for(int i = 0; i < n_obs; i++)
	{		
		X_env[0] = (int)(ENV_OFFSET_X + ENV_SCALE * (obs[i].x1));
		X_env[1] = (int)(ENV_OFFSET_Y - ENV_SCALE * (obs[i].y1));
		X_env[2] = (int)(ENV_OFFSET_X + ENV_SCALE * (obs[i].x2));
		X_env[3] = (int)(ENV_OFFSET_Y - ENV_SCALE * (obs[i].y2));
		
		rectfill(bmp, X_env[0], X_env[1], X_env[2], X_env[3], col);
	}
	
}

void add_waypoint(BITMAP *bmp, WPoint *array, int *num,  WPoint point)
{
	
	int n = *num + 1;
	
	if (n >= MAX_WPOINTS)
	{
		printf("Max n. of waypoints reached (5)\nPress SPACE to start\n");
		return;
	}
	
	if (getpixel(bmp, point.x, point.y) == makecol(0, 255, 0))
	{
		printf("Cannot place waypoint on obstacle\n");
		return;
	}
	
	if(n < MAX_WPOINTS - 1)
		printf("Click on waypoint %d\n", n + 1);
	
	array[n].x = point.x;
	array[n].y = point.y;
	
	*num = n;
}

void del_waypoint(BITMAP *bmp, WPoint *array, int *num,  WPoint point)
{
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
	
	if (clicked_col == makecol(255, 0, 0) || clicked_col == makecol(0, 255, 255)) 
	{
		for(int i = 0; i < MAX_WPOINTS; i++)
		{
			xy_dist[0] = point.x - tmp_new_array[i].x;
			xy_dist[1] = point.y - tmp_new_array[i].y;

			dist = sqrt(xy_dist[0] * xy_dist[0] + xy_dist[1] * xy_dist[1]);

			if (dist < curr_max_dist)
			{
				curr_max_dist = dist;
				min_dist_idx = i;
			}
		}
		
		tmp_new_array[min_dist_idx].x = -9999;
		tmp_new_array[min_dist_idx].y = -9999;

		for(int i = 0; i < MAX_WPOINTS; i++)
		{
			if(tmp_new_array[i].x > -9999)
			{
				array[j].x = tmp_new_array[i].x;
				array[j].y = tmp_new_array[i].y;
				j++;
			}
		}
	
		for(int i = j; i < MAX_WPOINTS; i++)
		{
			array[i].x = -9999;
			array[i].y = -9999;
		}
		
		printf("Point %d deleted\n", min_dist_idx);
		
		*num = n - 1;
	}
}

void draw_laser_traces(BITMAP *bmp, Trace* old, Trace* new, double* old_pose, double *pose)
{
	int red =  makecol(255, 0, 0);
	int blk = makecol(0,0,0);
	Trace temp;
	int X[3] = {0.0};
	int old_X[3] = {0.0};
	
	X[0] = (int)(ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3]));
	X[1] = (int)(ENV_OFFSET_Y - ENV_SCALE * (pose[4]));
	
	old_X[0] = (int)(ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (old_pose[3]));
	old_X[1] = (int)(ENV_OFFSET_Y - ENV_SCALE * (old_pose[4]));
	
	for(int i = 0; i < N_BEAMS; i++)
	{
		temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (old_pose[3] + old[i].x);
		temp.y = ENV_OFFSET_Y - ENV_SCALE * (old_pose[4] + old[i].y);
		//temp.z = old_pose[5] + old[i].z;
		if(getpixel(bmp, (int)temp.x, (int)temp.y) < 0) continue;
		fastline(bmp, old_X[0], old_X[1], (int)temp.x, (int)temp.y, blk);

		temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3] + new[i].x);
		temp.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + new[i].y);
		//temp.z = pose[5] + new[i].z;
		if(getpixel(bmp, (int)temp.x, (int)temp.y) < 0) continue;
		fastline(bmp, X[0], X[1], (int)temp.x, (int)temp.y, red);
	}
	//printf("---\n");

}

void draw_laser_points(BITMAP *bmp, Trace* old, Trace* new, double* old_pose, double *pose)
{
int red =  makecol(255, 0, 0);
int blk = makecol(0,0,0);
Trace temp = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};
Trace temp_old = {.x = BEAM_DMAX, .y = BEAM_DMAX, .z = BEAM_DMAX};

	for(int i = 0; i < N_BEAMS; i++)
	{
		temp_old.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (old_pose[3] + old[i].x);
		temp_old.y = ENV_OFFSET_Y - ENV_SCALE * (old_pose[4] + old[i].y);
		//temp_old.z = old_pose[5] + old[i].z;
		if(getpixel(bmp, (int)temp_old.x, (int)temp_old.y) < 0) continue;
		putpixel(bmp, (int)temp_old.x, (int)temp_old.y, blk);
		
		temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3] + new[i].x);
		temp.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + new[i].y);
		//temp.z = pose[5] + new[i].z;
		if(getpixel(bmp, (int)temp.x, (int)temp.y) < 0) continue;
		putpixel(bmp, (int)temp.x, (int)temp.y, red);
	}
	//printf("---\n");
	
}

void draw_quad(BITMAP* bmp, BITMAP* quad, BITMAP* bg, double* old, double* new)
{

int old_x = ENV_OFFSET_X + (int) (ENV_SCALE * old[3]);
int old_y = ENV_OFFSET_Y - (int) (ENV_SCALE * old[4]);
fixed old_yaw = ftofix(old[2] / (2*M_PI)*256);

int x = ENV_OFFSET_X + (int) (ENV_SCALE * new[3]);
int y = ENV_OFFSET_Y - (int) (ENV_SCALE * new[4]);
fixed yaw = ftofix(new[2] / (2*M_PI)*256);

	rotate_sprite(bmp, bg, old_x - bg->w / 2, old_y - bg->h / 2, old_yaw);
	rotate_sprite(bmp, quad, x - bg->w / 2, y - bg->h / 2, yaw);	
	
}

void draw_pose(BITMAP* bmp, double* old, double* new)
{
    
    int old_x = ENV_OFFSET_X + (int) (ENV_SCALE * old[3]);
    int old_y = ENV_OFFSET_Y - (int) (ENV_SCALE * old[4]);

    int x = ENV_OFFSET_X + (int) (ENV_SCALE * new[3]);
    int y = ENV_OFFSET_Y - (int) (ENV_SCALE * new[4]);
    
    int tr1_1_x = old_x - 12;
    int tr1_1_y = old_y - 12;

    int tr1_2_x = old_x - 12;
    int tr1_2_y = old_y + 12;
    
    int tr1_3_x = old_x + 12;
    int tr1_3_y = old_y - 12;
    
    int tr2_1_x = old_x + 12;
    int tr2_1_y = old_y + 12;
    
    triangle(bmp, tr1_1_x,tr1_1_y,tr1_2_x,tr1_2_y,tr1_3_x,tr1_3_y,makecol(0,0,0));
    triangle(bmp, tr2_1_x,tr2_1_y,tr1_2_x,tr1_2_y,tr1_3_x,tr1_3_y,makecol(0,0,0));
    
    tr1_1_x = x - 12;
    tr1_1_y = y - 12;

    tr1_2_x = x - 12;
    tr1_2_y = y + 12;
    
    tr1_3_x = x + 12;
    tr1_3_y = y - 12;
    
    tr2_1_x = x + 12;
    tr2_1_y = y + 12;
    
    triangle(bmp, tr1_1_x,tr1_1_y,tr1_2_x,tr1_2_y,tr1_3_x,tr1_3_y, COL_GREEN);
    triangle(bmp, tr2_1_x,tr2_1_y,tr1_2_x,tr1_2_y,tr1_3_x,tr1_3_y, COL_GREEN);
    
}

void draw_periods(BITMAP* bmp, int* tp, int size, int sel)
{
	char text[4];
	
	rectfill(bmp, 745, 40, 795, 125, makecol(0, 0, 0));
	
	for(int i = 0; i < size; i++)
	{
		sprintf(text, "%d", tp[i]);
		
		if (i == sel)
			textout_ex(bmp,font, text, 750, 45 + 10 * i, makecol(255, 165, 0), -1);
		else
			textout_ex(bmp,font, text, 750, 45 + 10 * i, COL_GREEN, -1);
	}
	
}

void draw_waypoints(BITMAP* bmp, WPoint* old_wpoints, WPoint* wpoints, int size)
{

int blk = makecol(0, 0, 0);
int red = makecol(255, 0, 0);

	// Draw black on old waypoints
	for(int i = 0; i < MAX_WPOINTS; i++)
	{
		if (old_wpoints[i].x <=-9999) continue;
		circlefill(bmp, old_wpoints[i].x, old_wpoints[i].y, 4, blk);
	}
	
	if (size < 0) return;
	if (size >= MAX_WPOINTS) return;
	
	for(int i = 0; i < MAX_WPOINTS; i++)
	{
		if (i==0 && wpoints[i].x > -9999)
			circlefill(bmp,wpoints[i].x, wpoints[i].y, 4, makecol(0, 255, 255));
		else if(i > 0 &&  wpoints[i].x > -9999)
			circlefill(bmp, wpoints[i].x, wpoints[i].y, 4, red);
		
	}
	
}

void update_plot(BITMAP* bmp, double* data, int coord_x, int coord_y, double scale)
{
	int x = coord_x - (PLT_STEP * PLT_DATA_SIZE);
	int y = coord_y - (data[0] * scale) - PLT_FRAME_SIZE / 2;
		
	int x_prev = x;
	int y_prev = y;
	
	for(int i = 0; i < PLT_DATA_SIZE; i++)
	{
		x = x + PLT_STEP;

		y = coord_y - (data[i] * scale) - PLT_FRAME_SIZE / 2;
		
		if (y <= (coord_y - PLT_FRAME_SIZE))
			y = (coord_y - PLT_FRAME_SIZE);
		
		if (y >= coord_y)
			y = coord_y;
		
		if(getpixel(bmp, x_prev, y_prev) < 0) continue;
		if(getpixel(bmp, x, y) < 0) continue;
		fastline(bmp, x_prev, y_prev, x, y, COL_GREEN);
		
		x_prev = x;
		y_prev = y;
	}
}

void draw_gains(BITMAP* bmp, double* p, double* d, int sel)
{
char text[20];
	
	rectfill(bmp, 672, 164, 795, 216, makecol(0, 0, 0));
	
	for(int i = 0; i < 6; i++)
	{
		sprintf(text, "%.1e %.1e", p[i], d[i]);
		
		if (i == sel - 2)
			textout_ex(bmp,font, text, 673, 165 + 10 * i, makecol(255, 165, 0), -1);
		else
			textout_ex(bmp,font, text, 673, 165 + 10 * i, COL_GREEN, -1);
	}
}

void draw_msg(BITMAP* bmp, int mode)
{
	if (mode == 0) return;
	
	switch(mode)
		case 1:
		{
			textout_centre_ex(bmp, font, "!!! Collision detected !!!", 400, 300,
							  makecol(0,0,0), makecol(255,0,0));
			textout_centre_ex(bmp, font, "Press [ESC] to exit simulation", 400, 309,
							  makecol(0,0,0), makecol(255,0,0));
		}
}
