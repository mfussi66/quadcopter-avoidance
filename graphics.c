/* Function file for RTS Project */

#include "graphics.h"

/* Graphics */ 

void start_allegro (void)
{

	allegro_init();
    
    set_color_depth(8);
    
	set_gfx_mode (GFX_XWINDOWS, 800, 600, 0, 0);
    install_keyboard();
    
    clear_to_color (screen, 0);
    
	printf ("Allegro correctly initialized\n");

	return;

}

void close_allegro(void)
{
	
	allegro_exit();
    
    printf("Graphics: Allegro is closed\n");
}

void update_plot(BITMAP* bmp, double* data, int coord_x, int coord_y)
{
	int x = coord_x - (PLT_STEP * PLT_DATA_SIZE);
	int y = coord_y - (int)(data[0] * PLT_SCALE / 2) - 50;
	int x_prev = x;
	int y_prev = y;
	
	for(int i = 0; i < PLT_DATA_SIZE; i++)
	{
		x = x + PLT_STEP;
		y = coord_y - (int)(data[i] * PLT_SCALE / 2) - 50;
		
		if(data[i] >  0.0)
			fastline(bmp, x_prev, y_prev, x, y, COL_GREEN);

		x_prev = x;
		y_prev = y;
	}
}

void build_gui(BITMAP* bmp, FONT* font, int col)
{
	rect(bmp, 5, 5, 560, 595, col);
	
	rect(bmp, 695, 285, PLT_XPOS_XCOORD, PLT_XPOS_YCOORD, col);
	rect(bmp, 695, 390, PLT_YPOS_XCOORD, PLT_YPOS_YCOORD, col);
	rect(bmp, 695, 495, PLT_ZPOS_XCOORD, PLT_ZPOS_YCOORD, col);
	
	rect(bmp, 580, 285, 680, 385, col);
	rect(bmp, 580, 390, 680, 490, col);
	rect(bmp, 580, 495, 680, 595, col);
	
	textout_centre_ex(bmp, font, "R", 575, 335, col, -1);
	textout_centre_ex(bmp, font, "P", 575, 440, col, -1);
	textout_centre_ex(bmp, font, "Y", 575, 545, col, -1);
	
	textout_centre_ex(bmp, font, "X", 690, 335, col, -1);
	textout_centre_ex(bmp, font, "Y", 690, 440, col, -1);
	textout_centre_ex(bmp, font, "Z", 690, 545, col, -1);

}

void draw_exit_screen(BITMAP* bmp, int col)
{
	textout_centre_ex(bmp, font, "_________________________", 400, 287, col, -1);
	textout_centre_ex(bmp, font, "   Simulation stopped    ", 400, 300, col, -1);
	textout_centre_ex(bmp, font, "  Thank you for playing  ", 400, 310, col, -1);
	textout_centre_ex(bmp, font, "_________________________", 400, 315, col, -1);	
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
	
	for(int i = 0; i < 5; i++)
	{
		temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (old_pose[3] + old[i].x);
		temp.y = ENV_OFFSET_Y - ENV_SCALE * (old_pose[4] + old[i].y);
		temp.z = old_pose[5] + old[i].z;
		
		fastline(bmp, old_X[0], old_X[1], (int)temp.x, (int)temp.y, blk);

		temp.x = ENV_OFFSET_X + OFFSET_LASER + ENV_SCALE * (pose[3] + new[i].x);
		temp.y = ENV_OFFSET_Y - ENV_SCALE * (pose[4] + new[i].y);
		temp.z = pose[5] + new[i].z;
		
		//printf("print %d, x: %d, y: %d xd: %f yd: %f \n", i, (int)temp.x,(int)temp.y, new[i].x,new[i].y);
		fastline(bmp, X[0], X[1], (int)temp.x, (int)temp.y, red);
	}
	//printf("---\n");

}

void draw_quad(BITMAP* bmp, BITMAP* quad, BITMAP* bg, double* old, double* new)
{

int old_x = ENV_OFFSET_X + (int) (ENV_SCALE * old[3]);
int old_y = ENV_OFFSET_Y - (int) (ENV_SCALE * old[4]);
fixed old_yaw = itofix(-old[2] * 180 / M_PI);

int x = ENV_OFFSET_X + (int) (ENV_SCALE * new[3]);
int y = ENV_OFFSET_Y - (int) (ENV_SCALE * new[4]);
fixed yaw = itofix(-new[2] * 180 / M_PI);
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
