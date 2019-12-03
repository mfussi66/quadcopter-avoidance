/* Function file for RTS Project */

#include "graphics.h"

/* Graphics */ 

void start_allegro (void)
{

	allegro_init();
    
    set_color_depth(8);
    
	set_gfx_mode (GFX_AUTODETECT_WINDOWED, 800, 600, 0, 0);
    
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

void update_graph (BITMAP* bmp, double* data, int coord_x, int coord_y)
{
	int x = coord_x - (PLT_STEP * PLT_DATA_SIZE);
	int y = coord_y - (int)(data[0] * PLT_SCALE);
	int x_prev = x;
	int y_prev = y;
	
	for(int i = 0; i < PLT_DATA_SIZE; i++)
	{
		x = x + PLT_STEP;
		y = coord_y - (int)(data[i] * PLT_SCALE);
		
		if(data[i] >  0.0)
			fastline(bmp, x_prev, y_prev, x, y, makecol(0, 255, 0));

		x_prev = x;
		y_prev = y;
	}
}

void update_pose (BITMAP* bmp, double* old, double* new)
{
    
    int old_x = ENV_OFFSET_X + (int)(ENV_SCALE * old[3]);
    int old_y = ENV_OFFSET_Y + (int)(ENV_SCALE * old[4]);

    int x = ENV_OFFSET_X + (int)(ENV_SCALE * new[3]);
    int y = ENV_OFFSET_Y + (int)(ENV_SCALE * new[4]);
    
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
    
    triangle(bmp, tr1_1_x,tr1_1_y,tr1_2_x,tr1_2_y,tr1_3_x,tr1_3_y,makecol(0,255,0));
    triangle(bmp, tr2_1_x,tr2_1_y,tr1_2_x,tr1_2_y,tr1_3_x,tr1_3_y,makecol(0,255,0));
    
}
