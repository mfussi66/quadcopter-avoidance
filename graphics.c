/* Function file for RTS Project */

#include "graphics.h"

/* Graphics */ 

void start_allegro (void)
{

	allegro_init();
    
	set_gfx_mode (GFX_AUTODETECT_WINDOWED, 800, 600, 0, 0);
	clear_to_color (screen, 0);
    
    install_keyboard();
    
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
	int x = coord_x - (GRAPH_STEP * GRAPH_DATA_SIZE);
	int y = coord_y - (int)(data[0] * GRAPH_SCALE);
	int x_prev = x;
	int y_prev = y;
	
	for(int i = 0; i < GRAPH_DATA_SIZE; i++)
	{
		x = x + GRAPH_STEP;
		y = coord_y - (int)(data[i] * GRAPH_SCALE);
		
		if(data[i] >  0.0)
			fastline(bmp, x_prev, y_prev, x, y, makecol(0, 255, 0));

		x_prev = x;
		y_prev = y;
	}
}
