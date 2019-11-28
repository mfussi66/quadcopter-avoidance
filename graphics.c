/* Function file for RTS Project */

#include "graphics.h"

/* Graphics */ 

void start_allegro (void)
{

	allegro_init ();
	set_gfx_mode (GFX_AUTODETECT_WINDOWED, 800, 600, 0, 0);
	clear_to_color (screen, 0);
	install_keyboard ();

	printf ("Allegro correctly initialized\n");

	return;

}

void close_allegro(void)
{
	printf("graphics: Closing Allegro...\n");
	allegro_exit();
}
