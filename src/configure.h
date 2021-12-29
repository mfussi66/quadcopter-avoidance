/* -------------------------- 
	 REAL TIME SYSTEMS
	 OBSTACLE AVOIDANCE
	   Mattia Fussi
	
 COMMON CONSTANTS AND STRUCTS
 -------------------------- */

//#ifndef CONFIGURE_H_

//#define CONFIGURE_H_

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include "toml.h"
#include "customdata.h"

static void error(const char* msg, const char* msg1);

int read_parameters(const char* config_file, DynParams* p);

// #endif  // CONFIGURE_H_
