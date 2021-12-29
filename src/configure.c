/* --------------------------
         REAL TIME SYSTEMS
         OBSTACLE AVOIDANCE
           Mattia Fussi

 COMMON CONSTANTS AND STRUCTS
 -------------------------- */

#include "configure.h"

static void error(const char* msg, const char* msg1) {
  fprintf(stderr, "ERROR: %s%s\n", msg, msg1 ? msg1 : "");
  exit(1);
}

int read_parameters(const char* config_file, DynParams* p) {
  FILE* fp;
  char errbuf[200];

  // 1. Read and parse toml file
  fp = fopen(config_file, "r");
  if (!fp) {
    error("cannot open config file - ", strerror(errno));
    return -1;
  }

  toml_table_t* conf = toml_parse_file(fp, errbuf, sizeof(errbuf));
  fclose(fp);

  if (!conf) {
    error("cannot parse - ", errbuf);
    return -1;
  }

  toml_table_t* dynamics = toml_table_in(conf, "dynamics");

  toml_datum_t mass = toml_double_in(dynamics, "m");
  if (mass.ok) p->mass = mass.u.d;

  toml_datum_t jxx = toml_double_in(dynamics, "jxx");
  if (jxx.ok) p->J_xx = jxx.u.d;

  toml_datum_t jyy = toml_double_in(dynamics, "jyy");
  if (jyy.ok) p->J_yy = jxx.u.d;

  toml_datum_t jzz = toml_double_in(dynamics, "jzz");
  if (jzz.ok) p->J_zz = jxx.u.d;

  toml_free(conf);

  return 0;
}
