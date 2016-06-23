#pragma once

#include <stdio.h>

typedef struct mrdplot_data
{
  const char *filename;
  int total_n_numbers;
  int n_points;
  int n_channels;
  float frequency;
  float *data;
  char **names;
  char **units;
} MRDPLOT_DATA;

/*************************************************************************/

MRDPLOT_DATA *malloc_mrdplot_data( int n_channels, int n_points );
MRDPLOT_DATA *read_mrdplot(const char *filename );
void write_mrdplot_file( MRDPLOT_DATA *d );
int find_channel(const char *name, MRDPLOT_DATA *d );
//char *generate_file_name();
char *generate_file_name(const char *prefix);
char *last_data();

void fwrite_reversed( char *p, int i1, int i2, FILE *stream );
/*************************************************************************/
