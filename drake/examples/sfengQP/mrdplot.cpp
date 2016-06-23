/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <termio.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "mrdplot.h"

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

	MRDPLOT_DATA *
malloc_mrdplot_data( int n_channels, int n_points )
{
	MRDPLOT_DATA *d;

	d = (MRDPLOT_DATA *) malloc( sizeof( MRDPLOT_DATA ) );
	if ( d == NULL )
	{
		fprintf( stderr, "Couldn't allocate MRDPLOT_DATA.\n" );
		exit( -1 );
	}
	d->filename = NULL;
	d->n_channels = n_channels;
	d->n_points = n_points;
	d->total_n_numbers = n_channels*n_points;
	d->frequency = 0;
	if ( n_channels > 0 )
	{
		d->names = (char **) malloc( d->n_channels*sizeof(char *) );
		d->units = (char **) malloc( d->n_channels*sizeof(char *) );
		if ( d->names == NULL || d->units == NULL )
		{
			fprintf( stderr,
					"malloc_mrdplot_data: Can't allocate memory for names or units.\n" );
			exit( -1 );
		}
	}
	else
	{
		d->names = NULL;
		d->units = NULL;
	}
	if ( d->total_n_numbers > 0 )
	{
		d->data = (float *) malloc( d->total_n_numbers*sizeof( float ) );
		if ( d->data == NULL )
		{
			fprintf( stderr,
					"malloc_mrdplot_data: Couldn't allocate memory of size %d\n", 
					d->total_n_numbers );
			exit( -1 );
		}
	}
	else
		d->data = NULL;
	return d;
}

/*****************************************************************************/

	MRDPLOT_DATA *
read_mrdplot(const char *filename )
{
	FILE *stream;
	int total_n_numbers, n_channels, n_points;
	float frequency;
	MRDPLOT_DATA *d;
	int i;
	char buffer1[1000];
	char buffer2[1000];
	char *p;
	int n_bytes;

	/* Windows needs binary flag. No effect on Unix */
	stream = fopen( filename, "rb" );
	if ( stream == NULL )
	{
		fprintf( stderr, "Couldn't open %s file for read.\n", 
				filename );
		exit( -1 );
	}

	if ( fscanf( stream, "%d%d%d%f",
				&total_n_numbers, 
				&n_channels, 
				&n_points, 
				&frequency ) != 4 )
	{
		fprintf( stderr, "Header error reading %s\n", filename );
		exit( -1 );
	}

	d = malloc_mrdplot_data( n_channels, n_points );
	d->filename = filename;
	d->frequency = frequency;

	printf(
			"%d points, %d channels in sample, %d numbers total, %g samples/second.\n",
			d->n_points, d->n_channels, 
			d->total_n_numbers, d->frequency );


	for( i = 0; i < d->n_channels; i++ )
	{
		if (fscanf( stream, "%s%s", buffer1, buffer2 ) != 2) {
			fprintf( stderr, "Name parsing error reading %s at channel %d\n", filename, i );
			exit( -1 );
		}
		d->names[i] = strdup( buffer1 );
		d->units[i] = strdup( buffer2 );
		//printf( "%d: %s %s\n", i, d->names[i], d->units[i] );
	}
	int ret = fscanf( stream, "%c%c%c", buffer1, buffer1, buffer1 );
	assert(ret);

	/* SGI version */
	/*
		 fread( d, n_channels*sizeof( float ), n_points, stream );
		 */
	/* Linux version */
	p = (char *) (d->data);
	n_bytes = d->total_n_numbers*4;
	for( i = 0; i < n_bytes; i += 4 )
	{
		ret = fread( &(p[i+3]), 1, 1, stream );
		assert( ret );
		ret = fread( &(p[i+2]), 1, 1, stream ); 
		assert( ret );
		ret = fread( &(p[i+1]), 1, 1, stream );
		assert( ret );
		ret = fread( &(p[i+0]), 1, 1, stream );
		assert( ret );
	}

	fclose( stream );
	return d;
}

/*****************************************************************************/

	int
find_channel( const char *name, MRDPLOT_DATA *d )
{
	int i;

	for ( i = 0; i < d->n_channels; i++ )
	{
		if ( strcmp( name, d->names[i] ) == 0 )
		{
			//printf( "Found %s at %d\n", name, i );
			return i;
		}
	}
	printf( "Didn't find %s\n", name );
	return -1;
}

/*****************************************************************************/
/*****************************************************************************/

void fwrite_reversed( char *p, int i1, int i2, FILE *stream )
{
	int total, i;

	total = i1*i2;

	for( i = 0; i < total; i += 4 )
	{
		fwrite( &(p[i+3]), 1, 1, stream );
		fwrite( &(p[i+2]), 1, 1, stream );
		fwrite( &(p[i+1]), 1, 1, stream );
		fwrite( &(p[i+0]), 1, 1, stream );
	}
}

/*****************************************************************************/

	void
write_mrdplot_file( MRDPLOT_DATA *d )
{
	FILE *stream;
	int i;

	// printf( "Writing %s\n", d->filename );

	/* Windows needs binary flag. No effect on Unix */
	stream = fopen( d->filename, "wb" );
	if ( stream == NULL )
	{
		fprintf( stderr, "Couldn't open %s file for write.\n", d->filename );
		exit( -1 );
	}

	/*
		 printf( "%d %d %d %f\n",
		 d->total_n_numbers, d->n_channels, d->n_points, d->frequency );
		 */

	fprintf( stream, "%d %d %d %f\n",
			d->total_n_numbers, d->n_channels, d->n_points, d->frequency );

	for( i = 0; i < d->n_channels; i++ )
	{
		fprintf( stream, "%s %s\n", d->names[i], d->units[i] );
	}
	// Linux version
	fprintf( stream, "\n\n" );
	// Windows version needs this?
	// fprintf( stream, "\r\r" );


	for( i = 0; i < d->n_points; i++ )
	{
		/* SGI version
			 fwrite( &(data[i*N_CHANNELS]),
			 N_CHANNELS*sizeof( float ), 1, stream );
			 */
		/* Linux version */
		fwrite_reversed( (char *) (&(d->data[i*d->n_channels])),
				d->n_channels*sizeof( float ), 1, stream );
		/*
			 fwrite( (char *) (&(d->data[i*d->n_channels])),
			 d->n_channels*sizeof( float ), 1, stream );
			 */
	}

	fclose( stream );
}

/*****************************************************************************/

char generated_file_name[10000];
	
  char *
generate_file_name(const char *prefix)
{
	FILE *stream;
	char tmp_string_time[1000];
	char tmp_string_user[1000];
	int file_number = 0;
	time_t now = time(0);
	struct tm tstruct;

	stream = fopen( "/logs/last_data", "r" );
	if ( stream != NULL )
	{
		fscanf( stream, "%d", &file_number );
		fclose( stream );
	}

	tstruct = *localtime(&now);
	strftime(tmp_string_time, sizeof(tmp_string_time), "%m_%d_%H_%M_%S", &tstruct);

	int i = getlogin_r(tmp_string_user, sizeof(tmp_string_user));
	// getlogin_r seems to be failing in some cases
	if (i != 0)
	{
		tmp_string_user[0] = 0;
		tmp_string_user[1] = 0;
	}


	sprintf(generated_file_name, "%s_%s.mrd", prefix, tmp_string_time);

	stream = fopen( "/home/siyuanfeng/logs/last_data", "w" );
	// change the mode of the file so everyone can read/write to it
	// If not owner this will cause warning:
	// chmod: changing permissions of `/logs/last_data': Operation not permitted
	system("chmod 666 /home/siyuanfeng/logs/last_data");
	if ( stream != NULL )
	{
		file_number++;
		fprintf( stream, "%d\n", file_number );
		fclose( stream );
	}

	return strdup(generated_file_name);
}

/*****************************************************************************/

char *last_data()
{
	FILE *stream;
	int file_number;

	stream = fopen( "last_data", "r" );
	if ( stream == NULL )
		return strdup( "d00000" );

	if(fscanf( stream, "%d", &file_number ) == 0)
		return nullptr;

	sprintf( generated_file_name, "d%05d", file_number - 1 );
	fclose( stream );
	return strdup( generated_file_name );
}

/*****************************************************************************/
/*****************************************************************************/
