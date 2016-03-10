#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Install.h"
#include "Output_Interface.h"

#if defined(_WIN32)
# if ! defined F_CALLCONV
#  define F_CALLCONV __stdcall
# endif
#else
# if ! defined(F_CALLCONV)
#  define F_CALLCONV
# endif
#endif

#if   defined(FNAME_LCASE_DECOR) /* fortran names: lower case, trailing _ */
# define FOUT    fout_
#elif defined(FNAME_LCASE_NODECOR) /* fortran names: lower case, no _ */
# define FOUT    fout
#elif defined(FNAME_UCASE_DECOR) /* fortran names: upper case, trailing _ */
# define FOUT    FOUT_
#elif defined(FNAME_UCASE_NODECOR) /* fortran names: upper case, no _ */
# define FOUT    FOUT
#else
#error "No compile define for fortran naming convention"
No_compile_define_for_fortran_naming_convention;
#endif

#define BUFFER_SIZE 1024

void F_CALLCONV FOUT(int *unitnum, char *message, int len);

static char buffer[BUFFER_SIZE];
static int log_file = 6;
static int status_file = -1;
static int listing_file = -1;

void for_output_set_log(int l)
{
  log_file = l;
  return;
}

void for_output_set_status(int s)
{
  status_file = s;
  return;
}

void for_output_set_listing(int l)
{
  listing_file = l;
  return;
}

static CB_FUNC(void) for_print(void *data, int mode, char *buf)
{
  if (mode & Output_Log) {
    if (log_file >= 0) {
      strcpy(buffer, buf);
      if (buffer[strlen(buffer) - 2] == '\n') {
        buffer[strlen(buffer) - 2] = '\0';
      }
      FOUT(&log_file, buffer, (signed) strlen(buffer) - 1);
    }
  }

  if (mode & Output_Status) {
    if (status_file >= 0) {
      strcpy(buffer, buf);
      if (buffer[strlen(buffer) - 2] == '\n') {
        buffer[strlen(buffer) - 2] = '\0';
      }
      FOUT(&status_file, buffer, (signed) strlen(buffer) - 1);
    }
  }

  if (mode & Output_Listing) {
    if (listing_file >= 0) {
      strcpy(buffer, buf);
      if (buffer[strlen(buffer) - 2] == '\n') {
        buffer[strlen(buffer) - 2] = '\0';
      }
      FOUT(&listing_file, buffer, (signed) strlen(buffer) - 1);
    }
  }

  return;
}

static Output_Interface for_output_interface =
{
  NULL,
  for_print,
  NULL
};

void for_install_output(void)
{
  Output_SetInterface(&for_output_interface);
  return;
}
