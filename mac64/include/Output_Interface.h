/*****************************************************************************/
/* Output_Interface.h                                                        */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Interface used to change the behaviour of the output.                   */
/*                                                                           */
/*   User defined output functions are communicated through an               */
/*   Output_Interface, which should be set using the Output_SetInterface( )  */
/*   function.                                                               */
/*****************************************************************************/

#ifndef OUTPUT_INTERFACE_H
#define OUTPUT_INTERFACE_H

#include <stdio.h>
#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Output_Interface declaration.                                             */
/*****************************************************************************/
/*                                                                           */
/* - output_data is a user defined piece of data passed as the first         */
/*   argument to all of the interface functions.                             */
/*                                                                           */
/* - print callback function should direct the msg argument to the log,      */
/*   status, and/or listing files indicated by the mode.  The mode should    */
/*   be checked for each destination as we can request output to more than   */
/*   destination at a time.  I.e.                                            */
/*     if (mode & Output_Log) print to log file                              */
/*     if (mode & Output_Status) print to status file                        */
/*     if (mode & Output_Listing) print to listing file                      */
/*                                                                           */
/* - flush callback function should flush any buffers associated with the    */
/*   log, status, and/or listing files indicated by the mode.  The mode      */
/*   should be checked for each destination as we can request flushing       */
/*   output for more than one destination at a time.  I.e.                   */
/*     if (mode & Output_Log) flush log file                                 */
/*     if (mode & Output_Status) flush status file                           */
/*     if (mode & Output_Listing) flush listing file                         */
/*   NOTE: the flush callback can be initialized to the NULL pointer without */
/*   any negative effects.                                                   */
/*                                                                           */
/*****************************************************************************/

typedef struct
{
  Void *output_data;

  Void (CB_FPTR print)(Void *data, Int mode, Char *msg);
  Void (CB_FPTR flush)(Void *data, Int mode);
} Output_Interface;

/*****************************************************************************/
/* Interface functions.                                                      */
/*****************************************************************************/
/*                                                                           */
/* Output_Default      - reset the Output_Interface to the default           */
/*                       implemetation.                                      */
/*                                                                           */
/* Output_SetInterface - set the Output_Interface.  This function should be  */
/*                       called during the system setup before calling any   */
/*                       option, creation, or algorithm routines.            */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Void) Output_Default(Void);
FUN_DECL(Void) Output_SetInterface(Output_Interface *i);

/*****************************************************************************/
/* Default implementation functions.                                         */
/*                                                                           */
/* NOTE: these functions only effect the default implementation.             */
/*****************************************************************************/
/*                                                                           */
/* Output_SetLog     - redirect the log to the indicated opened file.  If    */
/*                     NULL is passed in as the argument nothing will be     */
/*                     written to the log.                                   */
/*                                                                           */
/* Output_SetStatus  - redirect the status to the indicated opened file.  If */
/*                     NULL is passed in as the argument nothing will be     */
/*                     written to the status.                                */
/*                                                                           */
/* Output_SetListing - redirect the listing to the indicated opened file.    */
/*                     If NULL is passed in as the argument nothing will be  */
/*                     written to the listing.                               */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Void) Output_SetLog(FILE *f);
FUN_DECL(Void) Output_SetStatus(FILE *f);
FUN_DECL(Void) Output_SetListing(FILE *f);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef OUTPUT_INTERFACE_H */
