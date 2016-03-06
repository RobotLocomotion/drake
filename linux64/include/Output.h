/*****************************************************************************/
/* Output.h                                                                  */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Functions used to output messages.                                      */
/*****************************************************************************/

#ifndef OUTPUT_H
#define OUTPUT_H

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Output routines.                                                          */
/*****************************************************************************/
/*                                                                           */
/* - Output_Printf - print the message to the locations indicated by the     */
/*                   mode (log, status, and/or listing file).  The fmt       */
/*                   argument is the same as is used for the C standard      */
/*                   printf routine.  This function uses the current         */
/*                   Output_Interface.                                       */
/*                                                                           */
/* - Output_Flush  - flush any buffers for the locations indicated by the    */
/*                   mode (log, status, and/or listing file).  This function */
/*                   uses the current Output_Interface.                      */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Void) Output_Printf(Int mode, const Char *fmt, ...);
FUN_DECL(Void) Output_Flush(Int mode);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef OUTPUT_H */
