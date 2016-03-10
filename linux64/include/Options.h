/*****************************************************************************/
/* Options.h                                                                 */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Functions used work with the options.                                   */
/*                                                                           */
/*   The correct procedure for using the options interface is:               */
/*     1. Use Options_Create( ) to create a new option interface.            */
/*     2. Add appropriate options sets.  For user defined options, the       */
/*        Options_Add( ) routine is used.  For PATH, Path_AddOptions( ) is   */
/*        the routine to call.                                               */
/*     3. Set the defaults with Options_Default( ).                          */
/*     4. Modify the options however you desire.                             */
/*                                                                           */
/*   See the algorithm documentation for a listing of the available options. */
/*****************************************************************************/

#ifndef OPTIONS_H
#define OPTIONS_H

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _Options_Interface;
typedef struct _Options_Interface Options_Interface;

struct _Option_Set;
typedef struct _Option_Set Option_Set;

/*****************************************************************************/
/* Allocation and deallocation functions.                                    */
/*****************************************************************************/
/*                                                                           */
/* - Options_Create  - allocate and return an options structure.             */
/* - Options_Destroy - deallocate the give options structure.                */
/* - Options_Add     - add the indicate option set to the options structure. */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Options_Interface *) Options_Create(Void);
FUN_DECL(void) Options_Destroy(Options_Interface *);
FUN_DECL(void) Options_Add(Options_Interface *i, Option_Set *opt);

/*****************************************************************************/
/* Main functionality.                                                       */
/*****************************************************************************/
/*                                                                           */
/* - Options_Default - set all of the options to their default values.       */
/* - Options_Display - display the current values for the options to the log.*/
/* - Options_Read    - read and set specific options from the indicated      */
/*                     file.                                                 */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(void) Options_Default(const Options_Interface *i);
FUN_DECL(void) Options_Display(const Options_Interface *i);
FUN_DECL(int ) Options_Read(const Options_Interface *i, const char *filename);

/*****************************************************************************/
/* Setting options.                                                          */
/*****************************************************************************/
/*                                                                           */
/* - Options_Set        - set option indicated by string.  The string        */
/*                        contains the name of the option and the value      */
/*                        together.                                          */
/*                                                                           */
/* - Options_SetBoolean - set the value for a boolean option.  The string    */
/*                        is the option name.  The value is directly passed  */
/*                        into the function.                                 */
/*                                                                           */
/* - Options_SetDouble  - set the value for a double option.  The string     */
/*                        is the option name.  The value is directly passed  */
/*                        into the function.                                 */
/*                                                                           */
/* - Options_SetInt     - set the value for an integer option.  The string   */
/*                        is the option name.  The value is directly passed  */
/*                        into the function.                                 */
/*                                                                           */
/* - Options_SetOther   - set the value for an option with a different       */
/*                        enumerated type.  The string is the option name.   */
/*                        The value is directly passed into the function and */
/*                        corresponds to the value for the enumeratey type.  */
/*                        NOTE: the Options_SetOther is dangerous to use and */
/*                        we recommend using Options_Set when you want to    */
/*                        set one of the enumerated type options.            */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(int) Options_Set(const Options_Interface *i, const char *option);
FUN_DECL(int) Options_SetBoolean(const Options_Interface *i,
				 const char *option, const Boolean value);
FUN_DECL(int) Options_SetInt(const Options_Interface *i,
			     const char *option, const int value);
FUN_DECL(int) Options_SetDouble(const Options_Interface *i,
				const char *option, const double value);
FUN_DECL(int) Options_SetOther(const Options_Interface *i,
			       const char *option, const int value);

/*****************************************************************************/
/* Obtaining options.                                                        */
/*****************************************************************************/
/*                                                                           */
/* - Options_GetBoolean - get the value for a boolean option.  The string    */
/*                        is the option name.                                */
/*                                                                           */
/* - Options_GetDouble  - get the value for a double option.  The string     */
/*                        is the option name.                                */
/*                                                                           */
/* - Options_GetInt     - get the value for an integer option.  The string   */
/*                        is the option name.                                */
/*                                                                           */
/* - Options_GetOther   - get the value for an option with a different       */
/*                        enumerated type.  The string is the option name.   */
/*                        The value returned corresponds to the value for    */
/*                        the enumerated type.                               */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Boolean) Options_GetBoolean(const Options_Interface *i,
				     const char *option);
FUN_DECL(int)     Options_GetInt(const Options_Interface *i,
				 const char *option);
FUN_DECL(double)  Options_GetDouble(const Options_Interface *i,
				    const char *option);
FUN_DECL(int)     Options_GetOther(const Options_Interface *i,
				   const char *option);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef OPTIONS_H */
