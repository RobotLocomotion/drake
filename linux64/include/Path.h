/*****************************************************************************/
/* Path.h                                                                    */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Functions used to solve a complementarity problem using the Path        */
/*   algorithm.                                                              */
/*                                                                           */
/*   There are four step to solving a problem:                               */
/*     1.  Create and initialize the options                                 */
/*     2.  Create an MCP structure and intialize the MCP_Interface           */
/*     3.  Call Path_Solve to solve the problem.                             */
/*     4.  Delete the MCP_Structure.                                         */
/*                                                                           */
/*   If you are going to call PATH repeatedly, you should allocate a PATH    */
/*   workspace at the beginning and deallocate it at the end and reuse the   */
/*   MCP structure.  This technique will pevent having to allocate and free  */
/*   a bunch of memory for each call.  The steps in this case are:           */
/*     1.  Create and initialize the options                                 */
/*     2.  Use Path_Create to create a workspace                             */
/*     3.  Create an MCP structure and intialize the MCP_Interface           */
/*     4.  Repeatedly call Path_Solve                                        */
/*     5.  Delete the MCP_Structure                                          */
/*     6.  Use Path_Delete to destroy the workspace                          */
/*                                                                           */
/*   NOTE: all of the system interfaces (Error, Interrupt, Memory, Output,   */
/*   and Timer) should be set BEFORE creating and initializing the options   */
/*   if you want to modify these behaviours.                                 */
/*****************************************************************************/

#ifndef PATH_H
#define PATH_H

#include "Types.h"
#include "MCP_Interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Version information.                                                      */
/*****************************************************************************/
/*                                                                           */
/* - Path_Version - obtain an integer for the version number the thousands   */
/*                  place is the major version, the remainder is the minor   */
/*                  version.                                                 */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(const Char *) Path_Version(Void);

/*****************************************************************************/
/* License information.                                                      */
/*****************************************************************************/
/*                                                                           */
/* - Path_CheckLicense - check the license.  Returns nonzero on success.     */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Int) Path_CheckLicense(Int n, Int nnz);

/*****************************************************************************/
/* Workspace manipulation methods.                                           */
/*****************************************************************************/
/*                                                                           */
/* - Path_Create    - create a workspace with the indicated size and         */
/*                    increment the reference count                          */
/*                                                                           */
/* - Path_Destroy   - decrement the reference count and deallocate the       */
/*                    workspace when the reference count becomes zero.       */
/*                                                                           */
/* - Path_Reference - increment the reference count on the workspace.  It is */
/*                    only destroy when the reference count goes to zero.    */
/*                                                                           */
/* - Path_Size      - reallocate the workspace with the new problem size and */
/*                    number of nonzeros given.  The reallocation only       */
/*                    occurs if the new size is larger than the current size */
/*                    of the path workspace.                                 */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Void) Path_Create(Int maxSize, Int maxNNZ);
FUN_DECL(Void) Path_Destroy(Void);
FUN_DECL(Void) Path_Reference(Void);
FUN_DECL(Void) Path_Size(Int maxSize, Int maxNNZ);

/*****************************************************************************/
/* Modify method.                                                            */
/*****************************************************************************/
/*                                                                           */
/* Path_SetProblemClass - set the problem class for the indicated MCP to t.  */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Void) Path_SetProblemClass(MCP *m, MCP_Type i);

/*****************************************************************************/
/* Solve method.                                                             */
/*****************************************************************************/
/*                                                                           */
/* Path_Solve - solve the indicated complementarity problem using path.      */
/*              On input, the generate_output, use_start, and use_basics     */
/*              fields of the information structure should be set.  The      */
/*              remainder are filled by the path at the end of the solve.    */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(MCP_Termination) Path_Solve(MCP *m, Information *info);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PATH_H */
