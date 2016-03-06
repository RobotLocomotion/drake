/*****************************************************************************/
/* MCP_Interface.h                                                           */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   The mixed complementarity problem is to find an x_star such that for    */
/*   each i, at least one of the following hold:                             */
/*     1.  F_i(x_star) = 0, lb_i <= (x_star)_i <= ub_i                       */
/*     2.  F_i(x_star) > 0, (x_star)_i = lb_i                                */
/*     3.  F_i(x_star) < 0, (x_star)_i = ub_i                                */
/*   where F is a given function from R^n to R^n, and lb and ub are          */
/*   prescribed lower and upper bounds.                                      */
/*                                                                           */
/*   The function, jacobian, lower, and upper bounds are communicated        */
/*   through a MCP_Interface, which must be set in an allocated MCP          */
/*   structure using the MCP_SetInterface( ) function before calling any     */
/*   algorithms or conversion routines.                                      */
/*****************************************************************************/

#ifndef MCP_INTERFACE_H
#define MCP_INTERFACE_H

#include "Types.h"
#include "NLP_Interface.h"
#include "Presolve_Interface.h"

struct _MCP;
typedef struct _MCP MCP;

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Allocation and deallocation functions.                                    */
/*****************************************************************************/
/*                                                                           */
/* - MCP_Create -  allocate a MCS structure with the given estimate of the   */
/*                 problem size and number of nonzeros in the Jacobian.      */
/*                 If the estimated size passed into the creation routine    */
/*                 as smaller than the actual problem size, the code will    */
/*                 reallocate memory.                                        */
/*                                                                           */
/* - MCP_Destroy - deallocate the given MCP structure.  Further use of the   */
/*                 structure is prohibited.                                  */
/*                                                                           */
/* - MCP_Size    - reallocate the MCP structure with the new problem size    */
/*                 and number of nonzeros given.  The reallocation only      */
/*                 occurs if the new size is larger than the current size of */
/*                 the given MCP structure.                                  */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(MCP *) MCP_Create(Int maxModSize, Int maxModNNZ);
FUN_DECL(Void)  MCP_Destroy(MCP *m);
FUN_DECL(Void)  MCP_Size(MCP *m, Int maxModSize, Int maxModNNZ);

/*****************************************************************************/
/* Access routines - used to obtain information about the problem.  Do NOT   */
/*                   directly modify any of the data provided!  This can     */
/*                   cause really bad problems.                              */
/*****************************************************************************/
/*                                                                           */
/* - MCP_GetX - obtain the current x vector                                  */
/* - MCP_GetL - obtain the current lower bound vector                        */
/* - MCP_GetU - obtain the current upper bound vector                        */
/* - MCP_GetB - obtain the current basis indication vector                   */
/*                                                                           */
/* - MCP_GetF - obtain the current function evaluation vector                */
/* - MCP_GetJ - obtain the current Jacobian evaluation matrix                */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Double *) MCP_GetX(MCP *m);
FUN_DECL(Double *) MCP_GetL(MCP *m);
FUN_DECL(Double *) MCP_GetU(MCP *m);
FUN_DECL(Int *)    MCP_GetB(MCP *m);

FUN_DECL(Double *) MCP_GetF(MCP *m);
FUN_DECL(Void)     MCP_GetJ(MCP *m, Int *nz, Int **col, Int **len,
                            Int **row, Double **data);

/*****************************************************************************/
/* MCP_Interface declaration.                                                */
/*****************************************************************************/
/*                                                                           */
/* - interface_data is a user defined piece of data passed as the first      */
/*   argument to all of the interface functions.                             */
/*                                                                           */
/* - problem_size should fill in the size of the problem.  The number of     */
/*   nonzeros estimate must be an upper bound on the actual number of        */
/*   nonzeros that will be encountered.                                      */
/*                                                                           */
/* - bounds should fill in an initial point, and the lower and upper         */
/*   bounds on the problem.  The input lower and upper bounds are            */
/*   initialized to minus and plus infinity, respectively.  Only modify      */
/*   them to indicate the finite bounds on your problem.                     */
/*                                                                           */
/* - function_evaluations should perform a function evaluation at x,         */
/*   placing the result in f and returning the number of domain violations   */
/*   (i.e. divisions by zero)                                                */
/*                                                                           */
/* - jacobian_evaluation should:                                             */
/*       1)  Perform a function evaluation if wantf is true.                 */
/*       2)  Fill in the jacobian of F at x in compressed sparse column      */
/*           format, filling in nnz.  col[i] is the column start in          */
/*           [1, ..., nnz] in the row/data vector, len[i] is the number of   */
/*           nonzeros in the column, row[j] is the row index in [1, ..., n]  */
/*           data[j] is the value of the element.  NOTE: this uses FORTRAN   */
/*           style indices in col and row!  The function returns the number  */
/*           of domain violations encountered.                               */
/*                                                                           */
/* - start and finish are not required, but will be called at the start and  */
/*   end of the path call respectively.                                      */
/*                                                                           */
/* - variable_name and constraint_name are not required but can be used to   */
/*   tell the names of the variable/constraint.  The index given is between  */
/*   1 and n.  I.e. it is a FORTRAN style index.                             */
/*                                                                           */
/* - basis is not required, but is a function used to communicate an initial */
/*   basis to the code.                                                      */
/*                                                                           */
/*****************************************************************************/

typedef struct
{
  Void *interface_data;

  Void (CB_FPTR problem_size)(Void *id, Int *size, Int *nnz);
  Void (CB_FPTR bounds)(Void *id, Int size,
                        Double *x, Double *l, Double *u);

  Int (CB_FPTR function_evaluation)(Void *id, Int n, Double *x, Double *f);
  Int (CB_FPTR jacobian_evaluation)(Void *id, Int n, Double *x,
                                    Int wantf, Double *f,
                                    Int *nnz, Int *col, Int *len,
                                    Int *row, Double *data);

  /***************************************************************************/
  /* Hessian evaluation takes the values for the variables and lagrange      */
  /* multipliers as arguments.  The values for the variables are guaranteed  */
  /* to be the same as those from the last call to the function or Jacobian  */
  /* evaluation.  We want the full matrix (not just the upper triangular     */
  /* part).  This function should compute:                                   */
  /*      H = sum_i lambda[i] * nabla^2 F_i(x)                               */
  /***************************************************************************/
  Int (CB_FPTR hessian_evaluation)(Void *id, Int n, Double *x, Double *l,
                                   Int *nnz, Int *col, Int *len,
                                   Int *row, Double *data);

  /***************************************************************************/
  /* The following functions are not required.  If they are not provided,    */
  /* simply fill in NULL for the value.  The code reacts appropriately in    */
  /* such circumstances.                                                     */
  /***************************************************************************/

  Void (CB_FPTR start)(Void *id);
  Void (CB_FPTR finish)(Void *id, Double *x);
  Void (CB_FPTR variable_name)(Void *id, Int variable,
                               Char *buffer, Int buf_size);
  Void (CB_FPTR constraint_name)(Void *id, Int constr,
                                 Char *buffer, Int buf_size);
  Void (CB_FPTR basis)(Void *id, Int size, Int *basX);
} MCP_Interface;

/*****************************************************************************/
/* Interface functions.                                                      */
/*****************************************************************************/
/*                                                                           */
/* MCP_SetInterface - set the MCP_Interface for an allocated MCP.  This      */
/*                    function must be called before using the MCP in an     */
/*                    algorithm or conversion routine.                       */
/*                                                                           */
/* MCP_GetInterface - obtain the MCP_Interface for the provided MCP.         */
/*                                                                           */
/* MCP_SetPresolveInterface - set the Presolve_Interface for an allocated    */
/*                            MCP.  If preprocessing is going to be used,    */
/*                            this function must be called before using the  */
/*                            MCP in an algorithm or conversion routine.     */
/*                                                                           */
/* MCP_GetPresolveInterface - obtain the Presolve_Interface for the provided */
/*                            MCP.                                           */
/*                                                                           */
/* MCP_SetNLPInterface - set the NLP_Interface for an allocated MCP.  If     */
/*                       you want to use this, it should be set before       */
/*                       using the MCP in any algorithm or conversion        */
/*                       routine.                                            */
/*                                                                           */
/* MCP_GetNLPInterface - obtain the NLP_Interface for the provided MCP.      */
/*                                                                           */
/* NOTE: The NLP interface has effects on searches used by the algorithm.    */
/* The recommendation is to NOT set an NLP interface.                        */
/*****************************************************************************/

FUN_DECL(Void) MCP_SetInterface(MCP *m, MCP_Interface *i);
FUN_DECL(Void) MCP_GetInterface(MCP *m, MCP_Interface *i);

FUN_DECL(Void) MCP_SetPresolveInterface(MCP *m, Presolve_Interface *i);
FUN_DECL(Void) MCP_GetPresolveInterface(MCP *m, Presolve_Interface *i);

FUN_DECL(Void) MCP_SetNLPInterface(MCP *m, NLP_Interface *i);
FUN_DECL(Void) MCP_GetNLPInterface(MCP *m, NLP_Interface *i);

FUN_DECL(Void)   MCP_SetNLP_Lambda(MCP *m, Double l);
FUN_DECL(Double) MCP_GetNLP_Lambda(MCP *m);

/*****************************************************************************/
/* Jacobian flag functions - used to provide information about the structure */
/*                           of the Jacobian provided by the interface.      */
/*                           Make sure that if you set a flag, your jacobian */
/*                           evaluation routine obeys the convention.        */
/*                           Failure to do so can lead to problems.          */
/*                                                                           */
/* NOTE: in order for the presolve to work, you MUST have a constant         */
/*       Jacobian structure.                                                 */
/*****************************************************************************/
/*                                                                           */
/* MCP_Jacobian_Structure_Constant - if set to True, the structure of the    */
/*                                   Jacobian does not change when a new     */
/*                                   evaluation is performed; only the data  */
/*                                   changes.  A constant Jacobian structure */
/*                                   is a prerequisite for preprocessing.    */
/*                                                                           */
/* MCP_Jacobian_Data_Contiguous    - if set to True, the Jacobian data is    */
/*                                   stored continuously from [1..nnz] in    */
/*                                   data and row arrays.                    */
/*                                                                           */
/* MCP_Jacobian_Diagonal           - if set to True, each column in the      */
/*                                   Jacobian has an element on the          */
/*                                   diagonal.                               */
/*                                                                           */
/* MCP_Jacobian_Diagonal_First     - if set to True, for each column in the  */
/*                                   Jacobian having a diagonal element, the */
/*                                   diagonal element is first in the column */
/*                                   listing.                                */
/*                                                                           */
/*****************************************************************************/

FUN_DECL(Void) MCP_Jacobian_Structure_Constant(MCP *m, Boolean b);
FUN_DECL(Void) MCP_Jacobian_Data_Contiguous(MCP *m, Boolean b);
FUN_DECL(Void) MCP_Jacobian_Diagonal(MCP *m, Boolean b);
FUN_DECL(Void) MCP_Jacobian_Diagonal_First(MCP *m, Boolean b);

FUN_DECL(Void) MCP_Hessian_Structure_Constant(MCP *m, Boolean b);
FUN_DECL(Void) MCP_Hessian_Data_Contiguous(MCP *m, Boolean b);
FUN_DECL(Void) MCP_Hessian_Diagonal(MCP *m, Boolean b);
FUN_DECL(Void) MCP_Hessian_Diagonal_First(MCP *m, Boolean b);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef MCP_INTERFACE_H */
