/*****************************************************************************/
/* NLP_Interface.h                                                           */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Used to convey extra information to the algorithm when you are solving  */
/*   for the Karush-Kuhn-Tucker conditions for a nonlinear program.  The     */
/*   objective function in this case can be used during the merit function   */
/*   evaluation.  We simply use a convex combination of the normal           */
/*   complementarity merit function and the objective function.  By          */
/*   specifying an NLP_Interface, you are invoking this heuristic.  If you   */
/*   do not want this behavior, set the NLP_Interface to NULL.               */
/*                                                                           */
/*  The recommendation is to NOT use NLP interface.                          */
/*****************************************************************************/

#ifndef NLP_INTERFACE_H
#define NLP_INTERFACE_H

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* NLP_Interface declaration.                                                */
/*****************************************************************************/
/*                                                                           */
/* - nlp_data is a user defined piece of data passed as the first argument   */
/*   to all of the interface functions.                                      */
/*                                                                           */
/* - function_evaluation - obtain the objective function value at x and      */
/*                         place it in f.                                    */
/*                                                                           */
/* - gradient_evaluation - obtain the gradient of the objective function at  */
/*                         x and place it in g.  If (wantf) is true, then    */
/*                         also provide the objective function value in f.   */
/*                                                                           */
/* - hessian_evaluation  - obtain the hessian of the objective function at   */
/*                         x.  This is not currently used.                   */
/*                                                                           */
/* - start and finish are not required, but will be called at the start and  */
/*   end of the path call respectively.                                      */
/*                                                                           */
/*****************************************************************************/

typedef struct
{
  Void *nlp_data;

  Int (CB_FPTR function_evaluation)(Void *id, Int n, Double *x, Double *f);
  Int (CB_FPTR gradient_evaluation)(Void *id, Int n, Double *x,
                                    Int wantf, Double *f, Double *g);
  Int (CB_FPTR hessian_evaluation)(Void *id, Int n, Double *x,
                                   Int wantf, Double *f,
                                   Int wantg, Double *g,
                                   Int *nnz, Int *col, Int *len,
                                   Int *row, Double *data);

  /***************************************************************************/
  /* The following functions are not required.  If they are not provided,    */
  /* simply fill in NULL for the value.  The code reacts appropriately in    */
  /* such circumstances.                                                     */
  /***************************************************************************/

  Void (CB_FPTR start)(Void *id);
  Void (CB_FPTR finish)(Void *id, Double *x);
} NLP_Interface;

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef NLP_INTERFACE_H */
