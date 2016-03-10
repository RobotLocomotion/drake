/*****************************************************************************/
/* Presolve_Interface.h                                                      */
/*                                                                           */
/* DESCRIPTION                                                               */
/*   Presolving the model can have a significant impact upon the model       */
/*   passed along to the solver.  We primarily want to find variables that   */
/*   can be fixed and remove them from the model.  Furthermore, we want to   */
/*   improve the bounds on the variables and perhaps help to find a good     */
/*   starting point.  We want to immediately discard problems with no        */
/*   solution.                                                               */
/*                                                                           */
/*   In order to accomplish these goals, we need to have additional          */
/*   information provided.  These are given in the presolve interface.  All  */
/*   of the functions are optional, however, the more that can be provided,  */
/*   the better our presolve process will be.                                */
/*                                                                           */
/*   A Presolve_Interface can be set for a CNS or an MCP.  It will only be   */
/*   used if the Jacobian is constant!                                       */
/*****************************************************************************/

#ifndef PRESOLVE_INTERFACE_H
#define PRESOLVE_INTERFACE_H

#include "Types.h"

/*****************************************************************************/
/* Constant declarations.                                                    */
/*****************************************************************************/

/* Jacobian type constants.                                                  */
#define PRESOLVE_LINEAR    0    /* Type of element is linear                 */
#define PRESOLVE_NONLINEAR 1    /* Type of element is nonlinear              */

/* Constraint type constants.                                                */
#define PRESOLVE_UNKNOWN 0      /* Variable in unknown constraint set        */
#define PRESOLVE_PRIMAL  1      /* Variable is a primal variable             */
#define PRESOLVE_DUAL   -1      /* Variable is a dual variable               */

/* Proposed interval evaluation constants.                                   */
#define PRESOLVE_NORM  0        /* Value is normal, bound is actual value    */
#define PRESOLVE_POS   1        /* Value is > bound                          */
#define PRESOLVE_NEG  -1        /* Value is < bound                          */
#define PRESOLVE_PINF  2        /* Bound is plus infinity                    */
#define PRESOLVE_MINF -2        /* Bound is minus infinity                   */

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* Presolve_Interface declaration.                                           */
/*****************************************************************************/
/*                                                                           */
/* - presolve_data is a user defined piece of data passed as the first       */
/*   argument to all of the interface functions.                             */
/*                                                                           */
/* - start_pre callback function is called when the presolve starts.         */
/* - start_post callback function is called when the postsolve starts.       */
/*                                                                           */
/* - finish_pre callback function is called when the presolve finishes.      */
/* - finish_post callback function is called when the postsolve finishes.    */
/*                                                                           */
/* - jac_typ callback function should fill in the type (PRESOLVE_LINEAR or   */
/*   PRESOLVE_NONLINEAR) for each element in the Jacobian.  Since we assume  */
/*   the Jacobian has a constant structure, we only pass the number of       */
/*   nonzeros in the Jacobian and a data array for the type.  The typ array  */
/*   passed in is initialized to PRESOLVE_NONLINEAR.                         */
/*                                                                           */
/* - con_typ callback function should fill in the constraint types           */
/*   (PRESOLVE_PRIMAL or PRESOLVE_DUAL).  These should be known if you are   */
/*   solving the Karush-Kuhn-Tucker condition for a linear program,          */
/*   quadratic program, nonlinear program, or variational inequality.  The   */
/*   vector passed in is initialized to PRESOLVE_UNKNOWN.  The presolve      */
/*   procedure will attempt to identify primal/dual structure when nothing   */
/*   is identified by the user.                                              */
/*                                                                           */
/*****************************************************************************/

typedef struct
{
  Void *presolve_data;

  Void (CB_FPTR start_pre)(Void *pd);
  Void (CB_FPTR start_post)(Void *pd);

  Void (CB_FPTR finish_pre)(Void *pd);
  Void (CB_FPTR finish_post)(Void *pd);

  Void (CB_FPTR jac_typ)(Void *pd, Int nnz, Int *typ);
  Void (CB_FPTR con_typ)(Void *pd, Int n, Int *typ);

#if 0

  /***************************************************************************/
  /* The following a proposed functions that might appear in future versions */
  /* of the presolve interface.                                              */
  /***************************************************************************/

  Void (CB_FPTR var_fix)(Void *pd, Int var, Double val);

  Void (CB_FPTR fun_typ)(Void *pd, Int n, Int *typ);
  Void (CB_FPTR fun_domain)(Void *pd, Int func,
                            Double *xlb, Double *xub,
                            Int *xlb_type, Int *xub_type);
  Void (CB_FPTR fun_interval)(Void *pd, Int func,
                              const Double *x_lb, const Double *x_ub,
                              Double *f_lb, Double *f_ub,
                              Int *flb_type, Int *fub_type);

  Void (CB_FPTR jac_domain)(Void *pd, Int func,
                            Double *xlb, Double *xub,
                            Int *xlb_type, Int *xub_type);
  Void (CB_FPTR jac_interval)(Void *pd, Int func,
                              const Double *x_lb, Const double *x_ub,
                              Double *j_lb, Double *j_ub,
                              Int *jlb_type, Int *jub_type);
#endif
} Presolve_Interface;

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PRESOLVE_INTERFACE_H */
