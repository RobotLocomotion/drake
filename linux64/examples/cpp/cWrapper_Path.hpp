#if ! defined(_CWRAPPER_HPP_)
#define       _CWRAPPER_HPP_

/* cWrapper is a C-style wrapper used as an example of calling Path from C++ */

/*****************************************************************************/
/* The main path routine takes the following as arguments:                   */
/*     n      - the number of variables in the problem                       */
/*     nnz    - the number of nonzeros in the jacobian                       */
/*     z      - a starting point for the problem                             */
/*     lb     - lower bounds on the variables                                */
/*     ub     - upper bounds on the variables                                */
/*                                                                           */
/* The algorithm returns the following:                                      */
/*     status - final status of the problem, one of the following:           */
/*                  1 - solved                                               */
/*                  2 - stationary point found                               */
/*                  3 - major iteration limit                                */
/*                  4 - cumulative minor iteration limit                     */
/*                  5 - time limit                                           */
/*                  6 - user interrupt                                       */
/*                  7 - bound error (lb is not less than ub)                 */
/*                  8 - domain error (could not find a starting point)       */
/*                  9 - internal error                                       */
/*     z      - final point                                                  */
/*     f      - final function evaluation                                    */
/*                                                                           */
/* On input, z, f, lb, and ub must be vectors with at least n elements.      */
/* For infinity, use 1e20.                                                   */
/*****************************************************************************/

void pathMain(int n, int nnz, int *status, 
               double *z, double *f, double *lb, double *ub);

/*****************************************************************************/
/* You must provide the following two functions:                             */
/*     funcEval - evaluate the function at z, placing the result in f.       */
/*                Return the number of domain violations.                    */
/*     jacEval  - evaluate the Jacobian at z, placing the result in          */
/*                col_start, col_len, row, and data.  Return the number      */
/*                of domain violations.                                      */
/*                                                                           */
/* Note: all indices in the Jacobian begin with one.  The Jacobian is stored */
/* by columns.  Finally, if the structure of the Jacobian remains constant,  */
/* col_start, col_len, and row need only be filled once.                     */
/*****************************************************************************/

int funcEval(int n, double *z, double *f);
int jacEval(int n, int nnz, double *z, int *col_start, int *col_len,
            int *row, double *data);


#endif /* ! defined(_CWRAPPER_HPP_) */
