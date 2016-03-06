#ifndef STANDALONE_H
#define STANDALONE_H

/*****************************************************************************/
/* The main routine takes the following as arguments:                        */
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

/*****************************************************************************/
/* You must provide the following two functions:                             */
/*     FUNCEVAL - evaluate the function at z, placing the result in f        */
/*                Return the number of domain violations.                    */
/*         n    - size of the vectors (input)                                */
/*         z    - vector of size n (input), point at which to evaluate f     */
/*         f    - vector of size n (output), functions evaluation at z       */
/*                                                                           */
/*     JACEVAL  - evaluate the Jacobian at z, placing the result in          */
/*                col_start, col_len, row, and data.  Return the number      */
/*                of domain violations.                                      */
/*         n    - size of the vectors (input)                                */
/*         nnz  - allocated number of nonzeros (input)                       */
/*                actual number of nonzeros(output)                          */
/*         z    - vector of size n (input), point at which to evaluate jac.  */
/*         col_start - start of the column in row/data array (output)        */
/*         col_len   - number of elements in the column (output)             */
/*         row       - row index for element (output)                        */
/*         data      - value of jacobian for element (output)                */
/*                                                                           */
/* Note: all indices in the Jacobian begin with one.  The Jacobian is stored */
/* by columns.  Finally, if the structure of the Jacobian remains constant,  */
/* col_start, col_len, and row need only be filled once.                     */
/*****************************************************************************/


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
#define FUNCEVAL funceval_
#define JACEVAL  jaceval_
#define MYMAIN   pathmain_
#define SET_LOG  set_log_
#elif defined(FNAME_LCASE_NODECOR) /* fortran names: lower case, no _ */
#define FUNCEVAL funceval
#define JACEVAL  jaceval
#define MYMAIN   pathmain
#define SET_LOG  set_log
#elif defined(FNAME_UCASE_DECOR) /* fortran names: upper case, trailing _ */
#define FUNCEVAL FUNCEVAL_
#define JACEVAL  JACEVAL_
#define MYMAIN   PATHMAIN_
#define SET_LOG  SET_LOG_
#elif defined(FNAME_UCASE_NODECOR) /* fortran names: upper case, no _ */
#define FUNCEVAL FUNCEVAL
#define JACEVAL  JACEVAL
#define MYMAIN   PATHMAIN
#define SET_LOG  SET_LOG
#else
#error "No compile define for fortran naming convention"
No_compile_define_for_fortran_naming_convention;
#endif

void F_CALLCONV MYMAIN(int *n, int *nnz, int *status,
                       double *z, double *f, double *lb, double *ub);

int F_CALLCONV FUNCEVAL(int *n, double *z, double *f);
int F_CALLCONV JACEVAL(int *n, int *nnz, double *z,
                       int *col_start, int *col_len, int *row, double *data);

void F_CALLCONV SET_LOG(int *l);

#endif
