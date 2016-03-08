/*  The problem solved is:

        min 0.5(x'Qx) + c'x
        s.t  Ax ? b
             l <= x <= u

    Q must be symmetric.  Since it is symmetric, we can pass in just the
    lower triangular part, L.

    The matrix L is passed via the column-pointer, row-index scheme.
    No column length vector is used: the next column start determines a
    column's end.  We adopt the C convention that indices begin at 0.

       - n - the number of variables in the problem
       - m - the number of constraints in the problem
       - q_nnz - the number of nonzeros in the Q matrix
       - q_i - a vector of size q_nnz containing the row indices for Q
       - q_j - a vector of size q_nnz containing the column indices for Q
       - q_ij - a vector of size q_nnz containing the data for Q
       - c - a vector of size variables
       - a_nnz - the number of nonzeros in the A matrix
       - a_i - a vector of size a_nnz containing the row indices for A
       - a_j - a vector of size a_nnz containing the column indices for A
       - a_ij - a vector of size a_nnz containing the data for A
       - b - a vector of size constraints
       - c_type - a vector of size constraints that specifies the type of the
              constraint
       - lb - a vector of size variables containing the lower bounds on x
       - ub - a vector of size variables containing the upper bounds on x
*/

#ifndef SIMPLEQP_H
#define SIMPLEQP_H

#define QP_LE -1
#define QP_EQ 0
#define QP_GE 1

typedef enum {
  QP_Solved,
  QP_Unbounded,
  QP_Infeasible,
  QP_Error,
  QP_Other,
} QP_Termination;

void sparseQP (int n, int m,
               int Lnnz, const int *Li, const int *Lj, const double *Lval,
               const double *c,
               int Annz, const int *AcolPtr, const int *ArowIdx, const double *Aval,
               const double *b,
               const int *conType, const double *lb, const double *ub,
               QP_Termination *status, double *z, double *pi, double *obj);

#endif
