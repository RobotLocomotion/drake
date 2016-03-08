/*  The problem solved is:

        min 0.5(x'Qx) + c'x
        s.t  Ax ? b
             l <= x <= u

    Q must be symmetric.  Q should also be a positive semidefinite matrix.

    The QP uses the (i, j, data) format for inputing sparse matrices (Q, A).
    We adopt the FORTRAN convention that the indices begin at 1.

    Here is a description of each of the arguments to the QP_Create functions:

        - variables - the number of variables in the problem
        - constraints - the number of constraints in the problem
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
  QP_Error
} QP_Termination;

void simpleQP(int variables, int constraints,
              int q_nnz, int *q_i, int *q_j, double *q_ij, double *c,
              int a_nnz, int *a_i, int *a_j, double *a_ij, double *b,
              int *c_type, double *lb, double *ub,
              QP_Termination *status, double *z, double *pi, double *obj);

#endif
