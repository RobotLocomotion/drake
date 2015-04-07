/* Produced by CVXGEN, 2015-04-06 22:08:42 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double Q0[4];
  double Q1[4];
  double Q2[4];
  double x0[6];
  double xd0[6];
  double x1[6];
  double x2[6];
  double t1[1];
  double t0[1];
  double t2[1];
  double tf[1];
  double xf[6];
  double xdf[6];
} Params;
typedef struct Vars_t {
  double *y0_1; /* 2 rows. */
  double *y0_2; /* 2 rows. */
  double *y0_3; /* 2 rows. */
  double *y0_4; /* 2 rows. */
  double *y0_5; /* 2 rows. */
  double *y0_6; /* 2 rows. */
  double *y1_1; /* 2 rows. */
  double *y1_2; /* 2 rows. */
  double *y1_3; /* 2 rows. */
  double *y1_4; /* 2 rows. */
  double *y1_5; /* 2 rows. */
  double *y1_6; /* 2 rows. */
  double *y2_1; /* 2 rows. */
  double *y2_2; /* 2 rows. */
  double *y2_3; /* 2 rows. */
  double *y2_4; /* 2 rows. */
  double *y2_5; /* 2 rows. */
  double *y2_6; /* 2 rows. */
  double *C0_0; /* 6 rows. */
  double *C0_1; /* 6 rows. */
  double *C1_0; /* 6 rows. */
  double *C2_0; /* 6 rows. */
  double *C0_2; /* 6 rows. */
  double *C0_3; /* 6 rows. */
  double *C1_1; /* 6 rows. */
  double *C1_2; /* 6 rows. */
  double *C1_3; /* 6 rows. */
  double *C2_1; /* 6 rows. */
  double *C2_2; /* 6 rows. */
  double *C2_3; /* 6 rows. */
  double *y0[7];
  double *y1[7];
  double *y2[7];
  double *C0[4];
  double *C1[4];
  double *C2[4];
} Vars;
typedef struct Workspace_t {
  double *h;
  double *s_inv;
  double *s_inv_z;
  double b[108];
  double q[108];
  double rhs[216];
  double x[216];
  double *s;
  double *z;
  double *y;
  double lhs_aff[216];
  double lhs_cc[216];
  double buffer[216];
  double buffer2[216];
  double KKT[336];
  double L[480];
  double d[216];
  double v[216];
  double d_inv[216];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
