/*****************************************************************************
 * obstacle_c.cpp
 *
 * Implementation of the obstacle problem as described in "MCPLIB: A
 * Collection of Nonlinear Mixed Complementarity Problems" by Dirkse and
 * Ferris in Optimization Methods and Software, 5, 1995, 319-345.
 *
 * In this example, we have a C++ driver but we really implement the model
 * in more of a C style.
 *****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>

#include "cWrapper_Path.hpp"
#include "Types.h"

/*****************************************************************************
 * We will use a constant force.
 *****************************************************************************/

#define FORCE_CONSTANT 1.0

static int N;                   /* The number of elements on the X and Y axes*/
static int fill_structure;      /* Do we need to fill in the structure of    */
                                /* the Jacobian?                             */

using namespace std;

int funcEval(int n, double *z, double *f)
{
  /************************************************************************/
  /* Evaluate the function at z, placing the result in f.                 */
  /************************************************************************/

  int i, j, l;
  double dt;

  /************************************************************************/
  /* The number of variables is N*N, the interior vertices of the         */
  /* triangularization.                                                   */
  /************************************************************************/

  if (n != N*N) {
    fprintf (stderr, "Bad dimension argument 'n' to F: exiting . . .\n");
    exit (-1);
  }

  dt = -(FORCE_CONSTANT) / ((N + 1.0)*(N + 1.0));

  /************************************************************************/
  /* Calculate the function value.                                        */
  /************************************************************************/

  f[0] = dt + 4*z[0] - z[1] - z[N];
  for (i = 1;  i < N-1; i++) {
    f[i] = dt + 4*z[i] - z[i-1] - z[i+1] - z[i+N];
  }
  f[N-1] = dt + 4*z[N-1] - z[N-2] - z[N-1+N];

  for (j = 1;  j < N-1;  j++) {
    l = j*N;
    f[l] = dt + 4*z[l] - z[l+1] - z[l-N] - z[l+N];
    for (i = 1;  i < N-1; i++) {
      f[l+i] = dt + 4*z[l+i] - z[l+i-1] - 
        z[l+i+1] - z[l+i-N] - z[l+i+N];
    }
    f[l+N-1] = dt + 4*z[l+N-1] - z[l+N-2] - z[l-1] - z[l+N-1+N];
  }

  l = (N-1)*N;
  f[l] = dt + 4*z[l] - z[l+1] - z[l-N];
  for (i = 1;  i < N-1; i++) {
    f[l+i] = dt + 4*z[l+i] - z[l+i-1] - z[l+i+1] - z[l+i-N];
  }
  f[l+N-1] = dt + 4*z[l+N-1] - z[l+N-2] - z[l-1];
  return 0;
}

int jacEval(int n, int nnz, double *z, int *col_start, int *col_len,
            int *row, double *data)
{
  /************************************************************************/
  /* Evaluate the jacobian at z, placing the result in col_start, col_len,*/
  /* row, and data.                                                       */
  /*                                                                      */
  /* Note: We only need to fill in the structure of the jacobian          */
  /* (col_start, col_len, row) once.  We do this by looking at the        */
  /* fill_structure variable.                                             */
  /************************************************************************/

  int i, j, k, l;

  /************************************************************************/
  /* The number of variables is N*N, the interior vertices of the         */
  /* triangularization.                                                   */
  /************************************************************************/

  if (n != N*N) {
    fprintf (stderr, "Bad dimension argument 'n' to "
             "sparseJ: exiting . . .\n");
    exit (-1);
  }

  /************************************************************************/
  /* The number of nonzeros in the difference approximation is 5*N*N - 4*N*/
  /* Exit if there is not enough room allocated.                          */
  /************************************************************************/

  if (nnz < 5*N*N - 4*N) {
    fprintf (stderr, "Bad dimension argument 'nnz' to "
             "sparseJ: exiting . . .\n");
    exit (-1);
  }

  if (fill_structure) {
    /*******************************************************************/
    /* This is the first call to jacEval, fill in the structure.       */
    /*******************************************************************/

    k = 0;
    l = 0;

    col_start[k] = 1;
    row[l++] = 1;
    row[l++] = 2;
    row[l++] = N + 1;
    col_len[k] = l;
    k++;

    for (i = 1;  i < N-1;  i++) {
      col_start[k] = l + 1;
      row[l++] = k;
      row[l++] = k + 1;
      row[l++] = k + 2;
      row[l++] = k + N + 1;
      col_len[k] = l - col_start[k] + 1;
      k++;
    }

    col_start[k] = l + 1;
    row[l++] = k;
    row[l++] = k + 1;
    row[l++] = k + N + 1;
    col_len[k] = l - col_start[k] + 1;
    k++;

    for (j = 1;  j < N-1;  j++) {
      col_start[k] = l + 1;
      row[l++] = k - N + 1;
      row[l++] = k + 1;
      row[l++] = k + 2;
      row[l++] = k + N + 1;
      col_len[k] = l - col_start[k] + 1;
      k++;

      for (i = 1;  i < N-1;  i++) {
        col_start[k] = l + 1;
        row[l++] = k - N + 1;
        row[l++] = k;
        row[l++] = k + 1;
        row[l++] = k + 2;
        row[l++] = k + N + 1;
        col_len[k] = l - col_start[k] + 1;
        k++;
      }

      col_start[k] = l + 1;
      row[l++] = k - N + 1;
      row[l++] = k;
      row[l++] = k + 1;
      row[l++] = k + N + 1;
      col_len[k] = l - col_start[k] + 1;
      k++;
    }

    col_start[k] = l + 1;
    row[l++] = k - N + 1;
    row[l++] = k + 1;
    row[l++] = k + 2;
    col_len[k] = l - col_start[k] + 1;
    k++;

    for (i = 1;  i < N-1;  i++) {
      col_start[k] = l + 1;
      row[l++] = k - N + 1;
      row[l++] = k;
      row[l++] = k + 1;
      row[l++] = k + 2;
      col_len[k] = l - col_start[k] + 1;
      k++;
    }

    col_start[k] = l + 1;
    row[l++] = k - N + 1;
    row[l++] = k;
    row[l++] = k + 1;
    col_len[k] = l - col_start[k] + 1;
    k++;

    fill_structure = 0;
  }

  /************************************************************************/
  /* Fill in the data values for the jacobian.                            */
  /************************************************************************/

  l = 0;

  data[l++] = 4;
  data[l++] = -1;
  data[l++] = -1;
  for (i = 1;  i < N-1;  i++) {
    data[l++] = -1;
    data[l++] = 4;
    data[l++] = -1;
    data[l++] = -1;
  }

  data[l++] = -1;
  data[l++] = 4;
  data[l++] = -1;

  for (j = 1;  j < N-1;  j++) {
    data[l++] = -1;
    data[l++] = 4;
    data[l++] = -1;
    data[l++] = -1;

    for (i = 1;  i < N-1;  i++) {
      data[l++] = -1;
      data[l++] = -1;
      data[l++] = 4;
      data[l++] = -1;
      data[l++] = -1;
    }

    data[l++] = -1;
    data[l++] = -1;
    data[l++] = 4;
    data[l++] = -1;
  }

  data[l++] = -1;
  data[l++] = 4;
  data[l++] = -1;

  for (i = 1;  i < N-1;  i++) {
    data[l++] = -1;
    data[l++] = -1;
    data[l++] = 4;
    data[l++] = -1;
  }

  data[l++] = -1;
  data[l++] = -1;
  data[l++] = 4;
  return 0;
} /* jacEval */

#define MIN(A,B) ((A) < (B)) ? (A) : (B)

double resid (int n, const double z[], const double f[],
              const double lb[], const double ub[])
{
  double loTmp, upTmp, t, r;
  int i;

  r = 0;
  for (i = 0;  i < n;  i++) {
    assert(z[i] >= lb[i]);
    assert(z[i] <= ub[i]);
    if (f[i] > 0) {
      t = MIN(f[i],z[i]-lb[i]);
      r += t;
    }
    else {
      t = MIN(-f[i], ub[i]-z[i]);
      r += t;
    }
  }
  return r;
} /* resid */

int main (int argc, char **argv)
{
  double *lb;          /* Lower bounds on the variables             */
  double *ub;          /* Upper bounds on the variables             */
  double *z;           /* Solution vector                           */
  double *f;           /* Function evaluation                       */
  double r;

  int n;               /* Number of variable (N*N)                  */
  int nnz;             /* Number of nonzeros (5*N*N - 4*N)          */
  int status;          /* Termination status from PATH              */
  int i, j;

  /************************************************************************/
  /* Read in the number of elements to use on the X and Y axes.           */
  /************************************************************************/

  if (argc > 1) {
    N = atoi (argv[1]);
    if (N < 2)
      N = 2;
  }
  else {
    printf("Input n: ");
    status = scanf("%d", &N);
    if ((status != 1) || (N < 2)) {
      N = 2;
    }
  }

  /************************************************************************/
  /* Determine n (number of variables) and nnz (number of nonzeros)       */
  /************************************************************************/

  n = N*N;
  nnz = 5*N*N - 4*N;

  /************************************************************************/
  /* Allocate space for the bounds, starting point, and function value.   */
  /************************************************************************/

  lb = (double *)malloc(sizeof(double)*n);
  ub = (double *)malloc(sizeof(double)*n);
  z = (double *)malloc(sizeof(double)*n);
  f = (double *)malloc(sizeof(double)*n);

  /************************************************************************/
  /* Fill in the lower and upper bounds and a starting point.             */
  /************************************************************************/

  for (j = 0; j < N; j++) {
    for (i = 0; i < N; i++) {
      lb[i + j*N] = 
        pow(sin(9.2*i/(N+1.0))*sin(9.3*j/(N+1.0)), 3.0);
      ub[i + j*N] = 
        pow(sin(9.2*i/(N+1.0))*sin(9.3*j/(N+1.0)), 2.0) + 0.02;
      z[i + j*N] = 0.1;
    }
  }

  /************************************************************************/
  /* Set fill_structure to true (we ALWAYS need to fill in the structure  */
  /* of the jacobian at least once per call to PATH).                     */
  /************************************************************************/

  fill_structure = 1;

  /************************************************************************/
  /* Call PATH.                                                           */
  /************************************************************************/

  pathMain (n, nnz, &status, z, f, lb, ub);

  if (MCP_Solved != status)
    return EXIT_FAILURE;
  (void) funcEval (n, z, f);
  r = resid (n, z, f, lb, ub);
  if (r > 1e-6) {
    printf ("Residual of %g is too large: failure\n", r);
    return EXIT_FAILURE;
  }
  else
    printf ("Residual of %g is OK\n", r);

  /************************************************************************/
  /* Deallocate memory.                                                   */
  /************************************************************************/

  free(lb);
  free(ub);
  free(z);
  free(f);

  return EXIT_SUCCESS;
} /* main */
