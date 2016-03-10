/* ijvalDriver.c
 * This is a simple driver that reads a QP specified in "i j val" form
 * from a file and solves it as an MCP using the simpleQP interface
 */

#include "simpleQP.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main (int argc, char **argv)
{
  QP_Termination termination;
  char filename[1024];
  FILE *fp;

  int variables;
  int constraints;

  double *x_end;
  double *pi_end;

  int q_nnz;
  int *q_i;
  int *q_j;
  double *q_ij;
  double *c;

  int a_nnz;
  int *a_i;
  int *a_j;
  double *a_ij;
  double *b;
  int *c_type;
  double *lb;
  double *ub;
  double obj;
  int i;

  if (argc <= 1) {
    /* Read in data from a file. */
    printf("Enter filename: ");
    scanf("%s", filename);
  }
  else {
    strncpy (filename, argv[1], sizeof(filename)-1);
    filename[sizeof(filename)-1] = '\0';
  }

  if ((fp = fopen(filename, "r")) == NULL) {
    perror("fopen");
    printf("QP data file '%s' could not be opened\n", filename);
    return 1;
  }
  printf("Reading QP from file '%s'\n", filename);

  fscanf(fp, "%d%d", &variables, &constraints);

  x_end = (double *) malloc(sizeof(double)*variables + 1);
  pi_end = (double *) malloc(sizeof(double)*constraints + 1);

  fscanf(fp, "%d", &q_nnz);

  q_i = (int *) malloc(sizeof(int)*q_nnz + 1);
  q_j = (int *) malloc(sizeof(int)*q_nnz + 1);
  q_ij = (double *) malloc(sizeof(double)*q_nnz + 1);
  c = (double *) malloc(sizeof(double)*variables + 1);

  for (i = 0; i < q_nnz; i++) {
    fscanf(fp, "%d%d%lf", &(q_i[i]), &(q_j[i]), &(q_ij[i]));
  }

  for (i = 0; i < variables; i++) {
    x_end[i] = 0.0;
    fscanf(fp, "%lf", &(c[i]));
  }

  fscanf(fp, "%d", &a_nnz);

  a_i = (int *) malloc(sizeof(int)*a_nnz + 1);
  a_j = (int *) malloc(sizeof(int)*a_nnz + 1);
  a_ij = (double *) malloc(sizeof(double)*a_nnz + 1);
  b = (double *) malloc(sizeof(double)*constraints + 1);
  c_type = (int *) malloc(sizeof(int)*constraints + 1);

  for (i = 0; i < a_nnz; i++) {
    fscanf(fp, "%d%d%lf", &(a_i[i]), &(a_j[i]), &(a_ij[i]));
  }

  for (i = 0; i < constraints; i++) {
    pi_end[i] = 0.0;
    fscanf(fp, "%lf%d", &(b[i]), &(c_type[i]));
  }

  lb = (double *) malloc(sizeof(double)*variables + 1);
  ub = (double *) malloc(sizeof(double)*variables + 1);

  for (i = 0; i < variables; i++) {
    fscanf(fp, "%lf%lf", &(lb[i]), &(ub[i]));
  }
  (void) fclose(fp);

  simpleQP(variables, constraints, q_nnz, q_i, q_j, q_ij, c,
           a_nnz, a_i, a_j, a_ij, b, c_type, lb, ub,
           &termination, x_end, pi_end, &obj);

  switch (termination) {
  case QP_Solved:
    printf("QP solved.\n");
    printf("objective: %g\n", obj);
    for (i = 0; i < variables; i++) {
      printf("x[%3d] = %g\n", i+1, x_end[i]);
    }
    for (i = 0; i < constraints; i++) {
      printf("pi[%3d] = %g\n", i + 1, pi_end[i]);
    }
    break;
  case QP_Unbounded:
    printf("QP unbounded.\n");
    break;
  case QP_Infeasible:
    printf("QP infeasible.\n");
    break;
  case QP_Error:
    printf("*** Error solving QP.\n");
    break;
  default:
    printf ("Unexpected termination status: %d\n", termination);
  }

  free(x_end);
  free(pi_end);

  free(q_i);
  free(q_j);
  free(q_ij);
  free(c);

  free(a_i);
  free(a_j);
  free(a_ij);
  free(b);
  free(c_type);

  free(lb);
  free(ub);

  return 0;
} /* main */
