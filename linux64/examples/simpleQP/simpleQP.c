#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "simpleQP.h"

#include "MCP_Interface.h"

#include "Path.h"
#include "PathOptions.h"

#include "Macros.h"
#include "Output.h"
#include "Options.h"

typedef struct {
  int variables;
  int constraints;

  int n;
  int nnz;

  double *z;
  double *lb;
  double *ub;

  int *m_start;
  int *m_len;
  int *m_row;
  double *m_data;

  double *q;
} Problem;

static Problem problem;
static int filled;

static CB_FUNC(void) start(void *v)
{
  filled = 0;
  return;
}

static CB_FUNC(void) problem_size(void *v, int *n, int *nnz)
{
  *n = problem.n;
  *nnz = problem.nnz + 1;
  return;
}

static CB_FUNC(void) bounds(void *v, int n, double *z, double *lb, double *ub)
{
  int i;

  for (i = 0; i < n; i++) {
    z[i] = problem.z[i];
    lb[i] = problem.lb[i];
    ub[i] = problem.ub[i];
  }
  return;
}

static CB_FUNC(void) variable_name(void *v, int variable,
                                   char *buffer, int buffer_size)
{
  if (variable <= problem.variables) {
    sprintf(buffer, "x[%5d]", variable);
  }
  else {
    sprintf(buffer, "pi[%5d]", variable - problem.variables);
  }
  return;
}

static CB_FUNC(int) function_evaluation(void *v, int n, double *z, double *f)
{
  int col, colStart, colEnd, row;
  double value;

  for (col = 0; col < n; col++) {
    f[col] = problem.q[col];
  }

  for (col = 0; col < n; col++) {
    value = z[col];

    if (value != 0) {
      colStart = problem.m_start[col] - 1;
      colEnd = colStart + problem.m_len[col];

      while(colStart < colEnd) {
        row = problem.m_row[colStart] - 1;
        f[row] += problem.m_data[colStart]*value;
        colStart++;
      }
    }
  }

  return 0;
}

static CB_FUNC(int) jacobian_evaluation(void *v, int n, double *z, int wantf,
                                        double *f, int *nnz,
                                        int *col_start, int *col_len,
                                        int *row, double *data)
{
  int element;

  if (wantf) {
    function_evaluation(v, n, z, f);
  }

  if (!filled) {
    for (element = 0; element < problem.n; element++) {
      col_start[element] = problem.m_start[element];
      col_len[element] = problem.m_len[element];
    }

    for (element = 0; element < problem.nnz; element++) {
      row[element] = problem.m_row[element];
      data[element] = problem.m_data[element];
    }

    filled = 1;
  }

  *nnz = problem.nnz;
  return 0;
}

static CB_FUNC(void) mcp_typ(void *d, int nnz, int *typ)
{
  int i;

  for (i = 0; i < nnz; i++) {
    typ[i] = PRESOLVE_LINEAR;
  }
  return;
}

static CB_FUNC(void) con_typ(void *d, int n, int *typ)
{
  int i;

  for (i = 0; i < problem.variables; i++) {
    typ[i] = PRESOLVE_DUAL;
  }

  for ( ; i < problem.variables + problem.constraints; i++) {
    typ[i] = PRESOLVE_PRIMAL;
  }
  return;
}

static MCP_Interface mcp_interface =
{
  NULL,
  problem_size, bounds,
  function_evaluation, jacobian_evaluation,
  NULL, /* hessian evaluation */
  start, NULL,
  variable_name, NULL,
  NULL
};

static Presolve_Interface mcp_presolve =
{
  NULL,
  NULL, NULL,
  NULL, NULL,
  mcp_typ,
  con_typ
};

static void install_interface(MCP *m)
{
  MCP_SetInterface(m, &mcp_interface);
  MCP_SetPresolveInterface(m, &mcp_presolve);
  return;
}

static void sort(int rows, int cols, int elements,
                 int *row, int *col, double *data)
{
  double *m_data;
  int *m_start;
  int *m_len;
  int *m_row;

  int i, cs, ce;

  m_start = (int *) malloc(sizeof(int)*(cols + 1));
  m_len = (int *) malloc(sizeof(int)*(cols + 1));
  m_row = (int *) malloc(sizeof(int)*(elements + 1));
  m_data = (double *) malloc(sizeof(double)*(elements + 1));

  for(i = 0; i < cols; i++) {
    m_len[i] = 0;
  }

  for(i = 0; i < elements; i++) {
    assert(!((col[i] < 1) || (col[i] > cols)));

    assert(!((row[i] < 1) || (row[i] > rows)));

    m_len[col[i] - 1]++;
  }

  m_start[0] = 0;
  for (i = 1; i < cols; i++) {
    m_start[i] = m_start[i - 1] + m_len[i - 1];
    m_len[i - 1] = 0;
  }
  m_len[i - 1] = 0;

  for(i = 0; i < elements; i++) {
    cs = col[i] - 1;
    ce = m_start[cs] + m_len[cs];
    m_row[ce] = row[i];
    m_data[ce] = data[i];
    m_len[cs]++;
  }

  elements = 0;
  for (i = 0; i < cols; i++) {
    cs = m_start[i];
    ce = cs + m_len[i];

    while (cs < ce) {
      row[elements] = m_row[cs];
      col[elements] = i + 1;
      data[elements] = m_data[cs];
      elements++;
      cs++;
    }
  }

  free(m_data);
  free(m_row);
  free(m_len);
  free(m_start);
  return;
}

static void create(int variables, int constraints,
                   int q_nnz, int *q_i, int *q_j, double *q_ij, double *c,
                   int a_nnz, int *a_i, int *a_j, double *a_ij, double *b,
                   int *c_type, double *z, double *pi,
                   double *lb, double *ub)
{
  double inf;
  int q_index;
  int a_index;
  int m_count;
  int i;

  inf = 1e20;

  problem.n = variables + constraints;
  problem.nnz = q_nnz + 2*a_nnz;

  problem.z = (double *) malloc(sizeof(double)*problem.n);
  problem.lb = (double *) malloc(sizeof(double)*problem.n);
  problem.ub = (double *) malloc(sizeof(double)*problem.n);

  problem.m_start = (int *) malloc(sizeof(int)*problem.n);
  problem.m_len = (int *) malloc(sizeof(int)*problem.n);
  problem.m_row = (int *) malloc(sizeof(int)*problem.nnz+1);
  problem.m_data = (double *) malloc(sizeof(double)*problem.nnz+1);

  problem.q = (double *) malloc(sizeof(double)*problem.n);

  sort(variables, variables, q_nnz, q_i, q_j, q_ij);
  sort(constraints, variables, a_nnz, a_i, a_j, a_ij);

  for (i = 0; i < variables; i++) {
    problem.z[i] = z[i];

    problem.q[i] = c[i];
    problem.lb[i] = lb[i];
    problem.ub[i] = ub[i];
  }

  for (i = 0; i < constraints; i++) {
    problem.z[i + variables] = pi[i];
    problem.q[i + variables] = -b[i];
    switch(c_type[i]) {
    case QP_LE:         /* <= constraint */
      problem.lb[i + variables] = -inf;
      problem.ub[i + variables] = 0;
      break;

    case QP_GE:         /* >= constraint */
      problem.lb[i + variables] = 0;
      problem.ub[i + variables] = inf;
      break;

    case QP_EQ:
      problem.lb[i + variables] = -inf;
      problem.ub[i + variables] = inf;
      break;
    }
  }

  q_index = 0;
  a_index = 0;
  m_count = 0;
  for (i = 0; i < variables; i++) {
    problem.m_start[i] = m_count + 1;
    problem.m_len[i] = 0;

    while ((q_index < q_nnz) && (q_j[q_index] <= i + 1)) {
      if (q_ij[q_index] != 0) {
        problem.m_len[i]++;
        problem.m_row[m_count] = q_i[q_index];
        problem.m_data[m_count] = q_ij[q_index];
        m_count++;
      }
      q_index++;
    }

    while ((a_index < a_nnz) && (a_j[a_index] <= i + 1)) {
      if (a_ij[a_index] != 0) {
        problem.m_len[i]++;
        problem.m_row[m_count] = a_i[a_index] + variables;
        problem.m_data[m_count] = a_ij[a_index];
        m_count++;
      }
      a_index++;
    }
  }

  sort(variables, constraints, a_nnz, a_j, a_i, a_ij);
  a_index = 0;

  for (i = 0; i < constraints; i++) {
    problem.m_start[i + variables] = m_count + 1;
    problem.m_len[i + variables] = 0;

    while ((a_index < a_nnz) && (a_i[a_index] <= i + 1)) {
      if (a_ij[a_index] != 0) {
        problem.m_len[i + variables]++;
        problem.m_row[m_count] = a_j[a_index];
        problem.m_data[m_count] = -a_ij[a_index];
        m_count++;
      }
      a_index++;
    }
  }

  problem.nnz = m_count;
  return;
}

static void destroy(void)
{
  free(problem.z);
  free(problem.lb);
  free(problem.ub);
  free(problem.m_start);
  free(problem.m_len);
  free(problem.m_row);
  free(problem.m_data);
  free(problem.q);
  return;
}

void simpleQP(int variables, int constraints,
              int q_nnz, int *q_i, int *q_j, double *q_ij, double *c,
              int a_nnz, int *a_i, int *a_j, double *a_ij, double *b,
              int *c_type, double *lb, double *ub,
              QP_Termination *status, double *z, double *pi, double *obj)
{
  Options_Interface *o;
  MCP *m;
  MCP_Termination t;
  Information info;

  double *x;
  double dnnz;
  int element, i;
  int colStart, colEnd;

  o = Options_Create();
  Path_AddOptions(o);
  Options_Default(o);

  Output_Printf(Output_Log | Output_Status | Output_Listing,
                "%s: QP Link\n", Path_Version());

  create(variables, constraints, q_nnz, q_i, q_j, q_ij, c,
         a_nnz, a_i, a_j, a_ij, b, c_type, z, pi, lb, ub);

  if (problem.n == 0) {
    Output_Printf(Output_Log | Output_Status,
                  "\n ** EXIT - solution found (degenerate model).\n");
    (*status) = QP_Solved;
    Options_Destroy(o);
    return;
  }

  dnnz = MIN(1.0*problem.nnz, 1.0*problem.n*problem.n);
  if (dnnz > INT_MAX) {
    Output_Printf(Output_Log | Output_Status,
                  "\n ** EXIT - model too large.\n");
    (*status) = QP_Error;
    Options_Destroy(o);
    return;
  }
  problem.nnz = (int) dnnz;

  Output_Printf(Output_Log | Output_Status | Output_Listing,
                "%d row/cols, %d non-zeros, %3.2f%% dense.\n\n",
                problem.n, problem.nnz,
                100.0*problem.nnz/(1.0*problem.n*problem.n));

  m = MCP_Create(problem.n, problem.nnz+1);
  MCP_Jacobian_Structure_Constant(m, 1);
  install_interface(m);

  Options_Read(o, "OPT.opt");
  Options_Display(o);

  info.generate_output = Output_Log | Output_Status | Output_Listing;
  info.use_start = True;
  info.use_basics = True;

  t = Path_Solve(m, &info);

  if (t == MCP_Solved) {
    *status = QP_Solved;
  }
  else {
    Options_Default(o);

    Options_Read(o, "OPT.op2");
    Options_Display(o);

    for (i = 0; i < variables; i++) {
      problem.q[i] = 0;

      colStart = problem.m_start[i] - 1;
      colEnd = colStart + problem.m_len[i];

      while(colStart < colEnd) {
        if (problem.m_row[colStart] <= variables) {
          problem.m_data[colStart] = 0.0;
        }
        colStart++;
      }
    }

    info.generate_output = Output_Log | Output_Status | Output_Listing;
    info.use_start = True;
    info.use_basics = True;

    t = Path_Solve(m, &info);
    if (t == MCP_Solved) {
      *status = QP_Unbounded;
    }
    else {
      *status = QP_Infeasible;
    }
  }

  x = MCP_GetX(m);

  for (i = 0; i < variables; i++) {
    z[i] = x[i];
  }

  for (i = 0; i < constraints; i++) {
    pi[i] = x[i + variables];
  }

  *obj = 0;
  for (i = 0; i < variables; i++) {
    for (element = problem.m_start[i] - 1;
         element < problem.m_start[i] + problem.m_len[i] - 1;
         element++) {
      if (problem.m_row[element] <= variables) {
        *obj += problem.m_data[element] *
          z[i] * z[problem.m_row[element] - 1];
      }
    }
  }

  *obj /= 2.0;
  for (i = 0; i < variables; i++) {
    *obj += z[i] * problem.q[i];
  }

  MCP_Destroy(m);
  destroy();

  Options_Destroy(o);
  return;
}
