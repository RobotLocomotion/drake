#include <limits.h>
#include <stdio.h>

#include "Standalone_Path.h"

#include "Install.h"
#include "MCP_Interface.h"

#include "Path.h"
#include "PathOptions.h"

#include "Macros.h"
#include "Output.h"
#include "Options.h"

typedef struct
{
  int n;
  int nnz;

  double *z;
  double *f;

  double *lb;
  double *ub;
} Problem;

static Problem problem;

static CB_FUNC(void) problem_size(void *id, int *n, int *nnz)
{
  *n = problem.n;
  *nnz = problem.nnz;
  return;
}

static CB_FUNC(void) bounds(void *id, int n, double *z, double *lb, double *ub)
{
  int i;

  for (i = 0; i < n; i++) {
    z[i] = problem.z[i];
    lb[i] = problem.lb[i];
    ub[i] = problem.ub[i];
  }
  return;
}

static CB_FUNC(int) function_evaluation(void *id, int n, double *z, double *f)
{
  int err;

  err = FUNCEVAL(&n, z, f);
  return err;
}

static CB_FUNC(int) jacobian_evaluation(void *id, int n, double *z, int wantf,
                                        double *f, int *nnz,
                                        int *col_start, int *col_len,
                                        int *row, double *data)
{
  int i, err = 0;

  if (wantf) {
    err += function_evaluation(id, n, z, f);
  }

  err += JACEVAL(&n, nnz, z, col_start, col_len, row, data);

  (*nnz) = 0;
  for (i = 0; i < n; i++) {
    (*nnz) += col_len[i];
  }
  return err;
}

static MCP_Interface interfac =
{
  NULL,
  problem_size, bounds,
  function_evaluation, jacobian_evaluation,
  NULL, /* hessian evaluation */
  NULL, NULL,
  NULL, NULL,
  NULL
};

static void install_interface(MCP *m)
{
  MCP_SetInterface(m, &interfac);
  return;
}

void F_CALLCONV MYMAIN(int *n, int *nnz, int *status,
                       double *z, double *f, double *lb, double *ub)
{
  Options_Interface *o;
  MCP *m;
  MCP_Termination t;
  Information info;

  double *tempZ;
  double *tempF;
  double dnnz;
  int i;

  for_install_output();

  o = Options_Create();
  Path_AddOptions(o);
  Options_Default(o);

  Output_Printf(Output_Log | Output_Status | Output_Listing,
                "%s: Standalone-F Link\n", Path_Version());

  problem.n = *n;
  if (problem.n == 0) {
    Output_Printf(Output_Log | Output_Status,
                  "\n ** EXIT - solution found (degenerate model).\n");
    (*status) = MCP_Solved;
    Options_Destroy(o);
    return;
  }

  dnnz = MIN(1.0*(*nnz), 1.0*problem.n*problem.n);
  if (dnnz > INT_MAX) {
    Output_Printf(Output_Log | Output_Status,
                  "\n ** EXIT - model too large.\n");
    (*status) = MCP_Error;
    Options_Destroy(o);
    return;
  }
  problem.nnz = (int) dnnz;

  Output_Printf(Output_Log | Output_Status | Output_Listing,
                "%d row/cols, %d non-zeros, %3.2f%% dense.\n\n",
                problem.n, problem.nnz,
                100.0*problem.nnz/(1.0*problem.n*problem.n));
  problem.nnz++;

  problem.z = z;
  problem.f = f;
  problem.lb = lb;
  problem.ub = ub;

  m = MCP_Create(problem.n, problem.nnz);
  install_interface(m);

  Options_Read(o, "path.opt");
  Options_Display(o);

  info.generate_output = Output_Log | Output_Status | Output_Listing;
  info.use_start = True;
  info.use_basics = True;

  t = Path_Solve(m, &info);

  tempZ = MCP_GetX(m);
  tempF = MCP_GetF(m);

  for (i = 0; i < problem.n; i++) {
    z[i] = tempZ[i];
    f[i] = tempF[i];
  }
  *status = t;

  MCP_Destroy(m);
  Options_Destroy(o);
  return;
}

void F_CALLCONV SET_LOG(int *l)
{
  for_output_set_log(*l);
  return;
}
