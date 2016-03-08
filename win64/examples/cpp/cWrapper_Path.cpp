#include <climits>
#include <cstdio>

/* cWrapper is a C-style wrapper used as an example of calling Path from C++ */

#include "cWrapper_Path.hpp"

#include "MCP_Interface.h"

#include "Path.h"
#include "PathOptions.h"

#include "Macros.h"
#include "Output_Interface.h"
#include "Options.h"


typedef struct Problem
{
  int n;
  int nnz;

  double *z;
  double *f;

  double *lb;
  double *ub;
} Problem_t;

static Problem_t problem;

extern "C" {

static CB_FUNC(void) problem_size(void *id, int *n, int *nnz)
{
  *n = problem.n;
  *nnz = problem.nnz;
  return;
} // problem_size

static CB_FUNC(void) bounds(void *id, int n, double *z, double *lb, double *ub)
{
  int i;

  for (i = 0; i < n; i++) {
    z[i] = problem.z[i];
    lb[i] = problem.lb[i];
    ub[i] = problem.ub[i];
  }
  return;
} // bounds

static CB_FUNC(int) function_evaluation(void *id, int n, double *z, double *f)
{
  int err;

  err = funcEval(n, z, f);
  return err;
} // function_evaluation

static CB_FUNC(int) jacobian_evaluation(void *id, int n, double *z, int wantf, 
                                        double *f, int *nnz,
                                        int *col_start, int *col_len, 
                                        int *row, double *data)
{
  int i, err = 0;

  if (wantf) {
    err += function_evaluation(id, n, z, f);
  }

  err += jacEval(n, *nnz, z, col_start, col_len, row, data);

  (*nnz) = 0;
  for (i = 0; i < n; i++) {
    (*nnz) += col_len[i];
  }
  return err;
} // jacobian_evaluation

} // end extern "C"

static MCP_Interface mcp_interface =
{
  NULL,
  problem_size, bounds,
  function_evaluation, jacobian_evaluation,
  NULL, /* hessian evaluation */
  NULL, NULL,
  NULL, NULL,
  NULL
};

extern "C" {

/* callback to register with PATH: output from PATH will go here */
static CB_FUNC(void) messageCB (void *data, int mode, char *buf)
{
  fprintf (stdout, "%s", buf);
} /* messageCB */

} // end extern "C"


static Output_Interface outputInterface =
{
  NULL,
  messageCB,
  NULL
};

void pathMain(int n, int nnz, int *status,
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

#if defined(USE_OUTPUT_INTERFACE)
  Output_SetInterface(&outputInterface);
#else
  /* N.B.: the Output_SetLog call does not work when using a DLL:
   * the IO systems of a .exe and .dll do not automatically interoperate */
  Output_SetLog(stdout);
#endif

  o = Options_Create();
  Path_AddOptions(o);
  Options_Default(o);

  fprintf(stdout,"%s: Standalone C++ link using C-style interface\n", Path_Version());

  if (n == 0) {
    fprintf(stdout, "\n ** EXIT - solution found (degenerate model).\n");
    (*status) = MCP_Solved;
    Options_Destroy(o);
    return;
  }

  dnnz = MIN(1.0*nnz, 1.0*n*n);
  if (dnnz > INT_MAX) {
    fprintf(stdout, "\n ** EXIT - model too large.\n");
    (*status) = MCP_Error;
    Options_Destroy(o);
    return;
  }
  nnz = (int) dnnz;
  
  fprintf(stdout, "%d row/cols, %d non-zeros, %3.2f%% dense.\n\n",
          n, nnz, 100.0*nnz/(1.0*n*n));
  nnz++;

  problem.n = n;
  problem.nnz = nnz;
  problem.z = z;
  problem.f = f;
  problem.lb = lb;
  problem.ub = ub;

  m = MCP_Create(n, nnz);
  MCP_SetInterface(m, &mcp_interface);

  Options_Read(o, "path.opt");
  Options_Display(o);

  info.generate_output = Output_Log | Output_Status | Output_Listing;
  info.use_start = True;
  info.use_basics = True;

  t = Path_Solve(m, &info);

  tempZ = MCP_GetX(m);
  tempF = MCP_GetF(m);

  for (i = 0; i < n; i++) {
    z[i] = tempZ[i];
    f[i] = tempF[i];
  }
  *status = t;

  MCP_Destroy(m);
  Options_Destroy(o);
  return;
}
