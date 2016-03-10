#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>

#include "sparseQP.h"

#include "MCP_Interface.h"

#include "Path.h"
#include "PathOptions.h"

#include "Output.h"
#include "Output_Interface.h"
#include "Options.h"

typedef struct MCPData {
  int nQP;                      /* variables in QP */
  int mQP;                      /* constraints in QP */

  int n;
  int nnz;

  double *z;
  double *lb;
  double *ub;
  double *q;

  int *colTmp;                  /* either counts for or ptr to cols of M */
  int *colPtr;                  /* to column starts for M */
  int *rowIdx;                  /* row indices for nonzeros in M */
  double *val;                  /* values of M */

} MCPData_t;

static MCPData_t mcpData;
static int filled;

static CB_FUNC(void)
start(void *v)
{
  filled = 0;
  return;
}

static CB_FUNC(void)
problem_size(void *v, int *n, int *nnz)
{
  *n = mcpData.n;
  *nnz = mcpData.nnz;
  return;
}

static CB_FUNC(void)
bounds(void *v, int n, double *z, double *lb, double *ub)
{
  int i;

  for (i = 0; i < n; i++) {
    z[i] = mcpData.z[i];
    lb[i] = mcpData.lb[i];
    ub[i] = mcpData.ub[i];
  }
  return;
}

static CB_FUNC(void)
variable_name(void *v, int j, char *buffer, int buffer_size)
{
  if (j < mcpData.nQP) {
    sprintf(buffer, "x[%5d]", j+1);
  }
  else {
    sprintf(buffer, "pi[%5d]", j - mcpData.nQP + 1);
  }
  return;
}

static CB_FUNC(int)
function_evaluation(void *v, int n, double *z, double *f)
{
  int i, k, kStop;

  for (i = 0;  i < n;  i++) {
    f[i] = mcpData.q[i];
  }

  for (i = 0;  i < n;  i++) {
    if (0 != z[i]) {
      for (kStop = mcpData.colPtr[i+1], k = mcpData.colPtr[i];  k < kStop;  k++)
        f[mcpData.rowIdx[k]] += mcpData.val[k]*z[i];
    }
  }
  return 0;
}

static CB_FUNC(int)
jacobian_evaluation(void *v, int n, double *z, int wantf,
                    double *f, int *nnz,
                    int *colStart, int *colLen,
                    int *rowIdx, double *data)
{
  int i, k;

  if (wantf) {
    function_evaluation(v, n, z, f);
  }

  if (!filled) {
    for (i = 0;  i < mcpData.n;  i++) {
      colStart[i] = mcpData.colPtr[i] + 1; /* colStart must be one-based */
      colLen[i] = (mcpData.colPtr[i+1] - mcpData.colPtr[i]);
    }

    for (k = 0;  k < mcpData.nnz;  k++) {
      rowIdx[k] = mcpData.rowIdx[k] + 1;
      data[k] = mcpData.val[k];
    }

    filled = 1;
  }

  *nnz = mcpData.nnz;
  return 0;
}

static CB_FUNC(void)
mcp_typ(void *d, int nnz, int *typ)
{
  int i;

  for (i = 0; i < nnz; i++) {
    typ[i] = PRESOLVE_LINEAR;
  }
  return;
}

static CB_FUNC(void)
con_typ(void *d, int n, int *typ)
{
  int i;

  for (i = 0;  i < mcpData.nQP;  i++) {
    typ[i] = PRESOLVE_DUAL;
  }

  for (  ;  i < mcpData.nQP + mcpData.mQP;  i++) {
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

static void
install_interface(MCP *m)
{
  MCP_SetInterface(m, &mcp_interface);
  MCP_SetPresolveInterface(m, &mcp_presolve);
  return;
}

static void
checkA (int m, int n, int nnz,
        const int *colPtr, const int *rowIdx, int *colCnt)
{
  int i, iPrev, j, k, k0, k1;

  assert(colPtr[0] == 0);
  assert(colPtr[n] == nnz);
  for (k0 = 0, j = 0;  j < n;  j++) {
    k1 = colPtr[j+1];
    assert(k0 <= k1);
    iPrev = -1;
    for (k = k0;  k < k1;  k++) {
      i = rowIdx[k];
      assert(i > iPrev);
      iPrev = i;
      colCnt[n + i]++;
    }
    colCnt[j] += (k1-k0);
    assert(iPrev < m);
    k0 = k1;
  }
  assert(k0 == nnz);
} /* checkA */

/* check content of L, and return # of diagaonal nonzeros */
static int
checkL (int n, int nnz, const int *Li, const int *Lj, int *colCnt)
{
  int i, iPrev, j, jPrev, k;
  int diagCnt;

  diagCnt = 0;
  jPrev = 0;
  iPrev = -1;
  for (k = 0;  k < nnz;  k++) {
    i = Li[k];
    j = Lj[k];
    assert((i >= 0) && (i < n));
    assert((j >= 0) && (j < n));
    assert(i >= j);
    colCnt[j]++;
    if (i==j)
      diagCnt++;
    else
      colCnt[i]++;
    assert(j >= jPrev);
    if (j == jPrev) {
      assert(i > iPrev);
    }
    else {
      jPrev = j;
    }
    iPrev = i;
  }
  return diagCnt;
} /* checkL */


static void
create (int n, int m,
        int Lnnz, const int *Li, const int *Lj, const double *Lval, const double *c,
        int Annz, const int *AcolPtr, const int *ArowIdx, const double *Aval, const double *b,
        const int *conType, const double *x, const double *pi,
        const double *lb, const double *ub)
{
  double inf;
  double dnnz;
  int Ldiags;                   /* diagonal elements in L */
  int i, j, k, kk, kL, kA, kAstop;

  inf = 1e20;
  (void) memset (&mcpData, 0, sizeof(MCPData_t));
  mcpData.nQP = n;
  mcpData.mQP = m;
  mcpData.n = n + m;
  mcpData.colTmp = (int *) malloc(sizeof(int)*(mcpData.n + 1));
  (void) memset (mcpData.colTmp, 0, sizeof(int)*(mcpData.n + 1));
  Ldiags = checkL (n, Lnnz, Li, Lj,
                   mcpData.colTmp);
  checkA (m, n, Annz, AcolPtr, ArowIdx,
          mcpData.colTmp);
  dnnz = 2.0 * Lnnz;
  dnnz -= Ldiags;
  dnnz += 2.0 * Annz;
  assert (dnnz < INT_MAX);
  mcpData.nnz = (int) dnnz;

  mcpData.colPtr = (int *) malloc(sizeof(int)*(mcpData.n+1));
  mcpData.rowIdx = (int *) malloc(sizeof(int)*mcpData.nnz+1);
  mcpData.val = (double *) malloc(sizeof(double)*mcpData.nnz+1);

  for (k = 0, i = 0;  i < mcpData.n;  i++) {
    mcpData.colPtr[i] = k;
    k += mcpData.colTmp[i];
  }
  mcpData.colPtr[i] = k;
  assert(k==mcpData.nnz);

  mcpData.z = (double *) malloc(sizeof(double)*mcpData.n);
  mcpData.q = (double *) malloc(sizeof(double)*mcpData.n);
  mcpData.lb = (double *) malloc(sizeof(double)*mcpData.n);
  mcpData.ub = (double *) malloc(sizeof(double)*mcpData.n);

  for (j = 0;  j < n;  j++) {
    mcpData.z[j] = x[j];
    mcpData.q[j] = c[j];
    mcpData.lb[j] = lb[j];
    mcpData.ub[j] = ub[j];
  }
  for (i = 0;  i < m;  i++) {
    mcpData.z[i + n] = pi[i];
    mcpData.q[i + n] = -b[i];
    switch(conType[i]) {
    case QP_LE:         /* <= constraint */
      mcpData.lb[i + n] = -inf;
      mcpData.ub[i + n] = 0;
      break;

    case QP_GE:         /* >= constraint */
      mcpData.lb[i + n] = 0;
      mcpData.ub[i + n] = inf;
      break;

    case QP_EQ:
      mcpData.lb[i + n] = -inf;
      mcpData.ub[i + n] = inf;
      break;
    }
  }

  (void) memcpy (mcpData.colTmp, mcpData.colPtr,sizeof(int)*mcpData.n);
  for (kL = 0, j = 0;  j < n;  j++) {
    /* walk through L until we get past col j */
    while ((kL < Lnnz) && (Lj[kL] <= j)) {
      k = mcpData.colTmp[j]++;
      mcpData.rowIdx[k] = i = Li[kL];
      mcpData.val[k] = Lval[kL];
      /* printf ("L[%d][%d] = %g:  k=%d\n", i, j, Lval[kL], k); */
      if (i != j) {
        k = mcpData.colTmp[i]++;
        mcpData.rowIdx[k] = Lj[kL];
        mcpData.val[k] = Lval[kL];
        /* printf ("L[%d][%d] = %g:  k=%d\n", j, i, Lval[kL], k); */
      }
      kL++;
    }

    /* walk j'th col of A, adding it to lower left and upper right of M */
    for (k = mcpData.colTmp[j], kA = AcolPtr[j], kAstop = AcolPtr[j+1];
         kA < kAstop;  kA++) {
      i = ArowIdx[kA];
      mcpData.rowIdx[k] = i + n;
      mcpData.val[k] = Aval[kA];
      /* printf ("A[%d][%d] = %g:  k=%d\n", i, j, Aval[kA], k); */
      k++;
      kk = mcpData.colTmp[n+i]++;
      mcpData.rowIdx[kk] = j;
      mcpData.val[kk] = -Aval[kA];
    }
    mcpData.colTmp[j] = k;
  }
  /*
  for (i = 0;  i < mcpData.n;  i++)
    printf ("tmp: %d  ptr: %d\n", mcpData.colTmp[i], mcpData.colPtr[i]);
  */
  for (i = 0;  i < mcpData.n;  i++)
    assert(mcpData.colTmp[i] == mcpData.colPtr[i+1]);

  return;
} /* create */

static void destroy(void)
{
  free(mcpData.z);
  free(mcpData.lb);
  free(mcpData.ub);
  free(mcpData.q);
  free(mcpData.colTmp);
  free(mcpData.colPtr);
  free(mcpData.rowIdx);
  free(mcpData.val);
  return;
}

void sparseQP (int n, int m,
               int Lnnz, const int *Li, const int *Lj, const double *Lval,
               const double *c,
               int Annz, const int *AcolPtr, const int *ArowIdx, const double *Aval,
               const double *b,
               const int *conType, const double *lb, const double *ub,
               QP_Termination *status, double *x, double *pi, double *obj)
{
  Options_Interface *o;
  MCP *mcp;
  MCP_Termination t;
  Information info;

  double *z;
  int i, j, k, kStop;

#if defined(TIMECHECK)
  Output_SetLog(stdout);
#endif

  o = Options_Create();
  Path_AddOptions(o);
  Options_Default(o);
#if defined(TIMECHECK)
  Options_SetDouble(o, "time_limit", 2.0);
  Options_Set(o, "crash_method none");
#endif

  Output_Printf(Output_Log | Output_Status | Output_Listing,
                "%s: QP Link\n", Path_Version());

  create(n, m, Lnnz, Li, Lj, Lval, c,
         Annz, AcolPtr, ArowIdx, Aval, b, conType, x, pi, lb, ub);

  if (mcpData.n == 0) {
    Output_Printf(Output_Log | Output_Status,
                  "\n ** EXIT - solution found (degenerate model).\n");
    (*status) = QP_Solved;
    Options_Destroy(o);
    return;
  }

  Output_Printf(Output_Log | Output_Status | Output_Listing,
                "%d row/cols, %d non-zeros, %3.2f%% dense.\n\n",
                mcpData.n, mcpData.nnz,
                100.0*mcpData.nnz/(1.0*mcpData.n*mcpData.n));

  mcp = MCP_Create(mcpData.n, mcpData.nnz+1);
  MCP_Jacobian_Structure_Constant(mcp, 1);
  install_interface(mcp);

  Options_Read(o, "OPT.opt");
  Options_Display(o);

  info.generate_output = Output_Log | Output_Status | Output_Listing;
  info.use_start = True;
  info.use_basics = True;

  t = Path_Solve(mcp, &info);

  *obj = 1e20;
  *status = QP_Other;
  if (t == MCP_Solved) {
    *status = QP_Solved;
  }
  else if ((MCP_Infeasible == t) || (MCP_NoProgress == t)) {
    Options_Default(o);

    Options_Read(o, "OPT.op2");
    Options_Display(o);

    for (j = 0;  j < n;  j++) {
      mcpData.q[j] = 0;
      for (kStop = mcpData.colPtr[j+1], k = mcpData.colPtr[j];  k < kStop;  k++) {
        if (mcpData.rowIdx[k] >= n)
          break;
        mcpData.val[k] = 0.0;
      }
    }

    info.generate_output = Output_Log | Output_Status | Output_Listing;
    info.use_start = True;
    info.use_basics = True;

    t = Path_Solve(mcp, &info);
    if (t == MCP_Solved) {
      *status = QP_Unbounded;
    }
    else {
      *status = QP_Infeasible;
    }
  }

  if (QP_Solved == *status) {
    z = MCP_GetX(mcp);
    for (j = 0;  j < n;  j++) {
      x[j] = z[j];
    }
    for (i = 0;  i < m;  i++) {
      pi[i] = z[i + n];
    }
    *obj = 0;
    for (j = 0;  j < n;  j++) {
      for (kStop = mcpData.colPtr[j+1], k = mcpData.colPtr[j];  k < kStop;  k++) {
        if (mcpData.rowIdx[k] < n) {
          *obj += mcpData.val[k] * z[j] * z[mcpData.rowIdx[k]];
        }
      }
    }
    *obj /= 2.0;
    for (j = 0;  j < n;  j++) {
      *obj += z[j] * mcpData.q[j];
    }
  }

  MCP_Destroy(mcp);
  destroy();

  Options_Destroy(o);
  return;
}
