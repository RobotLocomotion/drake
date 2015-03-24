/* Produced by CVXGEN, 2015-03-24 13:12:44 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"C1_0", "C1_1", "C1_2", "C1_3", "C2_0", "C2_1", "C2_2", "C2_3", "max_accel", "C1", "C2"};
  const int num_var_names = 11;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "r0");
  if (xm == NULL) {
    printf("could not find params.r0.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("r0 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter r0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter r0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter r0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.r0;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "r_switch");
  if (xm == NULL) {
    printf("could not find params.r_switch.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("r_switch must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter r_switch must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter r_switch must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter r_switch must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.r_switch;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "r_switch_slack");
  if (xm == NULL) {
    printf("could not find params.r_switch_slack.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("r_switch_slack must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter r_switch_slack must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter r_switch_slack must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter r_switch_slack must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.r_switch_slack;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "rd0");
  if (xm == NULL) {
    printf("could not find params.rd0.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("rd0 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter rd0 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter rd0 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter rd0 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.rd0;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "rd_switch");
  if (xm == NULL) {
    printf("could not find params.rd_switch.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("rd_switch must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter rd_switch must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter rd_switch must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter rd_switch must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.rd_switch;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "rd_switch_slack");
  if (xm == NULL) {
    printf("could not find params.rd_switch_slack.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("rd_switch_slack must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter rd_switch_slack must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter rd_switch_slack must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter rd_switch_slack must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.rd_switch_slack;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "rdf");
  if (xm == NULL) {
    printf("could not find params.rdf.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("rdf must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter rdf must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter rdf must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter rdf must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.rdf;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "rf");
  if (xm == NULL) {
    printf("could not find params.rf.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("rf must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter rf must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter rf must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter rf must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.rf;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "t_f");
  if (xm == NULL) {
    printf("could not find params.t_f.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("t_f must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter t_f must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter t_f must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter t_f must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.t_f;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "t_switch");
  if (xm == NULL) {
    printf("could not find params.t_switch.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("t_switch must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter t_switch must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter t_switch must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter t_switch must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.t_switch;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 10) {
    printf("Error: %d parameters are invalid.\n", 10 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 1; i++)
      printf("  params.t_switch[%d] = %.6g;\n", i, params.t_switch[i]);
    for (i = 0; i < 1; i++)
      printf("  params.t_f[%d] = %.6g;\n", i, params.t_f[i]);
    for (i = 0; i < 6; i++)
      printf("  params.r0[%d] = %.6g;\n", i, params.r0[i]);
    for (i = 0; i < 6; i++)
      printf("  params.rd0[%d] = %.6g;\n", i, params.rd0[i]);
    for (i = 0; i < 6; i++)
      printf("  params.rf[%d] = %.6g;\n", i, params.rf[i]);
    for (i = 0; i < 6; i++)
      printf("  params.rdf[%d] = %.6g;\n", i, params.rdf[i]);
    for (i = 0; i < 6; i++)
      printf("  params.r_switch[%d] = %.6g;\n", i, params.r_switch[i]);
    for (i = 0; i < 6; i++)
      printf("  params.r_switch_slack[%d] = %.6g;\n", i, params.r_switch_slack[i]);
    for (i = 0; i < 6; i++)
      printf("  params.rd_switch[%d] = %.6g;\n", i, params.rd_switch[i]);
    for (i = 0; i < 6; i++)
      printf("  params.rd_switch_slack[%d] = %.6g;\n", i, params.rd_switch_slack[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  /* Create cell arrays for indexed variables. */
  dims[0] = 3;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "C1", cell);
  dims[0] = 3;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "C2", cell);
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C1_0", xm);
  dest = mxGetPr(xm);
  src = vars.C1_0;
  for (i = 0; i < 6; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C1_1", xm);
  xm_cell = mxCreateDoubleMatrix(6, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "C1");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.C1_1;
  for (i = 0; i < 6; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C1_2", xm);
  xm_cell = mxCreateDoubleMatrix(6, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "C1");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.C1_2;
  for (i = 0; i < 6; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C1_3", xm);
  xm_cell = mxCreateDoubleMatrix(6, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "C1");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.C1_3;
  for (i = 0; i < 6; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C2_0", xm);
  dest = mxGetPr(xm);
  src = vars.C2_0;
  for (i = 0; i < 6; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C2_1", xm);
  xm_cell = mxCreateDoubleMatrix(6, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "C2");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.C2_1;
  for (i = 0; i < 6; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C2_2", xm);
  xm_cell = mxCreateDoubleMatrix(6, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "C2");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.C2_2;
  for (i = 0; i < 6; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "C2_3", xm);
  xm_cell = mxCreateDoubleMatrix(6, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "C2");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.C2_3;
  for (i = 0; i < 6; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[0], 0, "max_accel", xm);
  dest = mxGetPr(xm);
  src = vars.max_accel;
  for (i = 0; i < 1; i++) {
    *dest++ = *src++;
  }
}
