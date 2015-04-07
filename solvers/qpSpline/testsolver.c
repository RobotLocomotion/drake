/* Produced by CVXGEN, 2015-04-06 22:08:42 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q0[0] = 1.5507979025745755;
  params.Q0[2] = 0;
  params.Q0[1] = 0;
  params.Q0[3] = 1.7081478226181048;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q1[0] = 1.2909047389129444;
  params.Q1[2] = 0;
  params.Q1[1] = 0;
  params.Q1[3] = 1.510827605197663;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q2[0] = 1.8929469543476547;
  params.Q2[2] = 0;
  params.Q2[1] = 0;
  params.Q2[3] = 1.896293088933438;
  params.x0[0] = -1.497658758144655;
  params.x0[1] = -1.171028487447253;
  params.x0[2] = -1.7941311867966805;
  params.x0[3] = -0.23676062539745413;
  params.x0[4] = -1.8804951564857322;
  params.x0[5] = -0.17266710242115568;
  params.xd0[0] = 0.596576190459043;
  params.xd0[1] = -0.8860508694080989;
  params.xd0[2] = 0.7050196079205251;
  params.xd0[3] = 0.3634512696654033;
  params.xd0[4] = -1.9040724704913385;
  params.xd0[5] = 0.23541635196352795;
  params.x1[0] = -0.9629902123701384;
  params.x1[1] = -0.3395952119597214;
  params.x1[2] = -0.865899672914725;
  params.x1[3] = 0.7725516732519853;
  params.x1[4] = -0.23818512931704205;
  params.x1[5] = -1.372529046100147;
  params.x2[0] = 0.17859607212737894;
  params.x2[1] = 1.1212590580454682;
  params.x2[2] = -0.774545870495281;
  params.x2[3] = -1.1121684642712744;
  params.x2[4] = -0.44811496977740495;
  params.x2[5] = 1.7455345994417217;
  params.t1[0] = 1.9039816898917352;
  params.t0[0] = 0.6895347036512547;
  params.t2[0] = 1.6113364341535923;
  params.tf[0] = 1.383003485172717;
  params.xf[0] = -0.48802383468444344;
  params.xf[1] = -1.631131964513103;
  params.xf[2] = 0.6136436100941447;
  params.xf[3] = 0.2313630495538037;
  params.xf[4] = -0.5537409477496875;
  params.xf[5] = -1.0997819806406723;
  params.xdf[0] = -0.3739203344950055;
  params.xdf[1] = -0.12423900520332376;
  params.xdf[2] = -0.923057686995755;
  params.xdf[3] = -0.8328289030982696;
  params.xdf[4] = -0.16925440270808823;
  params.xdf[5] = 1.442135651787706;
}
