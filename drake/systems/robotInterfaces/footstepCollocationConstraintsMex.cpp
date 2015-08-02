#include "mex.h"
#include <math.h>
#include <Eigen/Dense>

using namespace std;

void constraints(mxArray* c_out, mxArray* ceq_out, mxArray* dc_out, mxArray* dceq_out, const Eigen::MatrixXd& x, size_t nc, size_t nceq, size_t nv, size_t nsteps)
{
  Eigen::MatrixXd steps = x.block(0, 0, 6, nsteps);
  Eigen::MatrixXd rel_steps = x.block(6, 0, 6, nsteps);

  Eigen::MatrixXd c(nc, 1);
  c = Eigen::MatrixXd::Zero(nc, 1);
  Eigen::VectorXd ceq(nceq);
  ceq = Eigen::VectorXd::Zero(nceq);
  Eigen::MatrixXd dc(nv, nsteps);
  dc = Eigen::MatrixXd::Zero(nv, nc);
  Eigen::MatrixXd dceq(nv, nceq);
  dceq = Eigen::MatrixXd::Zero(nv, nceq);
  Eigen::Matrix2d R;
  Eigen::Vector2d dxy;
  Eigen::Vector2d proj;
  Eigen::Vector2d u;
  Eigen::Vector2d al;

  // ceq.segment(0,2) = steps.block(0,0,2,1) - rel_steps.block(0,0,2,1);
  // dceq.block(0,0,2,2) << 1, 0, 0, 1;
  // dceq.block(6,0,2,2) << -1, 0, 0, -1;

  int j;
  int x1_ndx;
  int dx_ndx;
  int x2_ndx;
  int con_ndx;
  int con_dndx;
  double dx, dy, si, co;

  for (j = 2; j <= nsteps; j++) {
    con_ndx = (j-1)*2;
    con_dndx = 2;
    si = sin(steps(5,j-2));
    co = cos(steps(5,j-2));
    R << co, -si, si, co;
    dxy = R * rel_steps.block(0,j-1,2,1);
    proj = steps.block(0,j-2,2,1) + dxy;
    ceq.segment(con_ndx, con_dndx) = steps.block(0,j-1,2,1) - proj;
    x1_ndx = (j-2)*12;
    dx_ndx = (j-1)*12+6;
    x2_ndx = (j-1)*12;
    dceq.block(x2_ndx,con_ndx,2,con_dndx) << 1, 0, 0, 1;
    dceq.block(x1_ndx,con_ndx,2,con_dndx) << -1, 0, 0, -1;
    dx = rel_steps(0,j-1);
    dy = rel_steps(1,j-1);
    dceq.block(x1_ndx+5,con_ndx,1,con_dndx) << dx*si + dy*co, -dx*co + dy*si;
    dceq.block(dx_ndx,con_ndx,1,con_dndx) << -co, -si;
    dceq.block(dx_ndx+1,con_ndx,1,con_dndx) << si, -co;
  }

  memcpy(mxGetPr(c_out), c.data(), sizeof(double)*c.rows()*c.cols());
  memcpy(mxGetPr(dc_out), dc.data(), sizeof(double)*dc.rows()*dc.cols());
  memcpy(mxGetPr(ceq_out), ceq.data(), sizeof(double)*ceq.rows()*ceq.cols());
  memcpy(mxGetPr(dceq_out), dceq.data(), sizeof(double)*dceq.rows()*dceq.cols());

  return;
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  // [c_mex, ceq_mex, dc_mex, dceq_mex] = stepCollocationConstraintsMex(x);

  size_t nv = mxGetM(prhs[0]);
  size_t nsteps = nv / 12;
  size_t nceq = 2 * nsteps;
  size_t nc = 0;

  Eigen::MatrixXd x = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[0]), 12, nsteps);

  /* Create matrices for the return arguments. */
  plhs[0] = mxCreateDoubleMatrix((mwSize)nc, (mwSize)1, mxREAL); // c
  plhs[1] = mxCreateDoubleMatrix((mwSize)nceq, (mwSize)1, mxREAL); // ceq
  plhs[2] = mxCreateDoubleMatrix((mwSize)nv, (mwSize)nc, mxREAL); // dc
  plhs[3] = mxCreateDoubleMatrix((mwSize)nv, (mwSize)nceq, mxREAL); // dceq

  /* Assign pointers to each output. */
  mxArray* c = plhs[0];
  mxArray* ceq = plhs[1];
  mxArray* dc = plhs[2];
  mxArray* dceq = plhs[3];

  constraints(c, ceq, dc, dceq, x, nc, nceq, nv, nsteps);
}