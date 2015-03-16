#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD> 
inline void manipulatorDynamics(RigidBodyManipulator *model, MatrixBase<DerivedA> const & q, MatrixBase<DerivedA> const & v, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> &B)
{
  const int nq = model->num_positions;
  H.resize(nq,nq); 
  C = VectorXd::Zero(nq);
  B = model->B;
  model->HandC(q, v, (MatrixXd*)nullptr, H, C, (MatrixXd*)nullptr, (MatrixXd*)nullptr, (MatrixXd*)nullptr);
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) { 
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  const int nq = model->num_positions;
  const int nv = model->num_velocities;

  Map<VectorXd> q(mxGetPr(prhs[1]),nq);
  Map<VectorXd> v(mxGetPr(prhs[2]),nv);
  
  MatrixXd H, B;
  VectorXd C;

  manipulatorDynamics(model, q, v, H, C, B);
  auto pos_constraints = model->positionConstraints<double>(1);
  
  // cout << "phiP" << endl;
  // cout << pos_constraints.value() << endl;
  // cout <<  "JP" << endl;
  // cout << pos_constraints.gradient().value() << endl;
  
  
}