#include <mex.h>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  string usage = "Usage [C, dC] = inverseDynamicsmex(model_ptr, cache_ptr, f_ext, df_ext)";
  if (nrhs < 2 || nrhs > 6) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfOutputs", usage.c_str());
  }

  int gradient_order = nlhs - 1;


  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  int nq = model->num_positions;
  int nv = model->num_velocities;
  const mxArray* f_ext_matlab = nullptr;
  const mxArray* df_ext_matlab = nullptr;
  if (nrhs > 2)
    f_ext_matlab = prhs[arg_num++];
  if (nrhs > 3)
    df_ext_matlab = prhs[arg_num++];

  typedef double Scalar;
  std::unordered_map<RigidBody const *, GradientVar<Scalar, TWIST_SIZE, 1> > f_ext;
  if (f_ext_matlab != nullptr) {
    if (!mxIsEmpty(f_ext_matlab)) {
      mwSize rows = mxGetM(f_ext_matlab);
      mwSize cols = mxGetN(f_ext_matlab);

      if (rows != TWIST_SIZE)
        throw runtime_error("f_ext matrix has number of rows not equal to 6");
      if (cols != model->bodies.size())
        throw runtime_error("f_ext matrix has number of columns not equal to number of rigid bodies in manipulator");


      GradientVar<double, TWIST_SIZE, Dynamic> f_ext_matrix(TWIST_SIZE, model->bodies.size(), nq + nv, gradient_order);
      f_ext_matrix.value() = matlabToEigen<TWIST_SIZE, Dynamic>(f_ext_matlab);
      if (gradient_order > 0) {
        if (df_ext_matlab == nullptr) {
          throw runtime_error("df_ext must be passed in if you pass in f_ext and want gradient output");
        }
        f_ext_matrix.gradient().value() = matlabToEigen<Dynamic, Dynamic>(df_ext_matlab);
      }

      for (int i = 0; i < model->bodies.size(); i++) {
        const shared_ptr<RigidBody>& body = model->bodies[i];
        GradientVar<double, TWIST_SIZE, 1> f_ext_i(TWIST_SIZE, 1, nq + nv, gradient_order);
        f_ext_i.value() = f_ext_matrix.value().col(i);
        if (f_ext_matrix.hasGradient()) {
          f_ext_i.gradient().value() = f_ext_matrix.gradient().value().middleRows<TWIST_SIZE>(TWIST_SIZE * i);
        }
        f_ext.insert({body.get(), f_ext_i});
      }
    }
  }
  else
    throw runtime_error("data type of f_ext not recognized");

  auto ret = model->dynamicsBiasTerm(*cache, f_ext, gradient_order);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}
