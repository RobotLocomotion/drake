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

  string usage = "Usage [C, dC] = inverseDynamicsmex(model_ptr, cache_ptr, f_ext, vd, df_ext, dvd)";
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
  const mxArray* vd_matlab = nullptr;
  const mxArray* df_ext_matlab = nullptr;
  const mxArray* dvd_matlab = nullptr;
  if (nrhs > 2)
    f_ext_matlab = prhs[arg_num++];
  if (nrhs > 3)
    vd_matlab = prhs[arg_num++];
  if (nrhs > 4)
    df_ext_matlab = prhs[arg_num++];
  if (nrhs > 5)
    dvd_matlab = prhs[arg_num++];

  map<int, unique_ptr<GradientVar<double, TWIST_SIZE, 1> > > f_ext;
  if (f_ext_matlab != nullptr) {
    if (!mxIsEmpty(f_ext_matlab)) {
      if (mxIsCell(f_ext_matlab)) {
        mwSize rows = mxGetM(f_ext_matlab);
        mwSize cols = mxGetN(f_ext_matlab);
        if (rows != 1)
          throw runtime_error("f_ext cell array has number of rows not equal to 1");
        if (cols != model->bodies.size())
          throw runtime_error("f_ext cell array has number of columns not equal to number of rigid bodies in manipulator");

        for (int i = 0; i < model->bodies.size(); i++) {
          mxArray* f_ext_cell = mxGetCell(f_ext_matlab, i);
          if (!mxIsEmpty(f_ext_cell)) {
            f_ext[i] = unique_ptr< GradientVar<double, TWIST_SIZE, 1> >(new GradientVar<double, TWIST_SIZE, 1>(TWIST_SIZE, 1, nq + nv, gradient_order));
            // TODO: could save a copy with a two-argument matlabToEigen.
            f_ext[i]->value() = matlabToEigen<TWIST_SIZE, Dynamic>(f_ext_cell);
            if (gradient_order > 0 && df_ext_matlab != nullptr) {
              mxArray* df_ext_cell = mxGetCell(df_ext_matlab, i);
              f_ext[i]->gradient().value() = matlabToEigen<Dynamic, Dynamic>(df_ext_cell);
            }
          }
        }
      }
      else if (mxIsNumeric(f_ext_matlab)) {
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
          f_ext[i] = unique_ptr< GradientVar<double, TWIST_SIZE, 1> >(new GradientVar<double, TWIST_SIZE, 1>(TWIST_SIZE, 1, nq + nv, gradient_order));
          f_ext[i]->value() = f_ext_matrix.value().col(i);
          if (f_ext_matrix.hasGradient()) {
            f_ext[i]->gradient().value() = f_ext_matrix.gradient().value().middleRows<TWIST_SIZE>(TWIST_SIZE * i);
          }
        }
      }
    }
  }
  else
    throw runtime_error("data type of f_ext not recognized");

  unique_ptr<GradientVar<double, Eigen::Dynamic, 1> > vd_ptr;
  if (vd_matlab != nullptr) {
    if (!mxIsEmpty(vd_matlab)) {
      int nv = model->num_velocities;
      vd_ptr = unique_ptr<GradientVar<double, Eigen::Dynamic, 1>>(new GradientVar<double, Eigen::Dynamic, 1>(nv, 1, nq + nv, gradient_order));
      vd_ptr->value() = matlabToEigen<Dynamic, 1>(vd_matlab);
      if (gradient_order > 0) {
        if (dvd_matlab == nullptr) {
          throw runtime_error("dvd must be passed in if you pass in vd and want gradient output");
        }
        vd_ptr->gradient().value() = matlabToEigen<Dynamic, Dynamic>(dvd_matlab);
      }
    }
  }

  auto ret = model->inverseDynamics(*cache, f_ext, vd_ptr.get(), gradient_order);

  plhs[0] = eigenToMatlab(ret.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(ret.gradient().value());
}
