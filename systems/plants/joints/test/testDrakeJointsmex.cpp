#include "DrakeJoint.h"
#include "QuaternionFloatingJoint.h"
#include "RollPitchYawFloatingJoint.h"
#include "HelicalJoint.h"
#include "PrismaticJoint.h"
#include "RevoluteJoint.h"
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include "mex.h"
#include "matrix.h"
#include "../../../../util/test/testUtil.h" // TODO

using namespace Eigen;
using namespace std;

mxArray* safelyGetField(const mxArray* array, string field_name, int index = 0)
{
  mxArray* ret = mxGetField(array, index, field_name.c_str());
  if (!ret)
  {
    mexErrMsgTxt(("Field not found: " + field_name).c_str());
  }
  return ret;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 1) { // TODO: nlhs
    mexErrMsgIdAndTxt("Drake:testDrakeJointsmex:BadInputs","Usage struct_out = testGeometryGradientsmex(struct_in)");
  }

  mxArray* mxPrismatic = safelyGetField(prhs[0], "prismatic");
  Vector3d prismatic_joint_axis = matlabToEigen<3, 1>(safelyGetField(mxPrismatic, "joint_axis"));

  mxArray* mxRevolute = safelyGetField(prhs[0], "revolute");
  Vector3d revolute_joint_axis = matlabToEigen<3, 1>(safelyGetField(mxRevolute, "joint_axis"));

  mxArray* mxHelical = safelyGetField(prhs[0], "helical");
  Vector3d helical_joint_axis = matlabToEigen<3, 1>(safelyGetField(mxHelical, "joint_axis"));
  double helical_joint_pitch = *mxGetPr(safelyGetField(mxHelical, "pitch"));

  Isometry3d transform_to_parent = Isometry3d::Identity();

  vector<unique_ptr<DrakeJoint>> joints;

  joints.push_back(unique_ptr<DrakeJoint>(new PrismaticJoint("prismatic", transform_to_parent, prismatic_joint_axis)));
  joints.push_back(unique_ptr<DrakeJoint>(new RevoluteJoint("revolute", transform_to_parent, revolute_joint_axis)));
  joints.push_back(unique_ptr<DrakeJoint>(new HelicalJoint("helical", transform_to_parent, helical_joint_axis, helical_joint_pitch)));
  joints.push_back(unique_ptr<DrakeJoint>(new QuaternionFloatingJoint("quaternionFloating", transform_to_parent)));
  joints.push_back(unique_ptr<DrakeJoint>(new RollPitchYawFloatingJoint("rollPitchYawFloating", transform_to_parent)));

  for (const auto& joint : joints) {
    mexWarnMsgTxt(joint->getName().c_str());
  }

//  PrismaticJoint* prismaticJoint = new PrismaticJoint("prismatic", transform_to_parent, prismatic_joint_axis);
//  mexErrMsgTxt("bla");
//  delete prismaticJoint;

//  unique_ptr < DrakeJoint > prismaticJoint(prismaticJoint);
//  joints.push_back(prismaticJoint);
//  joints.push_back(unique_ptr<DrakeJoint>(new RevoluteJoint("revolute", body, transform_to_parent, revolute_joint_axis)));
//  joints.push_back(unique_ptr<DrakeJoint>(new HelicalJoint("helical", body, transform_to_parent, helical_joint_axis, helical_joint_pitch)));
//  joints.push_back(unique_ptr<DrakeJoint>(new QuaternionFloatingJoint("quaternionFloating", body, transform_to_parent)));
//  joints.push_back(unique_ptr<DrakeJoint>(new RollPitchYawFloatingJoint("rollPitchYawFloating", body, transform_to_parent)));
//
//  for (const auto& joint : joints) {
//    mexErrMsgTxt(joint->getName().c_str());
//  }


//  if (nrhs != 2 || nlhs != 11) {
//    mexErrMsgIdAndTxt("Drake:testGeometryGradientsmex:BadInputs","Usage [omega2qd, domega2qd, omega2rpyd, domega2rpyd, ddomega2rpyd, rpyd2omega, qd2omega, dqd2omega, dq2R, drpydR, dqdR] = testGeometryGradientsmex(q, dq)");
//  }
//
//  int argnum = 0;
//  Isometry3d T;
//  auto q = matlabToEigen<QUAT_SIZE, 1>(prhs[argnum++]);
//  auto dq = matlabToEigen<QUAT_SIZE>(prhs[argnum++]);
//
//  auto rpy = quat2rpy(q);
//
//  Matrix<double, QUAT_SIZE, SPACE_DIMENSION> omega2qd;
//  typename Gradient<Matrix<double, QUAT_SIZE, SPACE_DIMENSION>, QUAT_SIZE, 1>::type domega2qd;
//  Matrix<double, RPY_SIZE, SPACE_DIMENSION> omega2rpyd;
//  typename Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 1>::type domega2rpyd;
//  typename Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 2>::type ddomega2rpyd;
//  Matrix<double, SPACE_DIMENSION, QUAT_SIZE> qd2omega;
//  typename Gradient<Matrix<double, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type dqd2omega;
//
//  angularvel2quatdotMatrix(q, omega2qd, &domega2qd);
//  angularvel2rpydotMatrix(rpy, omega2rpyd, &domega2rpyd, &ddomega2rpyd);
//  auto rpyd2omega = rpydot2angularvelMatrix(rpy);
//  quatdot2angularvelMatrix(q, qd2omega, &dqd2omega);
//  auto R = quat2rotmat(q);
//  auto dq2R = dquat2rotmat(q);
//  Matrix<double, RotmatSize, Dynamic> dR = dq2R * dq;
//  auto drpydR = drotmat2rpy(R, dR);
//  auto dqdR = drotmat2quat(R, dR);
//
//  int outnum = 0;
//  plhs[outnum++] = eigenToMatlab(omega2qd);
//  plhs[outnum++] = eigenToMatlab(domega2qd);
//  plhs[outnum++] = eigenToMatlab(omega2rpyd);
//  plhs[outnum++] = eigenToMatlab(domega2rpyd);
//  plhs[outnum++] = eigenToMatlab(ddomega2rpyd);
//  plhs[outnum++] = eigenToMatlab(rpyd2omega);
//  plhs[outnum++] = eigenToMatlab(qd2omega);
//  plhs[outnum++] = eigenToMatlab(dqd2omega);
//  plhs[outnum++] = eigenToMatlab(dq2R);
//  plhs[outnum++] = eigenToMatlab(drpydR);
//  plhs[outnum++] = eigenToMatlab(dqdR);
}
