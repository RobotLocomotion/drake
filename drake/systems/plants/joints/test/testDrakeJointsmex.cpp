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
#include "drakeMexUtil.h"

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

void safelySetField(mxArray* array, const string& fieldname, mxArray* data, int index = 0)
{
  int fieldnum;
  fieldnum = mxGetFieldNumber(array, fieldname.c_str());
  if (fieldnum < 0) {
    fieldnum = mxAddField(array, fieldname.c_str());
  }
  mxSetFieldByNumber(array, index, fieldnum, data);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if (nrhs != 1 || nlhs != 1) {
    mexErrMsgIdAndTxt("Drake:testDrakeJointsmex:BadInputs","Usage struct_out = testGeometryGradientsmex(struct_in)");
  }
  string prismaticJointName = "prismatic";
  mxArray* mxPrismatic = safelyGetField(prhs[0], prismaticJointName);
  Vector3d prismatic_joint_axis = matlabToEigen<3, 1>(safelyGetField(mxPrismatic, "joint_axis"));

  string revoluteJointName = "revolute";
  mxArray* mxRevolute = safelyGetField(prhs[0], revoluteJointName);
  Vector3d revolute_joint_axis = matlabToEigen<3, 1>(safelyGetField(mxRevolute, "joint_axis"));

  string helicalJointName = "helical";
  mxArray* mxHelical = safelyGetField(prhs[0], helicalJointName);
  Vector3d helical_joint_axis = matlabToEigen<3, 1>(safelyGetField(mxHelical, "joint_axis"));
  double helical_joint_pitch = *mxGetPr(safelyGetField(mxHelical, "pitch"));

  Isometry3d transform_to_parent = Isometry3d::Identity();

  vector<unique_ptr<DrakeJoint>> joints;
//  joints.push_back(unique_ptr<DrakeJoint>(new PrismaticJoint(prismaticJointName, transform_to_parent, prismatic_joint_axis)));
//  joints.push_back(unique_ptr<DrakeJoint>(new RevoluteJoint(revoluteJointName, transform_to_parent, revolute_joint_axis)));
//  joints.push_back(unique_ptr<DrakeJoint>(new HelicalJoint(helicalJointName, transform_to_parent, helical_joint_axis, helical_joint_pitch)));
  joints.push_back(unique_ptr<DrakeJoint>(new QuaternionFloatingJoint("quaternion_floating", transform_to_parent)));
  joints.push_back(unique_ptr<DrakeJoint>(new RollPitchYawFloatingJoint("rpy_floating", transform_to_parent)));

  const mwSize ndim = 1;
  mwSize dims[] = {1};
  plhs[0] = mxCreateStructArray(ndim, dims, 0, nullptr);
  for (vector<unique_ptr<DrakeJoint>>::const_iterator it = joints.begin(); it != joints.end(); it++) {
    const unique_ptr<DrakeJoint>& joint = (*it);
    mxArray* joint_struct_in = safelyGetField(prhs[0], joint->getName());
    VectorXd q = matlabToEigen<Eigen::Dynamic, 1>(safelyGetField(joint_struct_in, "q"));
    VectorXd v = matlabToEigen<Eigen::Dynamic, 1>(safelyGetField(joint_struct_in, "v"));
    string name = joint->getName();

    mxArray* joint_struct_out = mxCreateStructArray(1, dims, 0, nullptr);

    Isometry3d joint_transform = joint->jointTransform(q);
    safelySetField(joint_struct_out, "joint_transform", eigenToMatlab(joint_transform.matrix()));

    Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> motion_subspace;
    MatrixXd dmotion_subspace;
    joint->motionSubspace(q, motion_subspace, &dmotion_subspace);
    safelySetField(joint_struct_out, "motion_subspace", eigenToMatlab(motion_subspace));
    safelySetField(joint_struct_out, "dmotion_subspace", eigenToMatlab(dmotion_subspace));

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d motion_subspace_dot_times_v;
    Gradient<Vector6d, Eigen::Dynamic>::type dmotion_subspace_dot_times_vdq;
    Gradient<Vector6d, Eigen::Dynamic>::type dmotion_subspace_dot_times_vdv;

    joint->motionSubspaceDotTimesV(q, v, motion_subspace_dot_times_v,
        &dmotion_subspace_dot_times_vdq,
        &dmotion_subspace_dot_times_vdv);
    safelySetField(joint_struct_out, "motion_subspace_dot_times_v", eigenToMatlab(motion_subspace_dot_times_v));
    safelySetField(joint_struct_out, "dmotion_subspace_dot_times_vdq", eigenToMatlab(dmotion_subspace_dot_times_vdq));
    safelySetField(joint_struct_out, "dmotion_subspace_dot_times_vdv", eigenToMatlab(dmotion_subspace_dot_times_vdv));

    MatrixXd qdot_to_v;
    MatrixXd dqdot_to_v;
    joint->qdot2v(q, qdot_to_v, &dqdot_to_v);
    safelySetField(joint_struct_out, "qdot_to_v", eigenToMatlab(qdot_to_v));
    safelySetField(joint_struct_out, "dqdot_to_v", eigenToMatlab(dqdot_to_v));

    MatrixXd v_to_qdot;
    MatrixXd dv_to_qdot;
    joint->v2qdot(q, v_to_qdot, &dv_to_qdot);
    safelySetField(joint_struct_out, "v_to_qdot", eigenToMatlab(v_to_qdot));
    safelySetField(joint_struct_out, "dv_to_qdot", eigenToMatlab(dv_to_qdot));

    int fieldnum = mxAddField(plhs[0], name.c_str());
    mxSetFieldByNumber(plhs[0], 0, fieldnum, joint_struct_out);
  }
}
