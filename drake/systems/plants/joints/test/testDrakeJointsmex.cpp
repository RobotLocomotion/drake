#include <mex.h>
#include <matrix.h>

#include "drake/systems/plants/joints/Joint.h"
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>
#include "drake/util/drakeMexUtil.h"

using namespace Drake;
using namespace Eigen;
using namespace std;

mxArray* safelyGetField(const mxArray* array, string field_name,
                        int index = 0) {
  mxArray* ret = mxGetField(array, index, field_name.c_str());
  if (!ret) {
    mexErrMsgTxt(("Field not found: " + field_name).c_str());
  }
  return ret;
}

void safelySetField(mxArray* array, const string& fieldname, mxArray* data,
                    int index = 0) {
  int fieldnum;
  fieldnum = mxGetFieldNumber(array, fieldname.c_str());
  if (fieldnum < 0) {
    fieldnum = mxAddField(array, fieldname.c_str());
  }
  mxSetFieldByNumber(array, index, fieldnum, data);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs != 1 || nlhs != 1) {
    mexErrMsgIdAndTxt("Drake:testDrakeJointsmex:BadInputs",
                      "Usage struct_out = testGeometryGradientsmex(struct_in)");
  }
  string prismaticJointName = "prismatic";
  mxArray* mxPrismatic = safelyGetField(prhs[0], prismaticJointName);
  Vector3d prismatic_joint_axis =
      matlabToEigen<3, 1>(safelyGetField(mxPrismatic, "joint_axis"));

  string revoluteJointName = "revolute";
  mxArray* mxRevolute = safelyGetField(prhs[0], revoluteJointName);
  Vector3d revolute_joint_axis =
      matlabToEigen<3, 1>(safelyGetField(mxRevolute, "joint_axis"));

  string helicalJointName = "helical";
  mxArray* mxHelical = safelyGetField(prhs[0], helicalJointName);
  Vector3d helical_joint_axis =
      matlabToEigen<3, 1>(safelyGetField(mxHelical, "joint_axis"));
  double helical_joint_pitch = *mxGetPr(safelyGetField(mxHelical, "pitch"));

  Isometry3d transform_to_parent = Isometry3d::Identity();

  Joint<double> revolute(revoluteJointName, transform_to_parent, std::unique_ptr<JointType<double>>(new Revolute<double>(revolute_joint_axis)));
  Joint<double> prismatic(prismaticJointName, transform_to_parent, std::unique_ptr<JointType<double>>(new Prismatic<double>(prismatic_joint_axis)));
  Joint<double> helical(helicalJointName, transform_to_parent, std::unique_ptr<JointType<double>>(new Helical<double>(helical_joint_axis, helical_joint_pitch)));
  Joint<double> quaternionFloating("quaternion_floating", transform_to_parent, std::unique_ptr<JointType<double>>(new QuaternionFloating<double>()));
  Joint<double> rpyFloating("rpy_floating", transform_to_parent, std::unique_ptr<JointType<double>>(new RollPitchYawFloating<double>()));
  vector<Joint<double>*> joints({&revolute, &prismatic, &helical, &quaternionFloating, &rpyFloating});

  const mwSize ndim = 1;
  mwSize dims[] = {1};
  plhs[0] = mxCreateStructArray(ndim, dims, 0, nullptr);
  for (const auto& joint : joints) {
    mxArray *joint_struct_in = safelyGetField(prhs[0], joint->getName());
    VectorXd q = matlabToEigen<Eigen::Dynamic, 1>(safelyGetField(joint_struct_in, "q"));
    VectorXd v = matlabToEigen<Eigen::Dynamic, 1>(safelyGetField(joint_struct_in, "v"));
    string name = joint->getName();

    mxArray *joint_struct_out = mxCreateStructArray(1, dims, 0, nullptr);

    Isometry3d joint_transform = joint->jointTransform(q);
    safelySetField(joint_struct_out, "joint_transform", eigenToMatlab(joint_transform.matrix()));

    auto motion_subspace = joint->motionSubspace(q);
    safelySetField(joint_struct_out, "motion_subspace", eigenToMatlab(motion_subspace));

    auto motion_subspace_dot_times_v = joint->motionSubspaceDotTimesV(q, v);
    safelySetField(joint_struct_out, "motion_subspace_dot_times_v", eigenToMatlab(motion_subspace_dot_times_v));

    auto qdot_to_v = joint->configurationDerivativeToVelocity(q);
    safelySetField(joint_struct_out, "qdot_to_v", eigenToMatlab(qdot_to_v));

    auto v_to_qdot = joint->velocityToConfigurationDerivative(q);
    safelySetField(joint_struct_out, "v_to_qdot", eigenToMatlab(v_to_qdot));

    int fieldnum = mxAddField(plhs[0], name.c_str());
    mxSetFieldByNumber(plhs[0], 0, fieldnum, joint_struct_out);
  }
}
