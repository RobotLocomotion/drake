#include "QuaternionFloatingJoint.h"
#include <random>
#include "drakeGeometryUtil.h"

using namespace Eigen;
using namespace std;

QuaternionFloatingJoint::QuaternionFloatingJoint(const string& name, const Isometry3d& transform_to_parent_body) :
  DrakeJoint(name, transform_to_parent_body, 7, 6)
{
  // empty
}

QuaternionFloatingJoint::~QuaternionFloatingJoint()
{
  // empty
}

Isometry3d QuaternionFloatingJoint::jointTransform(double* const q) const
{
  Isometry3d ret(Quaterniond(q[3], q[4], q[5], q[6]));
  ret.translation() << q[0], q[1], q[2];
  ret.makeAffine();
  return ret;
}

void QuaternionFloatingJoint::motionSubspace(double* const q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace.setIdentity(TWIST_SIZE, getNumVelocities());
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void QuaternionFloatingJoint::motionSubspaceDotTimesV(double* const q, double* const v,
    Vector6d& motion_subspace_dot_times_v,
    Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq,
    Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv) const
{
  motion_subspace_dot_times_v.setZero();
  if (dmotion_subspace_dot_times_vdq) {
    dmotion_subspace_dot_times_vdq->setZero(motion_subspace_dot_times_v.size(), getNumPositions());
  }
  if (dmotion_subspace_dot_times_vdv) {
    dmotion_subspace_dot_times_vdv->setZero(motion_subspace_dot_times_v.size(), getNumVelocities());
  }
}

void QuaternionFloatingJoint::randomConfiguration(double* q, std::default_random_engine& generator) const
{
  normal_distribution<double> normal;

  // position
  q[0] = normal(generator);
  q[1] = normal(generator);
  q[2] = normal(generator);

  // orientation
  Vector4d quat = uniformlyRandomQuat(generator);
  q[3] = quat(0);
  q[4] = quat(1);
  q[5] = quat(2);
  q[6] = quat(3);
}

void QuaternionFloatingJoint::qdot2v(double* q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const
{
  qdot_to_v.resize(getNumVelocities(), getNumPositions());

  Map<Vector4d> quat(&q[3]);
  Matrix3d R = quat2rotmat(quat);

  Vector4d quattilde;
  Matrix<double, SPACE_DIMENSION, QUAT_SIZE> M;
  Matrix<double, SPACE_DIMENSION, QUAT_SIZE> RTransposeM;
  Gradient<Vector4d, QUAT_SIZE, 1>::type dquattildedquat;
  if (dqdot_to_v) {
    Gradient<Vector4d, QUAT_SIZE, 2>::type ddquattildedquat;
    normalizeVec(quat, quattilde, &dquattildedquat, &ddquattildedquat);
    auto dR = dquat2rotmat(quat);
    Gradient<Matrix<double, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type dM;
    quatdot2angularvelMatrix(quat, M, &dM);

    RTransposeM.noalias() = R.transpose() * M;
    auto dRTranspose = transposeGrad(dR, R.rows());
    auto dRTransposeM = matGradMultMat(R.transpose(), M, dRTranspose, dM);
    auto dRTransposeMdquattildedquat = matGradMultMat(RTransposeM, dquattildedquat, dRTransposeM, ddquattildedquat);
    dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
    setSubMatrixGradient<4>(*dqdot_to_v, dRTranspose, intRange<3>(3), intRange<3>(0), qdot_to_v.rows(), 3);
    setSubMatrixGradient<4>(*dqdot_to_v, dRTransposeMdquattildedquat, intRange<3>(0), intRange<4>(3), qdot_to_v.rows(), 3);
  }
  else {
    normalizeVec(quat, quattilde, &dquattildedquat);
    quatdot2angularvelMatrix(quat, M);
    RTransposeM.noalias() = R.transpose() * M;
  }
  qdot_to_v.block<3, 3>(0, 0).setZero();
  qdot_to_v.block<3, 4>(0, 3).noalias() = RTransposeM * dquattildedquat;
  qdot_to_v.block<3, 3>(3, 0) = R.transpose();
  qdot_to_v.block<3, 4>(3, 3).setZero();
}

void QuaternionFloatingJoint::v2qdot(double* q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const
{
  v_to_qdot.resize(getNumPositions(), getNumVelocities());

  Map<Vector4d> quat(&q[3]);
  Matrix3d R = quat2rotmat(quat);

  Matrix<double, QUAT_SIZE, SPACE_DIMENSION> M;
  if (dv_to_qdot) {
    auto dR = dquat2rotmat(quat);
    Gradient<decltype(M), QUAT_SIZE, 1>::type dM;
    angularvel2quatdotMatrix(quat, M, &dM);

    dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());

    setSubMatrixGradient<4>(*dv_to_qdot, dR, intRange<3>(0), intRange<3>(3), v_to_qdot.rows(), 3);
    auto dMR = matGradMultMat(M, R, dM, dR);
    setSubMatrixGradient<4>(*dv_to_qdot, dMR, intRange<4>(3), intRange<3>(0), v_to_qdot.rows(), 3);
  }
  else {
    angularvel2quatdotMatrix(quat, M);
  }

  v_to_qdot.block<3, 3>(0, 0).setZero();
  v_to_qdot.block<3, 3>(0, 3) = R;
  v_to_qdot.block<4, 3>(3, 0).noalias() = M * R;
  v_to_qdot.block<4, 3>(3, 3).setZero();
}
