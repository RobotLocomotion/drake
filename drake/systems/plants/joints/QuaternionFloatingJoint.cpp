#include "QuaternionFloatingJoint.h"
#include <random>
#include "drakeGeometryUtil.h"

#include "RigidBodyManipulator.h" // todo: remove this when I remove setupOldKinematicTree

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

std::string QuaternionFloatingJoint::getPositionName(int index) const
{
	switch (index) {
	case 0:
		return name+"_x";
	case 1:
		return name+"_y";
	case 2:
		return name+"_z";
	case 3:
		return name+"_qw";
	case 4:
		return name+"_qx";
	case 5:
		return name+"_qy";
	case 6:
		return name+"_qz";
	default:
		throw std::runtime_error("bad index");
	}
}

std::string QuaternionFloatingJoint::getVelocityName(int index) const
{
	switch (index) {
	case 0:
		return name+"_wx";
	case 1:
		return name+"_wy";
	case 2:
		return name+"_wz";
	case 3:
		return name+"_vx";
	case 4:
		return name+"_vy";
	case 5:
		return name+"_vz";
	default:
		throw std::runtime_error("bad index");
	}
}

Isometry3d QuaternionFloatingJoint::jointTransform(const Eigen::Ref<const VectorXd>& q) const
{
  Isometry3d ret(Quaterniond(q[3], q[4], q[5], q[6]));
  ret.translation() << q[0], q[1], q[2];
  ret.makeAffine();
  return ret;
}

void QuaternionFloatingJoint::motionSubspace(const Eigen::Ref<const VectorXd>& q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace.setIdentity(TWIST_SIZE, getNumVelocities());
  if (dmotion_subspace) {
    dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
  }
}

void QuaternionFloatingJoint::motionSubspaceDotTimesV(const Eigen::Ref<const VectorXd>& q, const Eigen::Ref<const VectorXd>& v,
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

VectorXd QuaternionFloatingJoint::randomConfiguration(std::default_random_engine& generator) const
{
	VectorXd q(7);
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
  return q;
}

void QuaternionFloatingJoint::qdot2v(const Eigen::Ref<const VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const
{
  qdot_to_v.resize(getNumVelocities(), getNumPositions());

  auto quat = q.middleRows<QUAT_SIZE>(SPACE_DIMENSION);
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

void QuaternionFloatingJoint::v2qdot(const Eigen::Ref<const VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const
{
  v_to_qdot.resize(getNumPositions(), getNumVelocities());

  auto quat = q.middleRows<QUAT_SIZE>(SPACE_DIMENSION);
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
    angularvel2quatdotMatrix(quat, M, (Gradient<decltype(M), QUAT_SIZE, 1>::type*) nullptr);
  }

  v_to_qdot.block<3, 3>(0, 0).setZero();
  v_to_qdot.block<3, 3>(0, 3) = R;
  v_to_qdot.block<4, 3>(3, 0).noalias() = M * R;
  v_to_qdot.block<4, 3>(3, 3).setZero();
}

