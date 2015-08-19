#include "RollPitchYawFloatingJoint.h"
#include <random>

#include "RigidBodyManipulator.h" // todo: remove this when I remove setupOldKinematicTree

using namespace Eigen;


RollPitchYawFloatingJoint::RollPitchYawFloatingJoint(const std::string& name, const Isometry3d& transform_to_parent_body) :
  DrakeJoint(name, transform_to_parent_body, 6, 6)
{
  // empty
}

RollPitchYawFloatingJoint::~RollPitchYawFloatingJoint()
{
  // empty
}

std::string RollPitchYawFloatingJoint::getPositionName(int index) const
{
	switch (index) {
	case 0:
		return name+"_x";
	case 1:
		return name+"_y";
	case 2:
		return name+"_z";
	case 3:
		return name+"_roll";
	case 4:
		return name+"_pitch";
	case 5:
		return name+"_yaw";
	default:
		throw std::runtime_error("bad index");
	}
}

Isometry3d RollPitchYawFloatingJoint::jointTransform(const Eigen::Ref<const VectorXd>& q) const
{
  Isometry3d ret;
  auto pos = q.middleRows<SPACE_DIMENSION>(0);
  auto rpy = q.middleRows<RPY_SIZE>(SPACE_DIMENSION);
  ret.linear() = rpy2rotmat(rpy);
  ret.translation() = pos;
  ret.makeAffine();
  return ret;
}

void RollPitchYawFloatingJoint::motionSubspace(const Eigen::Ref<const VectorXd>& q, MotionSubspaceType& motion_subspace, MatrixXd* dmotion_subspace) const
{
  motion_subspace.resize(TWIST_SIZE, getNumVelocities());
  auto rpy = q.middleRows<RPY_SIZE>(SPACE_DIMENSION);
  Matrix<double,SPACE_DIMENSION,RPY_SIZE> E;
  rpydot2angularvelMatrix(rpy,E);
  auto R = rpy2rotmat(rpy);
  motion_subspace.block<3, 3>(0, 0).setZero();
  motion_subspace.block<3, 3>(0, 3) = R.transpose() * E;
  motion_subspace.block<3, 3>(3, 0) = R.transpose();
  motion_subspace.block<3, 3>(3, 3).setZero();

  if (dmotion_subspace) {
    dmotion_subspace->resize(motion_subspace.size(), getNumPositions());

    using namespace std;
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    dmotion_subspace->transpose() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sr
        * sy + cr * cy * sp, cr * sy - cy * sp * sr, 0.0, 0.0, 0.0, 0.0, -cy * sr + cr * sp * sy, -cr * cy - sp * sr * sy, 0.0, 0.0, 0.0, 0.0, cp * cr, -cp * sr, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -sr, -cr, 0.0, 0.0, 0.0, 0.0, cp * cr, -cp * sr, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -cy * sp, cp * cy * sr, cp
        * cr * cy, 0.0, 0.0, 0.0, -sp * sy, cp * sr * sy, cp * cr * sy, 0.0, 0.0, 0.0, -cp, -sp * sr, -cr * sp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -cp, -sp * sr, -cr * sp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -cp * sy, -cr * cy - sp * sr * sy, cy * sr - cr * sp * sy, 0.0, 0.0, 0.0, cp
        * cy, -cr * sy + cy * sp * sr, sr * sy + cr*cy*sp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
}

void RollPitchYawFloatingJoint::motionSubspaceDotTimesV(const Eigen::Ref<const VectorXd>& q, const Eigen::Ref<const VectorXd>& v,
    Vector6d& motion_subspace_dot_times_v,
    Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq,
    Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv) const
{
  motion_subspace_dot_times_v.resize(TWIST_SIZE, 1);
  auto rpy = q.middleRows<RPY_SIZE>(SPACE_DIMENSION);
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);

  auto pd = v.middleRows<SPACE_DIMENSION>(0);
  double xd = pd(0);
  double yd = pd(1);
  double zd = pd(2);

  auto rpyd = v.middleRows<RPY_SIZE>(SPACE_DIMENSION);
  double rolld = rpyd(0);
  double pitchd = rpyd(1);
  double yawd = rpyd(2);

  using namespace std;
  double cr = cos(roll);
  double sr = sin(roll);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cy = cos(yaw);
  double sy = sin(yaw);

  motion_subspace_dot_times_v.transpose() << -pitchd * yawd * cp, rolld * yawd * cp * cr - pitchd * yawd * sp * sr - pitchd * rolld * sr, -pitchd * rolld * cr - pitchd * yawd * cr * sp - rolld * yawd * cp * sr, yd * (yawd * cp * cy - pitchd * sp * sy) - xd * (pitchd * cy * sp + yawd * cp * sy)
      - pitchd * zd * cp, zd * (rolld * cp * cr - pitchd * sp * sr) + xd * (rolld * (sr * sy + cr * cy * sp) - yawd * (cr * cy + sp * sr * sy) + pitchd * cp * cy * sr) - yd * (rolld * (cy * sr - cr * sp * sy) + yawd * (cr * sy - cy * sp * sr) - pitchd * cp * sr * sy), xd
      * (rolld * (cr * sy - cy * sp * sr) + yawd * (cy * sr - cr * sp * sy) + pitchd * cp * cr * cy) - zd * (pitchd * cr * sp + rolld * cp * sr) + yd * (yawd * (sr * sy + cr * cy * sp) - rolld * (cr * cy + sp * sr * sy) + pitchd * cp * cr * sy);

  if (dmotion_subspace_dot_times_vdq) {
    dmotion_subspace_dot_times_vdq->resize(motion_subspace_dot_times_v.rows(), getNumPositions());
    dmotion_subspace_dot_times_vdq->transpose() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -pitchd * rolld * cr - pitchd * yawd * cr * sp - rolld * yawd * cp * sr, pitchd * rolld * sr + pitchd * yawd * sp * sr - rolld * yawd * cp * cr, 0.0, xd
        * (rolld * (cr * sy - cy * sp * sr) + yawd * (cy * sr - cr * sp * sy) + pitchd * cp * cr * cy) - zd * (pitchd * cr * sp + rolld * cp * sr) + yd * (-rolld * (cr * cy + sp * sr * sy) + yawd * (sr * sy + cr * cy * sp) + pitchd * cp * cr * sy), -zd * (rolld * cp * cr - pitchd * sp * sr)
        - xd * (rolld * (sr * sy + cr * cy * sp) - yawd * (cr * cy + sp * sr * sy) + pitchd * cp * cy * sr) + yd * (rolld * (cy * sr - cr * sp * sy) + yawd * (cr * sy - cy * sp * sr) - pitchd * cp * sr * sy), pitchd * yawd * sp, -pitchd * yawd * cp * sr - rolld * yawd * cr * sp, rolld * yawd * sp
        * sr - pitchd * yawd * cp * cr, -xd * (pitchd * cp * cy - yawd * sp * sy) - yd * (pitchd * cp * sy + yawd * cy * sp) + pitchd * zd * sp, -zd * (pitchd * cp * sr + rolld * cr * sp) - xd * (-rolld * cp * cr * cy + pitchd * cy * sp * sr + yawd * cp * sr * sy)
        + yd * (rolld * cp * cr * sy + yawd * cp * cy * sr - pitchd * sp * sr * sy), -zd * (pitchd * cp * cr - rolld * sp * sr) - xd * (pitchd * cr * cy * sp + rolld * cp * cy * sr + yawd * cp * cr * sy) - yd * (-yawd * cp * cr * cy + pitchd * cr * sp * sy + rolld * cp * sr * sy), 0.0, 0.0, 0.0, -xd
        * (yawd * cp * cy - pitchd * sp * sy) - yd * (pitchd * cy * sp + yawd * cp * sy), yd * (rolld * (sr * sy + cr * cy * sp) - yawd * (cr * cy + sp * sr * sy) + pitchd * cp * cy * sr) + xd * (rolld * (cy * sr - cr * sp * sy) + yawd * (cr * sy - cy * sp * sr) - pitchd * cp * sr * sy), yd
        * (rolld * (cr * sy - cy * sp * sr) + yawd * (cy * sr - cr * sp * sy) + pitchd * cp * cr * cy) - xd * (-rolld * (cr * cy + sp * sr * sy) + yawd * (sr * sy + cr * cy * sp) + pitchd * cp * cr * sy);
  }

  if (dmotion_subspace_dot_times_vdv) {
    dmotion_subspace_dot_times_vdv->resize(motion_subspace_dot_times_v.rows(), getNumVelocities());
    dmotion_subspace_dot_times_vdv->transpose() << 0.0, 0.0, 0.0, -pitchd * cy * sp - yawd * cp * sy, rolld * (sr * sy + cr * cy * sp) - yawd * (cr * cy + sp * sr * sy) + pitchd * cp * cy * sr, rolld * (cr * sy - cy * sp * sr) + yawd * (cy * sr - cr * sp * sy) + pitchd * cp * cr * cy, 0.0, 0.0, 0.0, yawd * cp
        * cy - pitchd * sp * sy, -rolld * (cy * sr - cr * sp * sy) - yawd * (cr * sy - cy * sp * sr) + pitchd * cp * sr * sy, -rolld * (cr * cy + sp * sr * sy) + yawd * (sr * sy + cr * cy * sp) + pitchd * cp * cr * sy, 0.0, 0.0, 0.0, -pitchd * cp, rolld * cp * cr - pitchd * sp * sr, -pitchd * cr
        * sp - rolld * cp * sr, 0.0, -pitchd * sr + yawd * cp * cr, -pitchd * cr - yawd * cp * sr, 0.0, xd * (sr * sy + cr * cy * sp) - yd * (cy * sr - cr * sp * sy) + zd * cp * cr, xd * (cr * sy - cy * sp * sr) - yd * (cr * cy + sp * sr * sy) - zd * cp * sr, -yawd * cp, -sr * (rolld + yawd * sp), -cr
        * (rolld + yawd * sp), -zd * cp - xd * cy * sp - yd * sp * sy, sr * (-zd * sp + xd * cp * cy + yd * cp * sy), cr * (-zd * sp + xd * cp * cy + yd * cp * sy), -pitchd * cp, rolld * cp * cr - pitchd * sp * sr, -pitchd * cr * sp - rolld * cp * sr, cp * (yd * cy - xd * sy), -xd
        * (cr * cy + sp * sr * sy) - yd * (cr * sy - cy * sp * sr), xd * (cy * sr - cr * sp * sy) + yd * (sr * sy + cr * cy * sp);
  }
}

VectorXd RollPitchYawFloatingJoint::randomConfiguration(std::default_random_engine& generator) const
{
	VectorXd q(6);
  std::normal_distribution<double> normal;

  Map<Vector3d> pos(&q[0]);
  for (int i = 0; i < SPACE_DIMENSION; i++) {
    pos(i) = normal(generator);
  }

  Map<Vector3d> rpy(&q[3]);
  rpy = uniformlyRandomRPY(generator);
  return q;
}

void RollPitchYawFloatingJoint::qdot2v(const Eigen::Ref<const VectorXd>& q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const
{
  qdot_to_v.setIdentity(getNumVelocities(), getNumPositions());

  if (dqdot_to_v) {
    dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
  }
}

void RollPitchYawFloatingJoint::v2qdot(const Eigen::Ref<const VectorXd>& q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const
{
  v_to_qdot.setIdentity(getNumPositions(), getNumVelocities());

  if (dv_to_qdot) {
    dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
  }
}

