#ifndef QUATERNIONFLOATINGJOINT_H_
#define QUATERNIONFLOATINGJOINT_H_

#include "DrakeJoint.h"

class QuaternionFloatingJoint: public DrakeJoint
{
  // disable copy construction and assignment
  QuaternionFloatingJoint(const QuaternionFloatingJoint&) = delete;
  QuaternionFloatingJoint& operator=(const QuaternionFloatingJoint&) = delete;

public:
  static const int NUM_POSITIONS = 7;
  static const int NUM_VELOCITIES = 6;

public:
  QuaternionFloatingJoint(const std::string& name, const RigidBody& parent_body, const Eigen::Isometry3d& transform_to_parent_body);

  virtual ~QuaternionFloatingJoint();

  virtual Eigen::Isometry3d jointTransform(double* const q) const override;

  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace = nullptr) const override;

  virtual void motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
      Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic>* dmotion_subspace_dot_times_vdq = nullptr,
      Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic>* dmotion_subspace_dot_times_vdv = nullptr) const override;

  virtual void randomConfiguration(double* q, std::default_random_engine& generator) const override;

  virtual void qdotToV(double* q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const override;

  virtual void vToQdot(double* q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const override;
};

#endif /* QUATERNIONFLOATINGJOINT_H_ */
