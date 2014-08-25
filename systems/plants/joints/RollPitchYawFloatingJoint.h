#ifndef ROLLPITCHYAWFLOATINGJOINT_H_
#define ROLLPITCHYAWFLOATINGJOINT_H_

#include "DrakeJoint.h"

class RollPitchYawFloatingJoint: public DrakeJoint
{
public:
  // disable copy construction and assignment
  RollPitchYawFloatingJoint(const RollPitchYawFloatingJoint&) = delete;
  RollPitchYawFloatingJoint& operator=(const RollPitchYawFloatingJoint&) = delete;

public:
  RollPitchYawFloatingJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body);

  virtual ~RollPitchYawFloatingJoint();

  virtual Eigen::Isometry3d jointTransform(double* const q) const; //override;

  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace = nullptr) const; //override;

  virtual void motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
      typename Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      typename Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const; //override;

  virtual void randomConfiguration(double* const q, std::default_random_engine& generator) const; //override;

  virtual void qdot2v(double* q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const; //override;

  virtual void v2qdot(double* q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const; //override;
};

#endif /* ROLLPITCHYAWFLOATINGJOINT_H_ */
