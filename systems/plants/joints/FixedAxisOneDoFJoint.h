#ifndef ONEDOFJOINT_H_
#define ONEDOFJOINT_H_

#include "DrakeJoint.h"

class FixedAxisOneDoFJoint : public DrakeJoint
{
  // disable copy construction and assignment
  FixedAxisOneDoFJoint(const DrakeJoint&) = delete;
  FixedAxisOneDoFJoint& operator=(const FixedAxisOneDoFJoint&) = delete;

private:
  Eigen::Matrix<double, TWIST_SIZE, 1> joint_axis;
  double joint_limit_min;
  double joint_limit_max;

protected:
  FixedAxisOneDoFJoint(const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Matrix<double, TWIST_SIZE, 1>& joint_axis);

public:
  virtual ~FixedAxisOneDoFJoint();

  void setJointLimits(double joint_limit_min, double joint_limit_max);

  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace) const; //override;

  virtual void motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
      typename Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdq = nullptr,
      typename Gradient<Vector6d, Eigen::Dynamic>::type* dmotion_subspace_dot_times_vdv = nullptr) const; //override;

  virtual void randomConfiguration(double* const q, std::default_random_engine& generator) const; //override;

  virtual void qdot2v(double* q, Eigen::MatrixXd& qdot_to_v, Eigen::MatrixXd* dqdot_to_v) const; //override;

  virtual void v2qdot(double* q, Eigen::MatrixXd& v_to_qdot, Eigen::MatrixXd* dv_to_qdot) const; //override;
};

#endif /* ONEDOFJOINT_H_ */
