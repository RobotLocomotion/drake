#ifndef ONEDOFJOINT_H_
#define ONEDOFJOINT_H_

#include "DrakeJoint.h"

class FixedAxisOneDoFJoint: public DrakeJoint
{
private:
  Eigen::Matrix<double, TWIST_SIZE, 1> joint_axis;

protected:
  FixedAxisOneDoFJoint(const std::string& name, const RigidBody& parent_body, const Eigen::AffineCompact3d& transform_to_parent_body, const Eigen::Matrix<double, TWIST_SIZE, 1>& joint_axis);

public:
  virtual ~FixedAxisOneDoFJoint();

  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, Eigen::MatrixXd* dmotion_subspace) const override;

  virtual void motionSubspaceDotTimesV(double* const q, double* const v, Vector6d& motion_subspace_dot_times_v,
      Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic>* dmotion_subspace_dot_times_vdq = nullptr,
      Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic>* dmotion_subspace_dot_times_vdv = nullptr) const override;
};

#endif /* ONEDOFJOINT_H_ */
