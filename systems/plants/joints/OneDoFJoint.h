#ifndef ONEDOFJOINT_H_
#define ONEDOFJOINT_H_

#include "DrakeJoint.h"

class OneDoFJoint: public DrakeJoint
{
private:
  e::Matrix<double, TWIST_SIZE, 1> joint_axis;

protected:
  OneDoFJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Matrix<double, TWIST_SIZE, 1>& joint_axis);

public:
  virtual ~OneDoFJoint();

  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, e::MatrixXd* dmotion_subspace) const override;
};

#endif /* ONEDOFJOINT_H_ */
