#ifndef ONEDOFJOINT_H_
#define ONEDOFJOINT_H_

#include "DrakeJoint.h"
#include "DrakeJoint.cpp" // need to include .cpp as well because DrakeJoint is a template class

class OneDoFJoint: public DrakeJoint<1, 1>
{
private:
  e::Matrix<double, TWIST_SIZE, 1> joint_axis;

protected:
  OneDoFJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body, const e::Matrix<double, TWIST_SIZE, 1>& joint_axis);

public:
  virtual ~OneDoFJoint();

//  virtual void motionSubspace(double* const q, MotionSubspaceType& motion_subspace, DMotionSubspaceType* dmotion_subspace = nullptr) const override;

//  virtual e::AffineCompact3d jointTransform(double* const q) const override;
};

#endif /* ONEDOFJOINT_H_ */
