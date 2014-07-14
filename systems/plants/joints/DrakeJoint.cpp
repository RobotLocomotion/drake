#include "DrakeJoint.h"

DrakeJoint::DrakeJoint(const RigidBody& parent_body, const e::AffineCompact3d& transform_to_parent_body) :
parent_body(parent_body), transform_to_parent_body(transform_to_parent_body)
{
  // empty;
}

DrakeJoint::~DrakeJoint()
{
  // empty
}

const RigidBody& DrakeJoint::getParentBody() const
{
  return parent_body;
}

const e::AffineCompact3d& DrakeJoint::getTransformToParentBody() const
{
  return transform_to_parent_body;
}
