#include "DrakeJoint.h"

DrakeJoint::DrakeJoint(
    const RigidBody& parent_body,
    const e::AffineCompact3d& transform_to_parent_body,
    int num_positions, int num_velocities) :
parent_body(parent_body), transform_to_parent_body(transform_to_parent_body), num_positions(num_positions), num_velocities(num_velocities)
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

const int DrakeJoint::getNumPositions() const
{
  return num_positions;
}

const int DrakeJoint::getNumVelocities() const
{
  return num_velocities;
}
