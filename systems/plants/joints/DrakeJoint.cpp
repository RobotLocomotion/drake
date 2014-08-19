#include "DrakeJoint.h"

using namespace Eigen;

DrakeJoint::DrakeJoint(
    const std::string& name,
    const Isometry3d& transform_to_parent_body,
    int num_positions, int num_velocities) :
name(name), transform_to_parent_body(transform_to_parent_body), num_positions(num_positions), num_velocities(num_velocities)
{
  // empty;
}

DrakeJoint::~DrakeJoint()
{
  // empty
}

const Isometry3d& DrakeJoint::getTransformToParentBody() const
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

const std::string& DrakeJoint::getName() const
{
  return name;
}
