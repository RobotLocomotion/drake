#include "DrakeJoint.h"

using namespace Eigen;

DrakeJoint::DrakeJoint(
    const std::string& name,
    const Isometry3d& transform_to_parent_body,
    int num_positions, int num_velocities) :
_name(name), _transform_to_parent_body(transform_to_parent_body), _num_positions(num_positions), _num_velocities(num_velocities)
{
  // empty;
}

DrakeJoint::~DrakeJoint()
{
  // empty
}

const Isometry3d& DrakeJoint::getTransformToParentBody() const
{
  return _transform_to_parent_body;
}

const int DrakeJoint::getNumPositions() const
{
  return _num_positions;
}

const int DrakeJoint::getNumVelocities() const
{
  return _num_velocities;
}

const std::string& DrakeJoint::getName() const
{
  return _name;
}
