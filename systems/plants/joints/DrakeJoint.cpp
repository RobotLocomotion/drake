#include "DrakeJoint.h"
#include "RigidBodyManipulator.h"

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

void DrakeJoint::setupOldKinematicTree(RigidBodyManipulator* model, int body_ind, int position_num_start, int velocity_num_start) const
{
  model->bodies[body_ind]->jointname = name;
  model->bodies[body_ind]->Ttree = transform_to_parent_body.matrix();
//  model->bodies[body_ind]->T_body_to_joint = Matrix4d::Identity();
  model->bodies[body_ind]->floating = 0;
  model->bodies[body_ind]->pitch = 0;
}

