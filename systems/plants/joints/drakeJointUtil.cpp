#include "drakeJointUtil.h"
#include "DrakeJoint.h"
#include "HelicalJoint.h"
#include "PrismaticJoint.h"
#include "RevoluteJoint.h"
#include "QuaternionFloatingJoint.h"
#include "RollPitchYawFloatingJoint.h"
#include "drakeFloatingPointUtil.h"
#include <cmath>
#include <stdexcept>

using namespace Eigen;

std::unique_ptr<DrakeJoint> createJoint(const std::string& joint_name, const Isometry3d& transform_to_parent_body, int floating, const Vector3d& joint_axis, double pitch)
{
  std::unique_ptr<DrakeJoint> joint;
  switch (floating) {
  case 0: {
    if (pitch == 0.0) {
      joint = std::unique_ptr<RevoluteJoint>(new RevoluteJoint(joint_name, transform_to_parent_body, joint_axis));
    } else if (isInf(pitch)) {
      joint = std::unique_ptr<PrismaticJoint>(new PrismaticJoint(joint_name, transform_to_parent_body, joint_axis));
    } else {
      joint = std::unique_ptr<HelicalJoint>(new HelicalJoint(joint_name, transform_to_parent_body, joint_axis, pitch));
    }
    break;
  }
  case 1: {
    joint = std::unique_ptr<RollPitchYawFloatingJoint>(new RollPitchYawFloatingJoint(joint_name, transform_to_parent_body));
    break;
  }
  case 2: {
    joint = std::unique_ptr<QuaternionFloatingJoint>(new QuaternionFloatingJoint(joint_name, transform_to_parent_body));
    break;
  }
  default: {
    std::ostringstream stream;
    stream << "floating type " << floating << " not recognized.";
    throw std::runtime_error(stream.str());
    break;
  }
  }
  return joint;
}
