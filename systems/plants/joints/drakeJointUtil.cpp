#include "drakeJointUtil.h"
#include "makeUnique.h"
#include "DrakeJoint.h"
#include "HelicalJoint.h"
#include "PrismaticJoint.h"
#include "RevoluteJoint.h"
#include "QuaternionFloatingJoint.h"
#include "RollPitchYawFloatingJoint.h"

using namespace Eigen;
using namespace std;

unique_ptr<DrakeJoint> createJoint(const string& joint_name, const Isometry3d& transform_to_parent_body, int floating, const Vector3d& joint_axis, double pitch)
{
  unique_ptr<DrakeJoint> joint;
  switch (floating) {
  case 0: {
    if (pitch == 0.0) {
      joint = make_unique<RevoluteJoint>(joint_name, transform_to_parent_body, joint_axis);
    } else if (isinf(pitch)) {
      joint = make_unique<PrismaticJoint>(joint_name, transform_to_parent_body, joint_axis);
    } else {
      joint = make_unique<HelicalJoint>(joint_name, transform_to_parent_body, joint_axis, pitch);
    }
    break;
  }
  case 1: {
    joint = make_unique<RollPitchYawFloatingJoint>(joint_name, transform_to_parent_body);
    break;
  }
  case 2: {
    joint = make_unique<QuaternionFloatingJoint>(joint_name, transform_to_parent_body);
    break;
  }
  default: {
    ostringstream stream;
    stream << "floating type " << floating << " not recognized.";
    throw runtime_error(stream.str());
    break;
  }
  }
  return joint;
}
