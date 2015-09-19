#include "RollPitchYawFloatingJoint.h"
#include <random>

using namespace Eigen;

std::string RollPitchYawFloatingJoint::getPositionName(int index) const
{
	switch (index) {
	case 0:
		return name+"_x";
	case 1:
		return name+"_y";
	case 2:
		return name+"_z";
	case 3:
		return name+"_roll";
	case 4:
		return name+"_pitch";
	case 5:
		return name+"_yaw";
	default:
		throw std::runtime_error("bad index");
	}
}

VectorXd RollPitchYawFloatingJoint::randomConfiguration(std::default_random_engine& generator) const
{
	VectorXd q(6);
  std::normal_distribution<double> normal;

  Map<Vector3d> pos(&q[0]);
  for (int i = 0; i < SPACE_DIMENSION; i++) {
    pos(i) = normal(generator);
  }

  Map<Vector3d> rpy(&q[3]);
  rpy = uniformlyRandomRPY(generator);
  return q;
}
