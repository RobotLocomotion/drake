#include "QuaternionFloatingJoint.h"
#include <random>

using namespace Eigen;
using namespace std;

std::string QuaternionFloatingJoint::getPositionName(int index) const
{
	switch (index) {
	case 0:
		return name+"_x";
	case 1:
		return name+"_y";
	case 2:
		return name+"_z";
	case 3:
		return name+"_qw";
	case 4:
		return name+"_qx";
	case 5:
		return name+"_qy";
	case 6:
		return name+"_qz";
	default:
		throw std::runtime_error("bad index");
	}
}

std::string QuaternionFloatingJoint::getVelocityName(int index) const
{
	switch (index) {
	case 0:
		return name+"_wx";
	case 1:
		return name+"_wy";
	case 2:
		return name+"_wz";
	case 3:
		return name+"_vx";
	case 4:
		return name+"_vy";
	case 5:
		return name+"_vz";
	default:
		throw std::runtime_error("bad index");
	}
}

VectorXd QuaternionFloatingJoint::randomConfiguration(std::default_random_engine& generator) const
{
	VectorXd q(7);
	normal_distribution<double> normal;

	// position
	q[0] = normal(generator);
	q[1] = normal(generator);
	q[2] = normal(generator);

	// orientation
	Vector4d quat = uniformlyRandomQuat(generator);
	q[3] = quat(0);
	q[4] = quat(1);
	q[5] = quat(2);
	q[6] = quat(3);
	return q;
}