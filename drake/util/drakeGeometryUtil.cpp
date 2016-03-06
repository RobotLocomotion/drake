#include "drake/util/drakeGeometryUtil.h"
#include <Eigen/Sparse>
#include <stdexcept>

using namespace Eigen;


double angleDiff(double phi1, double phi2)
{
  double d = phi2-phi1;
  if(d>0.0)
  {
    d = fmod(d+M_PI,2*M_PI)-M_PI;
  }
  else
  {
    d = fmod(d-M_PI,2*M_PI)+M_PI;
  }
  return d;
}

Vector4d uniformlyRandomAxisAngle(std::default_random_engine& generator)
{
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  double angle = uniform(generator);
  Vector3d axis = Vector3d(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Vector4d a;
  a << axis, angle;
  return a;
}

Vector4d uniformlyRandomQuat(std::default_random_engine& generator)
{
  return axis2quat(uniformlyRandomAxisAngle(generator));
}

Eigen::Matrix3d uniformlyRandomRotmat(std::default_random_engine& generator)
{
  return axis2rotmat(uniformlyRandomAxisAngle(generator));
}

Eigen::Vector3d uniformlyRandomRPY(std::default_random_engine& generator)
{
  return axis2rpy(uniformlyRandomAxisAngle(generator));
}

DRAKEGEOMETRYUTIL_EXPORT int rotationRepresentationSize(int rotation_type)
{
  switch (rotation_type) {
    case 0:
      return 0;
      break;
    case 1:
      return 3;
      break;
    case 2:
      return 4;
      break;
    default:
      throw std::runtime_error("rotation representation type not recognized");
  }
}

Matrix3d rotz(double theta) {
  // returns 3D rotation matrix (about the z axis)
  Matrix3d M;
  double c=cos(theta);
  double s=sin(theta);
  M << c,-s, 0,
      s, c, 0,
      0, 0, 1;
  return M;
}


void rotz(double theta, Matrix3d &M, Matrix3d &dM, Matrix3d &ddM)
{
  double c=cos(theta), s=sin(theta);
  M << c,-s,0, s,c,0, 0,0,1;
  dM << -s,-c,0, c,-s,0, 0,0,0;
  ddM << -c,s,0, -s,-c,0, 0,0,0;
}
