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

DRAKEGEOMETRYUTIL_EXPORT GradientVar<double,3,1> quat2expmap(const Ref<const Vector4d> &q, int gradient_order)
{
  double t = sqrt(1-q(0)*q(0));
  bool is_degenerate=(t*t<std::numeric_limits<double>::epsilon());
  double s = is_degenerate?2.0:2.0*acos(q(0))/t;
  GradientVar<double,3,1> ret(3,1,4,gradient_order);
  ret.value() = s*q.tail(3);
  if(gradient_order>0)
  {
    ret.gradient().value() = Matrix<double,3,4>::Zero();
    double dsdq1 = is_degenerate?0.0: (-2*t+2*acos(q(0))*q(0))/pow(t,3);
    ret.gradient().value().col(0) = q.tail(3)*dsdq1;
    ret.gradient().value().block(0,1,3,3) = Matrix3d::Identity()*s;
  }
  else if(gradient_order>1)
  {
    throw std::runtime_error("gradient_order>1 is not supported in quat2expmap");
  }
  return ret;
}

DRAKEGEOMETRYUTIL_EXPORT GradientVar<double,3,1> flipExpmap(const Ref<const Vector3d> &expmap, int gradient_order)
{
  if(gradient_order>1)
  {
    throw std::runtime_error("gradient_order>1 is not supported in flipExpmap");
  }
  double expmap_norm = expmap.norm();
  bool is_degenerate=(expmap_norm<std::numeric_limits<double>::epsilon());
  GradientVar<double,3,1> ret(3,1,3,gradient_order);
  Matrix3d eye3 = Matrix3d::Identity();
  if(is_degenerate)
  {
    ret.value() = expmap;
    if(gradient_order>0)
    {
      ret.gradient().value() = eye3;
    }
  }
  else
  {
    ret.value() = expmap-expmap/expmap_norm*2*M_PI;
    if(gradient_order>0)
    {
      ret.gradient().value() = eye3-(expmap_norm*expmap_norm*eye3-expmap*expmap.transpose())/pow(expmap_norm,3)*2*M_PI;
    }
  }
  return ret;
}

DRAKEGEOMETRYUTIL_EXPORT GradientVar<double, 3,1> unwrapExpmap(const Ref<const Vector3d> & expmap1, const Ref<const Vector3d> &expmap2, int gradient_order)
{
  auto expmap2_flip = flipExpmap(expmap2,gradient_order);
  double distance1 = (expmap1-expmap2).squaredNorm();
  double distance2 = (expmap1-expmap2_flip.value()).squaredNorm();
  if(distance1>distance2)
  {
    return expmap2_flip;
  }
  else
  {
    GradientVar<double,3,1> ret(3,1,3,gradient_order);
    ret.value() = expmap2;
    if(gradient_order>0)
    {
      ret.gradient().value() = Matrix3d::Identity();
    }
    return ret;
  }
}


void quat2expmapSequence(const Ref<const Matrix<double,4,Dynamic>> &quat, const Ref<const Matrix<double,4,Dynamic>> &quat_dot, Ref<Matrix<double,3,Dynamic>> expmap, Ref<Matrix<double,3,Dynamic>> expmap_dot)
{
  Eigen::Index N = quat.cols();
  if(quat_dot.cols() != N)
  {
    throw std::runtime_error("quat_dot must have the same number of columns as quat in quat2expmapSequence");
  }
  expmap.resize(3,N);
  expmap_dot.resize(3,N);
  for(int i = 0;i<N;i++)
  {
    auto expmap_grad = quat2expmap(quat.col(i),1);
    expmap.col(i) = expmap_grad.value();
    expmap_dot.col(i) = expmap_grad.gradient().value()*quat_dot.col(i);
    if(i>=1)
    {
      auto closest_grad = closestExpmap(expmap.col(i-1),expmap.col(i),1);
      expmap.col(i) = closest_grad.value();
      expmap_dot.col(i) = closest_grad.gradient().value()*expmap_dot.col(i);
    }
  }
}

DRAKEGEOMETRYUTIL_EXPORT GradientVar<double, 3,1> closestExpmap(const Ref<const Vector3d> & expmap1, const Ref<const Vector3d> &expmap2, int gradient_order)
{
  if (gradient_order>1) {
    throw std::runtime_error("closestExpmap only supports first order gradient");
  }
  double expmap1_norm = expmap1.norm();
  double expmap2_norm = expmap2.norm();
  GradientVar<double, 3, 1> ret(3,1,3,gradient_order);
  if (expmap2_norm < std::numeric_limits<double>::epsilon()) {
    if (expmap1_norm > std::numeric_limits<double>::epsilon()) {
      Vector3d expmap1_axis = expmap1/expmap1_norm;
      int expmap1_round = static_cast<int>(expmap1_norm/(2*M_PI) + 0.5);
      ret.value() = expmap1_axis*expmap1_round*2*M_PI;
      if(ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Zero();
      }
      return ret;
    }
    else {
      ret.value() = expmap2;
      if (ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Identity();
      }
    }
  }
  else {
    Vector3d expmap2_axis = expmap2/expmap2_norm;
    Matrix3d dexpmap2_axis_dexpmap2 = (expmap2_norm*Matrix3d::Identity() - expmap2*expmap2.transpose()/expmap2_norm)/pow(expmap2_norm,2);
    double expmap2_closest_k = (expmap2_axis.transpose()*expmap1 - expmap2_norm)/(2*M_PI);
    int expmap2_closest_k1;
    int expmap2_closest_k2;
    if (expmap2_closest_k>0) {
      expmap2_closest_k1 = (int) expmap2_closest_k;
    }
    else {
      expmap2_closest_k1 = (int) expmap2_closest_k - 1;
    }
    expmap2_closest_k2 = expmap2_closest_k1 + 1;
    Vector3d expmap2_closest1 = expmap2 + 2*expmap2_closest_k1*M_PI*expmap2_axis;
    Vector3d expmap2_closest2 = expmap2 + 2*expmap2_closest_k2*M_PI*expmap2_axis;
    if ((expmap2_closest1 - expmap1).norm() < (expmap2_closest2 - expmap1).norm()) {
      ret.value() = expmap2_closest1;
      if (ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Identity() + 2*dexpmap2_axis_dexpmap2*(double)expmap2_closest_k1*M_PI;
      }
      return ret;
    }
    else {
      ret.value() = expmap2_closest2;
      if (ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Identity() + 2*dexpmap2_axis_dexpmap2*(double)expmap2_closest_k2*M_PI;
      }
      return ret;
    }
  }
  return ret;
}
