/*
* drakeUtil.cpp
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include "drake/util/drakeUtil.h"
#include <Eigen/Dense>
#include <stdexcept>

using namespace std;
using namespace Eigen;

void baseZeroToBaseOne(std::vector<int>& vec) {
  for (std::vector<int>::iterator iter = vec.begin(); iter != vec.end(); iter++)
    (*iter)++;
}

double angleAverage(double theta1, double theta2) {
  // Computes the average between two angles by averaging points on the unit
  // circle and taking the arctan of the result.
  //   see: http://en.wikipedia.org/wiki/Mean_of_circular_quantities
  // theta1 is a scalar or column vector of angles (rad)
  // theta2 is a scalar or column vector of angles (rad)

  double x_mean = 0.5 * (cos(theta1) + cos(theta2));
  double y_mean = 0.5 * (sin(theta1) + sin(theta2));

  double angle_mean = atan2(y_mean, x_mean);

  return angle_mean;
}

template <typename DerivedTorque, typename DerivedForce, typename DerivedNormal,
          typename DerivedPoint>
std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(
    const Eigen::MatrixBase<DerivedTorque>& torque,
    const Eigen::MatrixBase<DerivedForce>& force,
    const Eigen::MatrixBase<DerivedNormal>& normal,
    const Eigen::MatrixBase<DerivedPoint>& point_on_contact_plane) {
  // TODO(tkoolen): implement multi-column version
  using namespace Eigen;

  if (abs(normal.squaredNorm() - 1.0) > 1e-12) {
    throw std::runtime_error(
        "Drake:resolveCenterOfPressure:BadInputs: normal should be a unit "
        "vector");
  }

  Vector3d cop;
  double normal_torque_at_cop;

  double fz = normal.dot(force);
  bool cop_exists = abs(fz) > 1e-12;

  if (cop_exists) {
    auto torque_at_point_on_contact_plane =
        torque - point_on_contact_plane.cross(force);
    double normal_torque_at_point_on_contact_plane =
        normal.dot(torque_at_point_on_contact_plane);
    auto tangential_torque = torque_at_point_on_contact_plane -
                             normal * normal_torque_at_point_on_contact_plane;
    cop = normal.cross(tangential_torque) / fz + point_on_contact_plane;
    auto torque_at_cop = torque - cop.cross(force);
    normal_torque_at_cop = normal.dot(torque_at_cop);
  } else {
    cop.setConstant(std::numeric_limits<double>::quiet_NaN());
    normal_torque_at_cop = std::numeric_limits<double>::quiet_NaN();
  }
  return std::pair<Vector3d, double>(cop, normal_torque_at_cop);
}

template DRAKE_EXPORT std::pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>,
                                    double>
resolveCenterOfPressure<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const,
                                   0, Eigen::Stride<0, 0> >,
                        Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const,
                                   0, Eigen::Stride<0, 0> >,
                        Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const,
                                   0, Eigen::Stride<0, 0> >,
                        Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const,
                                   0, Eigen::Stride<0, 0> > >(
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0,
                                 Eigen::Stride<0, 0> > > const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0,
                                 Eigen::Stride<0, 0> > > const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0,
                                 Eigen::Stride<0, 0> > > const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, 1, 0, 3, 1> const, 0,
                                 Eigen::Stride<0, 0> > > const&);
template DRAKE_EXPORT
    std::pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>, double>
    resolveCenterOfPressure<Eigen::Matrix<double, 3, 1, 0, 3, 1>,
                            Eigen::Matrix<double, 3, 1, 0, 3, 1>,
                            Eigen::Matrix<double, 3, 1, 0, 3, 1>,
                            Eigen::Matrix<double, 3, 1, 0, 3, 1> >(
        Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&,
        Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&,
        Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&,
        Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&);
template DRAKE_EXPORT
    std::pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>, double>
    resolveCenterOfPressure<
        Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false>,
        Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false>,
        Eigen::Matrix<double, 3, 1, 0, 3, 1>,
        Eigen::Matrix<double, 3, 1, 0, 3, 1> >(
        Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3,
                                       1, false> > const&,
        Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3,
                                       1, false> > const&,
        Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&,
        Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1> > const&);
