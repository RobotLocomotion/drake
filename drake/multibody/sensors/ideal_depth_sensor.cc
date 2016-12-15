#include "drake/multibody/sensors/ideal_depth_sensor.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_output.h"

#include "drake/common/eigen_matrix_compare.h"  // REMOVE ME!!!!

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::make_unique;
using std::move;

namespace drake {

using systems::BasicVector;
using systems::Context;
using systems::LeafSystemOutput;
using systems::OutputPort;
using systems::SystemOutput;
using systems::kVectorValued;

namespace multibody {

namespace {
// Calculates the number of radians per increment assuming the arc bounded by
// @p max_angle and @p min_angle must be evenly divided by @p num_measurements.
// At a minimum, there must be one measurement at @p max_angle and one
// measurement at @p min_angle.
double CalculateIncrementSize(const double max_angle, const double min_angle,
                              const int num_measurements) {
  DRAKE_DEMAND(min_angle <= max_angle);
  DRAKE_DEMAND(num_measurements >= 1);
  double result = 0;  // This handles the case where max_angle == min_angle.
  if (max_angle != min_angle) {
    // We need to subtract 1 from num_measurements to account for the fact that
    // there are measurements at both the max_angle and min_angle.
    result = (max_angle - min_angle) / (num_measurements - 1);
  }
  return result;
}
}  // namespace

IdealDepthSensor::IdealDepthSensor(const RigidBodyTree<double>& tree,
                                   const std::string& sensor_name,
                                   const RigidBodyFrame<double>& frame,
                                   double min_theta, double max_theta,
                                   double min_phi, double max_phi,
                                   int num_theta_values, int num_phi_values,
                                   double min_range, double max_range)
    : RigidBodyTreeSensor(tree, sensor_name, frame),
      min_theta_(min_theta),
      max_theta_(max_theta),
      min_phi_(min_phi),
      max_phi_(max_phi),
      num_theta_values_(num_theta_values),
      num_phi_values_(num_phi_values),
      num_depth_readings_(num_theta_values * num_phi_values),
      min_range_(min_range),
      max_range_(max_range),
      theta_increment_(
          CalculateIncrementSize(max_theta, min_theta, num_theta_values)),
      phi_increment_(CalculateIncrementSize(max_phi, min_phi, num_phi_values)),
      raycast_endpoints_(std::make_unique<Matrix3Xd>()) {
  DRAKE_DEMAND(min_theta_ <= max_theta_ &&
               "min_theta must be less than or equal to max_theta.");
  DRAKE_DEMAND(min_phi_ <= max_phi_ &&
               "min_phi must be less than or equal to max_phi.");
  DRAKE_DEMAND(min_phi_ >= -M_PI / 2 &&
               "min_phi must be greater than or equal to -M_PI/2.");
  DRAKE_DEMAND(max_phi_ <= M_PI / 2 &&
               "min_phi must be less than or equal to M_PI/2.");
  if (min_theta_ == max_theta_) {
    DRAKE_DEMAND(num_theta_values_ == 1 && "num_theta_values must equal 1.");
  } else {
    DRAKE_DEMAND(num_theta_values_ >= 2 &&
                 "num_theta_values must be greater than or equal to 2.");
  }
  if (min_phi_ == max_phi_) {
    DRAKE_DEMAND(num_phi_values_ == 1 && "num_phi_values must equal 1.");
  } else {
    DRAKE_DEMAND(num_phi_values_ >= 2 &&
                 "num_phi_values must be greater than or equal to 2.");
  }
  DRAKE_DEMAND(min_range_ <= max_range_ &&
               "min_range must be less than or equal to max_range");
  state_input_port_id_ =
      DeclareInputPort(kVectorValued, tree.get_num_states()).get_index();
  state_output_port_id_ =
      DeclareOutputPort(kVectorValued, get_num_depth_readings()).get_index();
  CacheRaycastEndpoints();
}

void IdealDepthSensor::CacheRaycastEndpoints() {
  raycast_endpoints_->resize(3, get_num_depth_readings());

  // TODO(liang.fok) Optimize the following code by eliminating duplicate end
  // points. Currently, identical end points can occur when phi = PI / 2 or
  // -PI / 2 or when theta > 2 * PI.
  for (int i = 0; i < num_pixel_rows(); ++i) {
    double phi = min_phi_ + i * phi_increment_;

    // If this is the top-most row, set the phi equal to max_phi_. This is
    // necessary to account for small inaccuracies due to floating point
    // arithmetic.
    if (i == num_pixel_rows() - 1) phi = max_phi_;

    for (int j = 0; j < num_pixel_cols(); ++j) {
      double theta = min_theta_ + j * theta_increment_;

      // If this is the right-most column, set the theta equal to max_theta_.
      // This is necessary to account for small inaccuracies due to floating
      // point arithmetic.
      if (j == num_pixel_cols() - 1) theta = max_theta_;

      // Compute the location of the raycast end point assuming a max sensing
      // range of one and no occlusions. This is done using the same equations
      // that convert from spherical coordinates to Cartesian coordinates.
      double x = cos(phi) * cos(theta);
      double y = cos(phi) * sin(theta);
      double z = sin(phi);

      // The max range is increased by 10% (i.e., multiplied by 1.1) to ensure
      // the range cast end point exceeds the maximum range of the sensor. This
      // is so we can detect when an object is sensed at precisely the maximum
      // range of the sensor.
      raycast_endpoints_->col(i * num_pixel_cols() + j) =
          1.1 * max_range_ * Vector3<double>(x, y, z);
    }
  }
}

std::unique_ptr<SystemOutput<double>> IdealDepthSensor::AllocateOutput(
    const Context<double>& context) const {
  auto output = make_unique<LeafSystemOutput<double>>();
  auto data = make_unique<BasicVector<double>>(get_num_depth_readings());
  auto port = make_unique<OutputPort>(move(data));
  output->get_mutable_ports()->push_back(move(port));
  return std::unique_ptr<SystemOutput<double>>(output.release());
}

void IdealDepthSensor::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));

  const RigidBodyTree<double>& tree = RigidBodyTreeSensor::get_tree();

  VectorXd u = this->EvalEigenVectorInput(context, 0);
  auto q = u.head(tree.get_num_positions());
  KinematicsCache<double> kinematics_cache = tree.doKinematics(q);

  const int frame_index = RigidBodyTreeSensor::get_frame().get_frame_index();

  // Computes the origin of the rays in the world frame.
  Vector3d origin = tree.transformPoints(
      kinematics_cache,
      Vector3d::Zero() /* The origin of the rays in the sensor's frame. */,
      frame_index, 0 /* to_body_or_frame_ind */);

  // Computes the ends of the casted rays in the world frame. This needs to be
  // done each time this method is called since the sensor may have moved in the
  // world since the last time this method was called.
  //
  // TODO(liang.fok) Store the configuration of the RigidBodyTree branch that
  // holds this sensor. Only re-compute the ends of the casted rays in the world
  // frame if this configuration has changed.
  Matrix3Xd raycast_endpoints_world =
      tree.transformPoints(kinematics_cache, *raycast_endpoints_, frame_index,
                           0 /* to_body_or_frame_ind */);

  VectorX<double> distances(get_num_depth_readings());

  // TODO(liang.fok) Remove the need for const_cast once
  // RigidBodyTree::collisionRaycast() is made const, see #4592.
  const_cast<RigidBodyTree<double>&>(tree).collisionRaycast(
      kinematics_cache, origin, raycast_endpoints_world, distances);

  // Applies the min / max range of the sensor. Any measurement that is less
  // than the minimum or greater than the maximum is set to an invalid value.
  // This is so users of this sensor can distinguish between an object at the
  // maximum sensing distance and not detecting any object within the sensing
  // range.
  for (int i = 0; i < distances.size(); ++i) {
    if (distances[i] < 0.0 || distances[i] > max_range_) {
      distances[i] = kInvalidDepthMeasurement;
    } else if (distances[i] < min_range_) {
      distances[i] = kInvalidDepthMeasurement;
    }
  }

  // Evaluates the state output port.
  BasicVector<double>* output_vector =
      output->GetMutableVectorData(state_output_port_id_);

  output_vector->SetFromVector(distances);
}

std::ostream& operator<<(std::ostream& out, const IdealDepthSensor& sensor) {
  std::stringstream x_buff, y_buff, z_buff;
  x_buff << "x = [";
  y_buff << "y = [";
  z_buff << "z = [";
  for (int i = 0; i < sensor.raycast_endpoints_->cols(); ++i) {
    if (i != 0) {
      x_buff << ", ";
      y_buff << ", ";
      z_buff << ", ";
    }
    x_buff << (*sensor.raycast_endpoints_)(0, i);
    y_buff << (*sensor.raycast_endpoints_)(1, i);
    z_buff << (*sensor.raycast_endpoints_)(2, i);
  }
  x_buff << "]\n";
  y_buff << "]\n";
  z_buff << "]";

  out << "IdealDepthSensor:\n" << x_buff.str() << y_buff.str() << z_buff.str();
  return out;
}

}  // namespace multibody
}  // namespace drake
