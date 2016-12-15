#include "drake/systems/sensors/depth_sensor.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/sensors/depth_sensor_output.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::make_unique;
using std::move;

namespace drake {
namespace systems {
namespace sensors {

constexpr double DepthSensor::kError;
constexpr double DepthSensor::kTooFar;
constexpr double DepthSensor::kTooClose;

DepthSensor::DepthSensor(const std::string& name,
                         const RigidBodyTree<double>& tree,
                         const RigidBodyFrame<double>& frame, double min_theta,
                         double max_theta, double min_phi, double max_phi,
                         int num_theta_values, int num_phi_values,
                         double min_range, double max_range)
    : DepthSensor(name, tree, frame,
                  DepthSensorSpecification(
                      min_theta, max_theta, min_phi, max_phi, num_theta_values,
                      num_phi_values, min_range, max_range)) {}

DepthSensor::DepthSensor(const std::string& name,
                         const RigidBodyTree<double>& tree,
                         const RigidBodyFrame<double>& frame,
                         const DepthSensorSpecification& specification)
    : name_(name),
      tree_(tree),
      frame_(frame),
      specification_(specification),
      raycast_endpoints_(std::make_unique<Matrix3Xd>()) {
  DRAKE_DEMAND(specification_.min_theta() <= specification_.max_theta() &&
               "min_theta must be less than or equal to max_theta.");
  DRAKE_DEMAND(specification_.min_phi() <= specification_.max_phi() &&
               "min_phi must be less than or equal to max_phi.");
  DRAKE_DEMAND(specification_.min_phi() >= -M_PI / 2 &&
               "min_phi must be greater than or equal to -M_PI/2.");
  DRAKE_DEMAND(specification_.max_phi() <= M_PI / 2 &&
               "min_phi must be less than or equal to M_PI/2.");
  if (specification_.min_theta() == specification_.max_theta()) {
    DRAKE_DEMAND(specification_.num_theta_values() == 1 &&
                 "num_theta_values must equal 1.");
  } else {
    DRAKE_DEMAND(specification_.num_theta_values() >= 2 &&
                 "num_theta_values must be greater than or equal to 2.");
  }
  if (specification_.min_phi() == specification_.max_phi()) {
    DRAKE_DEMAND(specification_.num_phi_values() == 1 &&
                 "num_phi_values must equal 1.");
  } else {
    DRAKE_DEMAND(specification_.num_phi_values() >= 2 &&
                 "num_phi_values must be greater than or equal to 2.");
  }
  DRAKE_DEMAND(specification_.min_range() <= specification_.max_range() &&
               "min_range must be less than or equal to max_range");
  state_input_port_id_ =
      DeclareInputPort(kVectorValued,
                       tree.get_num_positions() + tree.get_num_velocities())
          .get_index();
  state_output_port_id_ =
      DeclareOutputPort(kVectorValued, get_num_depth_readings()).get_index();
  CacheRaycastEndpoints();
}

void DepthSensor::CacheRaycastEndpoints() {
  raycast_endpoints_->resize(3, get_num_depth_readings());

  // TODO(liang.fok) Optimize the following code by eliminating duplicate end
  // points. Currently, identical end points can occur when phi = PI / 2 or
  // -PI / 2 or when theta > 2 * PI.
  for (int i = 0; i < get_num_pixel_rows(); ++i) {
    double phi = specification_.min_phi() + i * specification_.phi_increment();

    // If this is the top-most row, set the phi equal to max_phi_. This is
    // necessary to account for small inaccuracies due to floating point
    // arithmetic.
    if (i == get_num_pixel_rows() - 1) phi = specification_.max_phi();

    for (int j = 0; j < get_num_pixel_cols(); ++j) {
      double theta =
          specification_.min_theta() + j * specification_.theta_increment();

      // If this is the right-most column, set the theta equal to max_theta_.
      // This is necessary to account for small inaccuracies due to floating
      // point arithmetic.
      if (j == get_num_pixel_cols() - 1) theta = specification_.max_theta();

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
      raycast_endpoints_->col(i * get_num_pixel_cols() + j) =
          1.1 * specification_.max_range() * Vector3<double>(x, y, z);
    }
  }
}

std::unique_ptr<SystemOutput<double>> DepthSensor::AllocateOutput(
    const Context<double>& context) const {
  auto output = make_unique<LeafSystemOutput<double>>();
  auto data = make_unique<DepthSensorOutput<double>>(specification_);
  auto port = make_unique<OutputPort>(move(data));
  output->get_mutable_ports()->push_back(move(port));
  return std::unique_ptr<SystemOutput<double>>(output.release());
}

void DepthSensor::DoCalcOutput(const systems::Context<double>& context,
                               systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));

  VectorXd u = this->EvalEigenVectorInput(context, 0);
  auto q = u.head(tree_.get_num_positions());
  KinematicsCache<double> kinematics_cache = tree_.doKinematics(q);

  const int frame_index = frame_.get_frame_index();

  // Computes the origin of the rays in the world frame.
  Vector3d origin = tree_.transformPoints(
      kinematics_cache,
      Vector3d::Zero() /* The origin of the rays in the sensor's frame. */,
      frame_index, 0 /* to_body_or_frame_ind */);

  // Computes the ends of the casted rays in the world frame. This needs to be
  // done each time this method is called since the sensor may have moved in the
  // world since the last time this method was called.
  //
  // TODO(liang.fok) Save the configuration of the RigidBodyTree branch that
  // holds this sensor. Only re-compute the ends of the casted rays in the world
  // frame if this configuration has changed.
  Matrix3Xd raycast_endpoints_world =
      tree_.transformPoints(kinematics_cache, *raycast_endpoints_, frame_index,
                            0 /* to_body_or_frame_ind */);

  VectorX<double> distances(get_num_depth_readings());

  // TODO(liang.fok) Remove the need for const_cast once
  // RigidBodyTree::collisionRaycast() is made const, see #4592.
  const_cast<RigidBodyTree<double>&>(tree_).collisionRaycast(
      kinematics_cache, origin, raycast_endpoints_world, distances);

  // Applies the min / max range of the sensor. Any measurement that is less
  // than the minimum or greater than the maximum is set to an invalid value.
  // This is so users of this sensor can distinguish between an object at the
  // maximum sensing distance and not detecting any object within the sensing
  // range.
  for (int i = 0; i < distances.size(); ++i) {
    if (distances[i] < 0) {
      // Through experimentation, Liang determined that infinity distance
      // measurements show up as -1.
      if (distances[i] == -1) {
        distances[i] = kTooFar;
      } else {
        drake::log()->warn("Measured distance was < 0 and != -1: " +
                           std::to_string(distances[i]));
        distances[i] = kError;
      }
    } else if (distances[i] > specification_.max_range()) {
      distances[i] = kTooFar;
    } else if (distances[i] < specification_.min_range()) {
      distances[i] = kTooClose;
    }
  }

  // Evaluates the state output port.
  BasicVector<double>* output_vector =
      output->GetMutableVectorData(state_output_port_id_);

  output_vector->SetFromVector(distances);
}

std::ostream& operator<<(std::ostream& out, const DepthSensor& sensor) {
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

  out << "DepthSensor: Raycast endpoints:\n"
      << x_buff.str() << y_buff.str() << z_buff.str();

  return out;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
