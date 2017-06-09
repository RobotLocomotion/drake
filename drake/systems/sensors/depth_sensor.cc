#include "drake/systems/sensors/depth_sensor.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/depth_sensor_output.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::make_unique;
using std::move;

namespace drake {
namespace systems {

using rendering::PoseVector;

namespace sensors {

DepthSensor::DepthSensor(const std::string& name,
                         const RigidBodyTree<double>& tree,
                         const RigidBodyFrame<double>& frame,
                         const DepthSensorSpecification& specification)
    : name_(name), tree_(tree), frame_(frame), specification_(specification) {
  DRAKE_DEMAND(specification_.min_yaw() <= specification_.max_yaw() &&
               "min_yaw must be less than or equal to max_yaw.");
  DRAKE_DEMAND(specification_.min_pitch() <= specification_.max_pitch() &&
               "min_pitch must be less than or equal to max_pitch.");
  if (specification_.min_yaw() == specification_.max_yaw()) {
    DRAKE_DEMAND(specification_.num_yaw_values() == 1 &&
                 "num_yaw_values must equal 1.");
  } else {
    DRAKE_DEMAND(specification_.num_yaw_values() >= 2 &&
                 "num_yaw_values must be greater than or equal to 2.");
  }
  if (specification_.min_pitch() == specification_.max_pitch()) {
    DRAKE_DEMAND(specification_.num_pitch_values() == 1 &&
                 "num_pitch_values must equal 1.");
  } else {
    DRAKE_DEMAND(specification_.num_pitch_values() >= 2 &&
                 "num_pitch_values must be greater than or equal to 2.");
  }
  DRAKE_DEMAND(specification_.min_range() <= specification_.max_range() &&
               "min_range must be less than or equal to max_range");
  input_port_index_ =
      DeclareInputPort(kVectorValued,
                       tree.get_num_positions() + tree.get_num_velocities())
          .get_index();
  depth_output_port_index_ =
      DeclareVectorOutputPort(DepthSensorOutput<double>(specification_),
                              &DepthSensor::CalcDepthOutput)
          .get_index();
  pose_output_port_index_ =
      DeclareVectorOutputPort(PoseVector<double>(),
                              &DepthSensor::CalcPoseOutput)
          .get_index();
  PrecomputeRaycastEndpoints();
}

void DepthSensor::PrecomputeRaycastEndpoints() {
  raycast_endpoints_.resize(3, get_num_depth_readings());

  // TODO(liang.fok) Optimize the following code by eliminating duplicate end
  // points. Currently, identical raycast end points can occur when:
  //
  // (1) pitch = PI / 2
  // (2) pitch = -PI / 2
  // (3) pitch = PI and and pitch = -PI for a given yaw
  // (3) yaw > 2 * PI
  //
  for (int i = 0; i < get_num_pitch(); ++i) {
    double pitch =
        specification_.min_pitch() + i * specification_.pitch_increment();

    // If this is the top-most row, set the pitch equal to max_pitch_. This is
    // necessary to account for small inaccuracies due to floating point
    // arithmetic.
    if (i == get_num_pitch() - 1) pitch = specification_.max_pitch();

    for (int j = 0; j < get_num_yaw(); ++j) {
      double yaw =
          specification_.min_yaw() + j * specification_.yaw_increment();

      // If this is the right-most column, set the yaw equal to max_yaw_.
      // This is necessary to account for small inaccuracies due to floating
      // point arithmetic.
      if (j == get_num_yaw() - 1) yaw = specification_.max_yaw();

      // Compute the location of the raycast end point assuming a max sensing
      // range of one and no occlusions. This is done using the same equations
      // that convert from spherical coordinates to Cartesian coordinates.
      const double x = cos(pitch) * cos(yaw);
      const double y = cos(pitch) * sin(yaw);
      const double z = sin(pitch);

      // The max range is increased by 10% (i.e., multiplied by 1.1) to ensure
      // the range cast end point exceeds the maximum range of the sensor. This
      // is so we can detect when an object is sensed at precisely the maximum
      // range of the sensor.
      raycast_endpoints_.col(i * get_num_yaw() + j) =
          1.1 * specification_.max_range() * Vector3<double>(x, y, z);
    }
  }
}

const InputPortDescriptor<double>&
DepthSensor::get_rigid_body_tree_state_input_port() const {
  return this->get_input_port(input_port_index_);
}

const OutputPort<double>& DepthSensor::get_sensor_state_output_port() const {
  return System<double>::get_output_port(depth_output_port_index_);
}

const OutputPort<double>& DepthSensor::get_pose_output_port() const {
  return System<double>::get_output_port(pose_output_port_index_);
}

// TODO(sherm1) Should be accessing an already-calculated kinematics cache,
// not recalculating.
void DepthSensor::CalcDepthOutput(
    const Context<double>& context,
    DepthSensorOutput<double>* data_output) const {
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
      tree_.transformPoints(kinematics_cache, raycast_endpoints_, frame_index,
                            0 /* to_body_or_frame_ind */);

  VectorX<double> distances(get_num_depth_readings());

  // TODO(liang.fok) Remove the need for const_cast once GeometryWorld and
  // GeometryQuery are introduced. See:
  // https://github.com/RobotLocomotion/drake/issues/4592#issuecomment-269491752
  const_cast<RigidBodyTree<double>&>(tree_).collisionRaycast(
      kinematics_cache, origin, raycast_endpoints_world, distances);

  ApplyLimits(&distances);

  // Evaluates the output port containing the depth measurements.
  data_output->SetFromVector(distances);
}

void DepthSensor::ApplyLimits(VectorX<double>* distances) const {
  VectorX<double>& d = *distances;

  for (int i = 0; i < d.size(); ++i) {
    // Infinity distance measurements show up as -1.
    DRAKE_DEMAND((d[i] >= 0) || (d[i] == -1));
    if (d[i] == -1) {
      d[i] = DepthSensorOutput<double>::GetTooFarDistance();
    } else if (d[i] > specification_.max_range()) {
      d[i] = DepthSensorOutput<double>::GetTooFarDistance();
    } else if (d[i] < specification_.min_range()) {
      d[i] = DepthSensorOutput<double>::GetTooCloseDistance();
    }
  }
}

// Evaluates the output port containing X_WS.
// TODO(sherm1) Should be accessing an already-calculated kinematics cache,
// not recalculating.
void DepthSensor::CalcPoseOutput(const Context<double>& context,
                                 PoseVector<double>* pose_output) const {
  VectorXd u = this->EvalEigenVectorInput(context, 0);
  auto q = u.head(tree_.get_num_positions());
  KinematicsCache<double> kinematics_cache = tree_.doKinematics(q);

  const drake::Isometry3<double> X_WS =
      tree_.CalcFramePoseInWorldFrame(kinematics_cache, frame_);

  pose_output->set_translation(
      Eigen::Translation<double, 3>(X_WS.translation()));
  pose_output->set_rotation(Eigen::Quaternion<double>(X_WS.rotation()));
}

std::ostream& operator<<(std::ostream& out, const DepthSensor& sensor) {
  std::stringstream x_buff, y_buff, z_buff;
  x_buff << "x = [";
  y_buff << "y = [";
  z_buff << "z = [";
  for (int i = 0; i < sensor.raycast_endpoints_.cols(); ++i) {
    if (i != 0) {
      x_buff << ", ";
      y_buff << ", ";
      z_buff << ", ";
    }
    x_buff << sensor.raycast_endpoints_(0, i);
    y_buff << sensor.raycast_endpoints_(1, i);
    z_buff << sensor.raycast_endpoints_(2, i);
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
