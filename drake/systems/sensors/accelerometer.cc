#include "drake/systems/sensors/accelerometer.h"

#include "drake/math/quaternion.h"
#include "drake/systems/sensors/accelerometer_output.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using std::make_unique;
using std::move;

namespace drake {

using math::quatRotateVec;

namespace systems {
namespace sensors {

constexpr int Accelerometer::kNumMeasurements;

Accelerometer::Accelerometer(const std::string& name,
                             const RigidBodyFrame<double>& frame,
                             const RigidBodyTree<double>& tree,
                             bool include_gravity_compensation)
    : name_(name),
      frame_(frame),
      tree_(tree),
      include_gravity_compensation_(include_gravity_compensation) {
  state_input_port_index_ =
      DeclareInputPort(kVectorValued, tree_.get_num_positions() +
                                      tree_.get_num_velocities()).get_index();
  state_derivative_input_port_index_ =
      DeclareInputPort(kVectorValued, tree_.get_num_positions() +
                                      tree_.get_num_velocities()).get_index();
  output_port_index_ =
      DeclareOutputPort(kVectorValued, kNumMeasurements).get_index();
}

std::unique_ptr<BasicVector<double>> Accelerometer::AllocateOutputVector(
    const OutputPortDescriptor<double>& descriptor) const {
  return std::make_unique<AccelerometerOutput<double>>();
}

void Accelerometer::DoCalcOutput(const systems::Context<double>& context,
                                 systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));

  // Obtains the two input vectors. x is the state vector. vdot is the
  // sacceleration vector.
  const VectorXd x = this->EvalEigenVectorInput(
    context, state_input_port_index_);
  const VectorXd xdot = this->EvalEigenVectorInput(
      context, state_derivative_input_port_index_);

  // Computes:
  //
  //  - q:    The RigidBodyPlant's position state vector.
  //  - v:    The RigidBodyPlant's velocity state vector.
  //  - vdot: The derivative of v, which is the RigidBodyPlant's acceleration
  //          vector.
  //
  // Note that x = [q, v].
  //
  const auto q = x.head(get_tree().get_num_positions());
  const auto v = x.segment(get_tree().get_num_positions(),
                           get_tree().get_num_velocities());
  const auto vdot = xdot.bottomRows(get_tree().get_num_velocities());

  const KinematicsCache<double> kinematics_cache =
      tree_.doKinematics(q, v);

  // The sensor's frame coincides with frame_'s origin.
  const Vector3d sensor_origin = Vector3d::Zero();


  const auto J = tree_.transformPointsJacobian(
      kinematics_cache, sensor_origin, frame_.get_frame_index(),
      RigidBodyTreeConstants::kWorldBodyIndex, false /* in_terms_of_qdot */);


  const auto Jdot_times_v = tree_.transformPointsJacobianDotTimesV(
      kinematics_cache, sensor_origin, frame_.get_frame_index(),
      RigidBodyTreeConstants::kWorldBodyIndex);


  const Vector4d quat_world_to_body = tree_.relativeQuaternion(
      kinematics_cache, RigidBodyTreeConstants::kWorldBodyIndex,
      frame_.get_frame_index());


  const Vector3d accel_base = Jdot_times_v + J * vdot;
  Vector3d accel_body = quatRotateVec(quat_world_to_body, accel_base);

  if (include_gravity_compensation_) {
    const Vector3d gravity(0, 0, 9.81);
    accel_body += quatRotateVec(quat_world_to_body, gravity);
  }


  // Saves the acceleration readings into the output port.
  BasicVector<double>* output_vector =
      output->GetMutableVectorData(output_port_index_);

  output_vector->SetFromVector(accel_body);
}

std::ostream& operator<<(std::ostream& out, const Accelerometer& sensor) {
  out << "Accelerometer:\n"
      << "  - name = " << sensor.get_name() << "\n"
      << "  - frame = " << sensor.get_frame().get_name() << "\n";
  return out;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
