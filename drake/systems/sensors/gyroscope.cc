#include "drake/systems/sensors/gyroscope.h"

#include "drake/math/quaternion.h"
#include "drake/systems/sensors/gyroscope_output.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

using std::make_unique;

namespace drake {
namespace systems {
namespace sensors {

constexpr int Gyroscope::kNumMeasurements;

Gyroscope::Gyroscope(const std::string& name,
                     const RigidBodyFrame<double>& frame,
                     const RigidBodyTree<double>& tree)
    : name_(name),
      frame_(frame),
      tree_(tree) {
  input_port_index_ =
      DeclareInputPort(kVectorValued, tree_.get_num_positions() +
                                      tree_.get_num_velocities()).get_index();
  output_port_index_ =
      DeclareOutputPort(kVectorValued, kNumMeasurements).get_index();
}

std::unique_ptr<BasicVector<double>> Gyroscope::AllocateOutputVector(
    const OutputPortDescriptor<double>& descriptor) const {
  return make_unique<GyroscopeOutput<double>>();
}

void Gyroscope::DoCalcOutput(const systems::Context<double>& context,
                                 systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));

  // Obtains x, the input containing the state vector of the RigidBodyPlant that
  // owns tree_.
  const VectorXd x = this->EvalEigenVectorInput(context, input_port_index_);

  // Computes:
  //
  //  - q:    The RigidBodyPlant's position state vector.
  //  - v:    The RigidBodyPlant's velocity state vector.
  //
  // Note that x = [q, v].
  //
  const auto q = x.head(get_tree().get_num_positions());
  const auto v = x.segment(get_tree().get_num_positions(),
                           get_tree().get_num_velocities());

  const KinematicsCache<double> kinematics_cache =
      tree_.doKinematics(q, v);

  auto relative_twist =
      get_tree().relativeTwist(kinematics_cache,
                               0 /* base_or_frame_ind */,
                               frame_.get_frame_index(),
                               frame_.get_frame_index());

  Eigen::Vector3d angular_rates = relative_twist.head<3>();

  // Saves the acceleration readings into the output port.
  BasicVector<double>* output_vector =
      output->GetMutableVectorData(output_port_index_);
  output_vector->SetFromVector(angular_rates);
}

std::ostream& operator<<(std::ostream& out, const Gyroscope& sensor) {
  out << "Gyroscope:\n"
      << "  - name = " << sensor.get_name() << "\n"
      << "  - frame = " << sensor.get_frame().get_name() << "\n";
  return out;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
