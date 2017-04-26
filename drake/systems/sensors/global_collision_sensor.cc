#include "drake/systems/sensors/global_collision_sensor.h"

#include <utility>
#include <vector>

#include <Eigen/Dense>

using Eigen::VectorXd;

using std::make_unique;
using std::string;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace sensors {

GlobalCollisionSensor::GlobalCollisionSensor(const string& name,
    unique_ptr<RigidBodyTree<double>> tree)
    : name_(name), plant_(std::move(tree)) {
  const RigidBodyTree<double>& t = plant_.get_rigid_body_tree();
  input_port_index_ = DeclareInputPort(kVectorValued,
      t.get_num_positions() + t.get_num_velocities()).get_index();
  output_port_index_ = DeclareAbstractOutputPort().get_index();
}

GlobalCollisionSensor* GlobalCollisionSensor::AttachGlobalCollisionSensor(
    const string& name,
    unique_ptr<RigidBodyTree<double>> tree,
    const OutputPortDescriptor<double>& plant_state_port,
    DiagramBuilder<double>* builder) {
  auto global_collision_sensor =
      builder->template AddSystem<GlobalCollisionSensor>(name, std::move(tree));
  builder->Connect(plant_state_port, global_collision_sensor->get_input_port());
  return global_collision_sensor;
}

std::unique_ptr<AbstractValue> GlobalCollisionSensor::AllocateOutputAbstract(
    const OutputPortDescriptor<double>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_data_type() == kAbstractValued);
  if (descriptor.get_index() == output_port_index_) {
    return make_unique<Value<ContactResults<double>>>(ContactResults<double>());
  }
  DRAKE_ABORT_MSG("Unknown abstract output port.");
  return nullptr;
}

void GlobalCollisionSensor::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  const VectorXd x = this->EvalEigenVectorInput(context, input_port_index_);
  const RigidBodyTree<double>& tree = plant_.get_rigid_body_tree();

  // Computes:
  //
  //  - q:    The RigidBodyPlant's position state vector.
  //  - v:    The RigidBodyPlant's velocity state vector.
  //
  // Note that x = [q, v].
  //
  const auto q = x.head(tree.get_num_positions());
  const auto v = x.tail(tree.get_num_velocities());

  // TODO(liang.fok): Obtain the KinematicsCache directly from the context
  // instead of recomputing it here.
  const KinematicsCache<double> cache = tree.doKinematics(q, v);

  // Updates the contact results output port.
  auto& contact_results =
      output->GetMutableData(output_port_index_)->
          template GetMutableValue<ContactResults<double>>();
  VectorX<double> result = plant_.ComputeContactForce(cache, &contact_results);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
