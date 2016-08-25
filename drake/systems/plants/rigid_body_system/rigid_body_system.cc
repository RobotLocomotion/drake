#include "drake/systems/plants/rigid_body_system/rigid_body_system.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_state_vector.h"
// TODO(amcastro-tri): parsers are not "plants" and should therefore be moved
// somewhere else. Maybe inside "multibody_dynamics/parsers" when that exists.
#include "drake/systems/plants/parser_urdf.h"
#include "drake/common/eigen_autodiff_types.h"

using std::move;
using std::string;

using drake::parsers::ModelInstanceIdTable;

namespace drake {
namespace systems {

template <typename T>
RigidBodySystem<T>::RigidBodySystem(std::unique_ptr<const RigidBodyTree> mbd_world) :
    mbd_world_(move(mbd_world)) {

  // The input to the system is the generalized forces on the actuators.
  System<T>::DeclareInputPort(
      kVectorValued, get_num_actuators(), kContinuousSampling);
  // The output to the system is the state vector.
  System<T>::DeclareOutputPort(
      kVectorValued, get_num_states(), kContinuousSampling);

}

template <typename T>
RigidBodySystem<T>::~RigidBodySystem() { }

template <typename T>
bool RigidBodySystem<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
const RigidBodyTree& RigidBodySystem<T>::get_multibody_world() const {
  return *mbd_world_.get();
}

template <typename T>
int RigidBodySystem<T>::get_num_generalized_positions() const {
  return mbd_world_->number_of_positions();
}

template <typename T>
int RigidBodySystem<T>::get_num_generalized_velocities() const {
  return mbd_world_->number_of_velocities();
}

template <typename T>
int RigidBodySystem<T>::get_num_states() const {
  return get_num_generalized_positions() + get_num_generalized_velocities();
}

template <typename T>
int RigidBodySystem<T>::get_num_actuators() const {
  return mbd_world_->actuators.size();
}

template <typename T>
int RigidBodySystem<T>::get_num_inputs() const {
  return get_num_actuators();
}

template <typename T>
int RigidBodySystem<T>::get_num_outputs() const {
  return get_num_states();
}

template <typename T>
std::unique_ptr<ContinuousState<T>>
RigidBodySystem<T>::AllocateContinuousState() const {
  // The state is second-order.
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() == get_num_actuators());
  // TODO(amcastro-tri): add z state to track energy conservation.
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicStateVector<T>>(get_num_states()),
      get_num_generalized_positions() /* num_q */,
      get_num_generalized_velocities() /* num_v */, 0 /* num_z */);
}

template <typename T>
void RigidBodySystem<T>::EvalOutput(const ContextBase<T>& context,
                                    SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  VectorBase<T>* output_vector = output->GetMutableVectorData(0);

  // TODO(amcastro-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_vector->get_mutable_value() = context.get_xc().CopyToVector();
}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_RBS_EXPORT RigidBodySystem<double>;
//template class DRAKESYSTEMFRAMEWORK_EXPORT Gain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
