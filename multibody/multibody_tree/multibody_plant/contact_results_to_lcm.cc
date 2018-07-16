#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/systems/framework/value.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

using systems::Context;
using systems::Value;

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const MultibodyPlant<T>& plant)
    : systems::LeafSystem<T>() {
  DRAKE_DEMAND(plant.is_finalized());
  const int body_count = plant.num_bodies();
  const MultibodyTree<T>& model = plant.model();

  body_names_.reserve(body_count);
  using std::to_string;
  for (BodyIndex i{0}; i < body_count; ++i) {
    const Body<T>& body = model.get_body(i);
    body_names_.push_back(body.name() + "(" + to_string(body.model_instance()) +
                          ")");
  }
  this->set_name("ContactResultsToLcmSystem");
  // Must be the first declared input port to be compatible with the constexpr
  // declaration of contact_result_input_port_index_.
  this->DeclareAbstractInputPort(Value<ContactResults<T>>());
  this->DeclareAbstractOutputPort(
      &ContactResultsToLcmSystem::CalcLcmContactOutput);
}

template <typename T>
const systems::InputPort<T>&
ContactResultsToLcmSystem<T>::get_contact_result_input_port() const {
  return this->get_input_port(contact_result_input_port_index_);
}

template <typename T>
const systems::OutputPort<T>&
ContactResultsToLcmSystem<T>::get_lcm_message_output_port() const {
  return this->get_output_port(message_output_port_index_);
}

template <typename T>
void ContactResultsToLcmSystem<T>::CalcLcmContactOutput(
    const Context<T>& context, lcmt_contact_results_for_viz* output) const {
  // Get input / output.
  const auto& contact_results =
      this->EvalAbstractInput(context, contact_result_input_port_index_)
          ->template GetValue<ContactResults<T>>();
  auto& msg = *output;

  // Time in microseconds.
  msg.timestamp = static_cast<int64_t>(
      ExtractDoubleOrThrow(context.get_time()) * 1e6);
  msg.num_contacts = contact_results.num_contacts();
  msg.contact_info.resize(msg.num_contacts);

  for (int i = 0; i < contact_results.num_contacts(); ++i) {
    lcmt_contact_info_for_viz& info_msg = msg.contact_info[i];
    info_msg.timestamp = msg.timestamp;

    const PointPairContactInfo<T>& contact_info =
        contact_results.contact_info(i);

    info_msg.body1_name = body_names_.at(contact_info.bodyA_index());
    info_msg.body2_name = body_names_.at(contact_info.bodyB_index());

    auto write_double3 = [](const Vector3<T>& src, double* dest) {
      dest[0] = ExtractDoubleOrThrow(src(0));
      dest[1] = ExtractDoubleOrThrow(src(1));
      dest[2] = ExtractDoubleOrThrow(src(2));
    };
    write_double3(contact_info.contact_point(), info_msg.contact_point);
    write_double3(contact_info.contact_force(), info_msg.contact_force);
    write_double3(contact_info.point_pair().nhat_BA_W, info_msg.normal);
  }
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

// This should be kept in sync with the scalars that MultibodyPlant supports.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::multibody_plant::ContactResultsToLcmSystem)
