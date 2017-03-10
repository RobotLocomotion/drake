#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"

#include <memory>

#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace systems {

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const RigidBodyTree<T>& tree)
    : tree_(tree) {
  set_name("ContactResultsToLcmSystem");
  DeclareAbstractInputPort(Value<ContactResults<T>>());
  DeclareAbstractOutputPort(Value<lcmt_contact_results_for_viz>());
}

template <typename T>
void ContactResultsToLcmSystem<T>::DoCalcOutput(const Context<T>& context,
                                                SystemOutput<T>* output) const {
  // Get input / output.
  const auto& contact_results =
      EvalAbstractInput(context, 0)->template GetValue<ContactResults<T>>();
  auto& msg = output->GetMutableData(0)
                  ->template GetMutableValue<lcmt_contact_results_for_viz>();

  msg.timestamp = static_cast<int64_t>(context.get_time() * 1e6);
  msg.num_contacts = contact_results.get_num_contacts();
  msg.contact_info.resize(msg.num_contacts);

  for (int i = 0; i < contact_results.get_num_contacts(); ++i) {
    lcmt_contact_info_for_viz& info_msg = msg.contact_info[i];
    info_msg.timestamp = static_cast<int64_t>(context.get_time() * 1e6);

    const ContactInfo<T>& contact_info = contact_results.get_contact_info(i);
    const RigidBody<T>* b1 = tree_.FindBody(contact_info.get_element_id_1());
    const RigidBody<T>* b2 = tree_.FindBody(contact_info.get_element_id_2());

    const ContactForce<T>& contact_force = contact_info.get_resultant_force();

    info_msg.body1_name = b1->get_name();
    info_msg.body2_name = b2->get_name();

    eigenVectorToCArray(contact_force.get_application_point(),
                        info_msg.contact_point);
    eigenVectorToCArray(contact_force.get_force(), info_msg.contact_force);
    eigenVectorToCArray(contact_force.get_normal(), info_msg.normal);
  }
}

template class ContactResultsToLcmSystem<double>;

}  // namespace systems
}  // namespace drake
