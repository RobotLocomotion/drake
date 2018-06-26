#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"

#include <memory>

#include "drake/lcmt_contact_results_for_viz.hpp"
//#include "drake/util/drakeUtil.h"
#include "drake/systems/framework/value.h"

#include <iostream>
//#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VAR(a) (void) a;

namespace drake {
namespace multibody {
namespace multibody_plant {

using Eigen::Map;
using systems::Context;
using systems::Value;

template <typename T>
ContactResultsToLcmSystem<T>::ContactResultsToLcmSystem(
    const MultibodyPlant<T>& plant) : plant_(plant) {
  set_name("ContactResultsToLcmSystem");
  DeclareAbstractInputPort(Value<ContactResults<T>>());
  DeclareAbstractOutputPort(&ContactResultsToLcmSystem::CalcLcmContactOutput);
}

template <typename T>
void ContactResultsToLcmSystem<T>::CalcLcmContactOutput(
    const Context<T>& context, lcmt_contact_results_for_viz* output) const {
  // Get input / output.
  const auto& contact_results =
      EvalAbstractInput(context, 0)->template GetValue<ContactResults<T>>();
  auto& msg = *output;

  msg.timestamp = static_cast<int64_t>(context.get_time() * 1e6);
  msg.num_contacts = contact_results.num_contacts();
  msg.contact_info.resize(msg.num_contacts);

  const MultibodyTree<T>& model = plant_.model();

  PRINT_VAR(__PRETTY_FUNCTION__);

  for (int i = 0; i < contact_results.num_contacts(); ++i) {
    lcmt_contact_info_for_viz& info_msg = msg.contact_info[i];
    info_msg.timestamp = static_cast<int64_t>(context.get_time() * 1e6);

    const PointPairContactInfo<T>& contact_info =
        contact_results.contact_info(i);

    info_msg.body1_name = model.get_body(contact_info.bodyA_index()).name();
    info_msg.body2_name = model.get_body(contact_info.bodyB_index()).name();

    Map<Vector3<T>>(info_msg.contact_point) = contact_info.contact_point();

    Map<Vector3<T>>(info_msg.contact_force) = contact_info.contact_force();
    Map<Vector3<T>>(info_msg.normal) = contact_info.point_pair().nhat_BA_W;

    PRINT_VAR(info_msg.body1_name);
    PRINT_VAR(info_msg.body2_name);
    PRINT_VAR(contact_info.contact_point().transpose());
    PRINT_VAR(contact_info.contact_force().transpose());
    PRINT_VAR(contact_info.point_pair().nhat_BA_W.transpose());


  //    eigenVectorToCArray(contact_force.get_normal(), info_msg.normal);
  }
}

template class ContactResultsToLcmSystem<double>;

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
