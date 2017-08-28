#include "drake/multibody/multibody_tree/modeler/multibody_modeler.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

// The look up and error-throwing method for const values.
template <class Key, class Value>
const Value& GetValueOrThrow(const Key& key,
                             const std::unordered_map<Key, Value>& map) {
  auto itr = map.find(key);
  if (itr != map.end()) {
    return itr->second;
  }
  throw std::logic_error("Key not found in the map.");
}

template <typename T>
const Body<T>& MultibodyModeler<T>::get_link_body(LinkId link_id) const {
  // So far we are assuming there are no ghost bodies and therefore there is
  // only one body per link.
  const std::unordered_set<BodyIndex>& bodies_set =
      GetValueOrThrow(link_id, model_state_.link_id_to_body_index_map);
  DRAKE_DEMAND(bodies_set.size() == 1);
  BodyIndex body_index = *bodies_set.begin();
  DRAKE_DEMAND(model_state_.multibody_tree != nullptr);
  return model_state_.multibody_tree->get_body(body_index);
}

template <typename T>
void MultibodyModeler<T>::MakeMultibodyTreeModel() const {

  // A number of convenient aliases into the model state.
  auto& owned_links = model_state_.owned_links;
  auto& owned_joints = model_state_.owned_joints;
  auto& link_id_to_body_index_map = model_state_.link_id_to_body_index_map;
  auto& body_index_to_link_id_map = model_state_.body_index_to_link_id_map;
  auto& model = model_state_.multibody_tree;

  model = std::move(std::make_unique<MultibodyTree<T>>());

#if 0
  // List of links excluding the world link.
    std::vector<LinkId> links_ids(get_num_links()-1);
    std::copy_if(owned_links.begin(), owned_links.end(),
                 links_ids.begin(),
                 [world_id = model_state_.world_id](LinkId id){
                   return id != world_id;
                 });
#endif

  // Pre-compute the number of bodies.
  // TODO(amcastro-tri): check if a link needs to be split into ghost bodies.
  const int num_bodies = get_num_links();

  PRINT_VAR(num_bodies);

  // Create bodies.
  // TODO(amcastro-tri): check if a link needs to be split into ghost bodies.

  body_index_to_link_id_map.reserve(num_bodies);
  for(const auto& id_link_pair : owned_links) {
    LinkId link_id = id_link_pair.first;
    const auto& link = id_link_pair.second;

    PRINT_VAR(link_id);
    PRINT_VAR(link->get_id());
    PRINT_VAR(link->get_name());
    PRINT_VAR(model_state_.world_id);

    BodyIndex body_index;
    if (link_id == model_state_.world_id) {
      body_index = model->get_world_body().get_index();
    } else {
      body_index = model->AddBody(link->MakeBody()).get_index();
    }
    PRINT_VAR(body_index);

    DRAKE_ASSERT(body_index < num_bodies);
    link_id_to_body_index_map[link_id].insert(body_index);
    body_index_to_link_id_map[body_index] = link_id;
  }
  PRINT_VAR(link_id_to_body_index_map.size());
  DRAKE_ASSERT(
      static_cast<int>(link_id_to_body_index_map.size()) == get_num_links());

  PRINT_VAR("Create Mobilizers");

  // Create Mobilizer objects.
  for(const auto& id_joint_pair : owned_joints) {
    //JointId joint_id = id_joint_pair.first;
    const auto& joint = id_joint_pair.second;

    PRINT_VAR(joint->get_id());
    PRINT_VAR(joint->get_name());
    PRINT_VAR(joint->get_parent_link().get_name());
    PRINT_VAR(joint->get_child_link().get_name());

    // Frame Jp attached on parent link P.
    // So far we are assuming there are no ghost bodies and therefore there is
    // only one body per link.
    const Body<T>& inboard_body =
        get_link_body(joint->get_parent_link().get_id());
    const Body<T>& outboard_body =
        get_link_body(joint->get_child_link().get_id());

    // Define the inboard frame.
    const Frame<T>* frame_Jp;
    if(joint->get_X_PJp()) {
      // Create frame Jp attached to parent body P.
      frame_Jp = &model->template AddFrame<FixedOffsetFrame>(
          inboard_body, *joint->get_X_PJp());
    } else {
      frame_Jp = &inboard_body.get_body_frame();
    }

    // Define the outboard frame.
    const Frame<T>* frame_Jc;
    if(joint->get_X_CJc()) {
      // Create frame Jc attached to child body C.
      frame_Jc = &model->template AddFrame<FixedOffsetFrame>(
          outboard_body, *joint->get_X_PJp());
    } else {
      frame_Jc = &outboard_body.get_body_frame();
    }

    // Create the mobilizer.
    model->AddMobilizer(joint->MakeMobilizer(*frame_Jp, *frame_Jc));
  }

  model->Finalize();
}


// Explicitly instantiates on the most common scalar types.
template class MultibodyModeler<double>;
template class MultibodyModeler<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
