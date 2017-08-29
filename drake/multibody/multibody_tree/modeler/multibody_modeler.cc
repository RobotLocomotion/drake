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
void MultibodyModeler<T>::CalcMassMatrixViaInverseDynamics(
    const Context<T>& context, Eigen::Ref<MatrixX<T>> H) const {
  const MultibodyTree<T>& model = get_multibody_tree_model();
  PositionKinematicsCache<T> pc(model.get_topology());
  VelocityKinematicsCache<T> vc(model.get_topology());

  // ======================================================================
  // Compute position kinematics.
  model.CalcPositionKinematicsCache(context, &pc);

  // ======================================================================
  // Compute velocity kinematics.
  model.CalcVelocityKinematicsCache(context, pc, &vc);

  // ======================================================================
  // Compute one column of the mass matrix via inverse dynamics at a time.
  const int nv = model.get_num_velocities();
  VectorX<T> vdot(nv);
  VectorX<T> tau(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(model.get_num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(model.get_num_bodies());

  vdot.setZero();
  for (int j = 0; j < nv; ++j) {
    // TODO(amcastro-tri): make next line to work by making CalcInverseDynamics
    // take an Eigen::Ref<VectorX<T>> instead of a pointer.
    // auto tau = H.col(j);
    if( j != 0) vdot(j-1) = 0.0;
    vdot(j) = 1.0;
    model.CalcInverseDynamics(context, pc, vc, vdot,
                              &A_WB_array, &F_BMo_W_array, &tau);
    H.col(j) = tau;
  }
}

template <typename T>
void MultibodyModeler<T>::CalcBiasTerm(
    const Context<T>& context, Eigen::Ref<VectorX<T>> C) const {
  const MultibodyTree<T>& model = get_multibody_tree_model();
  PositionKinematicsCache<T> pc(model.get_topology());
  VelocityKinematicsCache<T> vc(model.get_topology());

  const int nv = model.get_num_velocities();
  if (C.size() != nv) C.resize(nv);
  C.setZero();

  //const systems::BasicVector<T>& x =
  //    dynamic_cast<const systems::BasicVector<T>&>(
  //        context.get_continuous_state_vector());

  //PRINT_VAR("CalcBiasTerm()");
  //PRINT_VAR(x.CopyToVector().transpose());

  // ======================================================================
  // Compute position kinematics.
  model.CalcPositionKinematicsCache(context, &pc);

  // ======================================================================
  // Compute velocity kinematics.
  model.CalcVelocityKinematicsCache(context, pc, &vc);

  // ======================================================================
  // Compute one column of the mass matrix via inverse dynamics at a time.
  const VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(model.get_num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(model.get_num_bodies());

  // TODO(amcastro-tri): make next line to work by makcing CalcInverseDynamics
  // take an Eigen::Ref<VectorX<T>> instead of a pointer.
  VectorX<T> tau(nv);

  // TODO(amcastro-tri): provide specific API for when vdot = 0.
  model.CalcInverseDynamics(context, pc, vc, vdot,
                            &A_WB_array, &F_BMo_W_array, &tau);
#if 0
  PRINT_VAR(tau.transpose());
  for (const auto& id_link_pair : model_state_.owned_links) {
    LinkId link_id = id_link_pair.first;
    const Link<T>& link = *id_link_pair.second;
    const Body<T>& body = get_link_body(link_id);
    PRINT_VAR(link.get_name());
    PRINT_VAR(A_WB_array[body.get_index()]);
    PRINT_VAR(F_BMo_W_array[body.get_index()]);
    PRINT_VAR(vc.get_V_WB(body.get_node_index()));
  }
#endif
  C = tau;
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
    PRINT_VAR(inboard_body.get_index());
    PRINT_VAR(outboard_body.get_index());

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
          outboard_body, *joint->get_X_CJc());
    } else {
      frame_Jc = &outboard_body.get_body_frame();
    }

    // Create the mobilizer.
    const auto& mobilizer =
        model->AddMobilizer(joint->MakeMobilizer(*frame_Jp, *frame_Jc));
    joint->SetImplementation(&mobilizer);
  }

  model->Finalize();
}


// Explicitly instantiates on the most common scalar types.
template class MultibodyModeler<double>;
template class MultibodyModeler<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
