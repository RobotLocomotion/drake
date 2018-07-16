#include "drake/systems/controllers/inverse_dynamics.h"

#include <vector>

#include "drake/multibody/rigid_body_tree.h"

using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::VelocityKinematicsCache;

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
InverseDynamics<T>::InverseDynamics(const RigidBodyTree<T>& tree,
                                    bool pure_gravity_compensation)
    : rigid_body_tree_(&tree),
      pure_gravity_compensation_(pure_gravity_compensation),
      q_dim_(tree.get_num_positions()),
      v_dim_(tree.get_num_velocities()),
      act_dim_(tree.get_num_actuators()) {
  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_torque_ =
      this->DeclareVectorOutputPort(BasicVector<T>(act_dim_),
                                    &InverseDynamics<T>::CalcOutputTorque)
          .get_index();

  // Doesn't declare desired acceleration input port if we are only doing
  // gravity compensation.
  if (!pure_gravity_compensation_) {
    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }

  if (v_dim_ != act_dim_) {
    std::stringstream msg;
    msg << "The model is under-actuated!\n"
        << "  - size of gravity vector: " << v_dim_ << "\n"
        << "  - number of actuators: " << act_dim_;
    throw std::runtime_error(msg.str().c_str());
  }
}

template <typename T>
InverseDynamics<T>::InverseDynamics(const MultibodyPlant<T>& plant,
                                    const Context<T>& multibody_plant_context,
                                    bool pure_gravity_compensation)
    : multi_body_plant_(&plant),
      pure_gravity_compensation_(pure_gravity_compensation),
      q_dim_(plant.model().num_positions()),
      v_dim_(plant.model().num_velocities()),
      act_dim_(plant.model().num_actuators()) {
  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_torque_ =
      this->DeclareVectorOutputPort(BasicVector<T>(act_dim_),
                                    &InverseDynamics<T>::CalcOutputTorque)
          .get_index();

  // TODO(edrumwri): Replace this with multibody_plant_context.Clone() when
  // Issue #9118 has been addressed.
  multibody_plant_context_ = plant.CreateDefaultContext();
  multibody_plant_context_->get_mutable_state().SetFrom(
      multibody_plant_context.get_state());
  multibody_plant_context_->get_mutable_parameters().SetFrom(
      multibody_plant_context.get_parameters());

  // Doesn't declare desired acceleration input port if we are only doing
  // gravity compensation.
  if (!pure_gravity_compensation_) {
    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }

  if (v_dim_ != act_dim_) {
    std::stringstream msg;
    msg << "The model is under-actuated!\n"
        << "  - size of gravity vector: " << v_dim_ << "\n"
        << "  - number of actuators: " << act_dim_;
    throw std::runtime_error(msg.str().c_str());
  }
}

template <typename T>
void InverseDynamics<T>::CalcOutputTorque(const Context<T>& context,
                                          BasicVector<T>* output) const {
  // State input.
  VectorX<T> x = this->EvalEigenVectorInput(context, input_port_index_state_);

  // Desired acceleration input.
  VectorX<T> desired_vd = VectorX<T>::Zero(v_dim_);

  if (!pure_gravity_compensation_) {
    // Only eval acceleration input port when we are not in pure gravity
    // compensation mode.
    desired_vd = this->EvalEigenVectorInput(
        context, input_port_index_desired_acceleration_);
  } else {
    // Sets velocity to zero in pure gravity compensation.
    x.tail(v_dim_).setZero();
  }

  if (rigid_body_tree_) {
    KinematicsCache<T> cache = rigid_body_tree_->CreateKinematicsCache();
    cache.initialize(x.head(q_dim_), x.tail(v_dim_));
    rigid_body_tree_->doKinematics(cache, true);

    eigen_aligned_std_unordered_map<RigidBody<T> const*, drake::TwistVector<T>>
        f_ext;

    VectorX<T> torque = rigid_body_tree_->inverseDynamics(
        cache, f_ext, desired_vd,
        !pure_gravity_compensation_ /* include v dependent terms */);

    DRAKE_ASSERT(torque.size() == output->size());
    output->get_mutable_value() = torque;
  } else {
    DRAKE_DEMAND(multi_body_plant_);

    // Set the position and velocity in the context.
    multibody_plant_context_->get_mutable_continuous_state().
        get_mutable_generalized_position().SetFromVector(x.head(q_dim_));
    multibody_plant_context_->get_mutable_continuous_state().
        get_mutable_generalized_velocity().SetFromVector(x.tail(v_dim_));

    // Compute the caches.
    const auto& tree = multi_body_plant_->model();
    PositionKinematicsCache<T> pcache(tree.get_topology());
    VelocityKinematicsCache<T> vcache(tree.get_topology());
    tree.CalcPositionKinematicsCache(*multibody_plant_context_, &pcache);
    tree.CalcVelocityKinematicsCache(*multibody_plant_context_, pcache,
                                     &vcache);

    // Compute inverse dynamics.
    VectorX<T> tau_applied = VectorX<T>::Zero(
        tree.num_velocities());
    std::vector<multibody::SpatialAcceleration<T>> A_WB_array(
        tree.num_bodies());
    std::vector<multibody::SpatialForce<T>> F_BMo_W_array(tree.num_bodies());
    for (auto& f : F_BMo_W_array)
      f.SetZero();
    VectorX<T> tau_array(tree.num_velocities());

    tree.CalcInverseDynamics(*multibody_plant_context_, pcache, vcache,
        desired_vd, {}, tau_applied, &A_WB_array, &F_BMo_W_array, &tau_array);

    DRAKE_ASSERT(tau_array.size() == output->size());
    output->get_mutable_value() = tau_array;
  }
}

template class InverseDynamics<double>;
// TODO(siyuan) template on autodiff.
// template class InverseDynamics<AutoDiffXd>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
