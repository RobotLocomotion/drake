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
InverseDynamics<T>::InverseDynamics(const RigidBodyTree<T>* tree,
                                    bool pure_gravity_compensation)
    : rigid_body_tree_(tree),
      pure_gravity_compensation_(pure_gravity_compensation),
      q_dim_(tree->get_num_positions()),
      v_dim_(tree->get_num_velocities()) {
  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_force_ =
      this->DeclareVectorOutputPort(BasicVector<T>(v_dim_),
                                    &InverseDynamics<T>::CalcOutputForce)
          .get_index();

  // Doesn't declare desired acceleration input port if we are only doing
  // gravity compensation.
  if (!pure_gravity_compensation_) {
    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }
}

template <typename T>
InverseDynamics<T>::InverseDynamics(const MultibodyPlant<T>* plant,
                                    const Parameters<T>& multibody_parameters,
                                    bool pure_gravity_compensation)
    : multibody_plant_(plant),
      pure_gravity_compensation_(pure_gravity_compensation),
      q_dim_(plant->model().num_positions()),
      v_dim_(plant->model().num_velocities()) {
  DRAKE_DEMAND(plant->is_finalized());

  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_force_ =
      this->DeclareVectorOutputPort(BasicVector<T>(v_dim_),
                                    &InverseDynamics<T>::CalcOutputForce)
          .get_index();

  // Copy the parameters.
  multibody_plant_context_ = plant->CreateDefaultContext();
  multibody_plant_context_->get_mutable_parameters().SetFrom(
      multibody_parameters);

  // Doesn't declare desired acceleration input port if we are only doing
  // gravity compensation.
  if (!pure_gravity_compensation_) {
    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }
}

template <typename T>
void InverseDynamics<T>::CalcOutputForce(const Context<T>& context,
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

    VectorX<T> force = rigid_body_tree_->inverseDynamics(
        cache, f_ext, desired_vd,
        !pure_gravity_compensation_ /* include v dependent terms */);

    DRAKE_ASSERT(force.size() == output->size());
    output->get_mutable_value() = force;
  } else {
    DRAKE_DEMAND(multibody_plant_);
    const auto& tree = multibody_plant_->model();

    if (pure_gravity_compensation_) {
      output->get_mutable_value() = tree.CalcGravityGeneralizedForces(
          *multibody_plant_context_);
      return;
    }

    // Set the position and velocity in the context.
    tree.get_mutable_multibody_state_vector(multibody_plant_context_.get()) = x;

    // Compute the caches.
    PositionKinematicsCache<T> pcache(tree.get_topology());
    VelocityKinematicsCache<T> vcache(tree.get_topology());
    tree.CalcPositionKinematicsCache(*multibody_plant_context_, &pcache);
    tree.CalcVelocityKinematicsCache(*multibody_plant_context_, pcache,
                                     &vcache);

    // Compute the contribution from force elements.
    multibody::MultibodyForces<T> external_forces(tree);
    tree.CalcForceElementsContribution(
        *multibody_plant_context_, pcache, vcache, &external_forces);

    // Compute inverse dynamics.
    output->get_mutable_value() = tree.CalcInverseDynamics(
        *multibody_plant_context_, desired_vd, external_forces);
  }
}

template class InverseDynamics<double>;
// TODO(siyuan) template on autodiff.
// template class InverseDynamics<AutoDiffXd>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
