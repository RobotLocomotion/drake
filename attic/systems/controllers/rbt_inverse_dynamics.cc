#include "drake/systems/controllers/rbt_inverse_dynamics.h"

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {
namespace controllers {
namespace rbt {

template <typename T>
InverseDynamics<T>::InverseDynamics(const RigidBodyTree<T>* tree,
                                    bool pure_gravity_compensation)
    : InverseDynamics(tree, pure_gravity_compensation ?
                      InverseDynamicsMode::kGravityCompensation :
                      InverseDynamicsMode::kInverseDynamics) {}

template <typename T>
InverseDynamics<T>::InverseDynamics(const RigidBodyTree<T>* tree,
                                    const InverseDynamicsMode mode)
    : rigid_body_tree_(tree),
      mode_(mode),
      q_dim_(tree->get_num_positions()),
      v_dim_(tree->get_num_velocities()) {
  DRAKE_DEMAND(tree != nullptr);

  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_force_ =
      this->DeclareVectorOutputPort(BasicVector<T>(v_dim_),
                                    &InverseDynamics<T>::CalcOutputForce)
          .get_index();

  // Doesn't declare desired acceleration input port if we are only doing
  // gravity compensation.
  if (!this->is_pure_gravity_compensation()) {
    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }
}

// We need this in the *.cc file so that rigid_body_tree.h does not need to be
// included by our header file.
template <typename T>
InverseDynamics<T>::~InverseDynamics() = default;

template <typename T>
void InverseDynamics<T>::CalcOutputForce(const Context<T>& context,
                                          BasicVector<T>* output) const {
  // State input.
  VectorX<T> x = this->EvalEigenVectorInput(context, input_port_index_state_);

  // Desired acceleration input.
  VectorX<T> desired_vd = VectorX<T>::Zero(v_dim_);

  if (!this->is_pure_gravity_compensation()) {
    // Only eval acceleration input port when we are not in pure gravity
    // compensation mode.
    desired_vd = this->EvalEigenVectorInput(
        context, input_port_index_desired_acceleration_);
  } else {
    // Sets velocity to zero in pure gravity compensation.
    x.tail(v_dim_).setZero();
  }

  // TODO(jwnimmer-tri) Remove this vestigial level of indentation.
  {
    KinematicsCache<T> cache = rigid_body_tree_->CreateKinematicsCache();
    cache.initialize(x.head(q_dim_), x.tail(v_dim_));
    rigid_body_tree_->doKinematics(cache, true);

    eigen_aligned_std_unordered_map<RigidBody<T> const*, drake::TwistVector<T>>
        f_ext;

    VectorX<T> force = rigid_body_tree_->inverseDynamics(
        cache, f_ext, desired_vd,
        (mode_ == InverseDynamicsMode::kInverseDynamics));

    DRAKE_ASSERT(force.size() == output->size());
    output->get_mutable_value() = force;
  }
}

template class InverseDynamics<double>;

}  // namespace rbt
}  // namespace controllers
}  // namespace systems
}  // namespace drake
