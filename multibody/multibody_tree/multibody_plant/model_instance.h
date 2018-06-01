#pragma once

#include <vector>

#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

template <typename T>
class ModelInstance {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModelInstance)

  /// Create a model instance describing a colllection of bodies and joint
  /// actuators to facilitate accessing portions of state and actuation
  /// vectors.  State vectors are populates with positions/velocites for each
  /// body in the order described in @bodies.  Actuation vectors follow the
  /// ordering of the actuators in @p joint_actuators.
  ModelInstance(const std::vector<BodyIndex>& bodies,
                const std::vector<JointActuatorIndex>& joint_actuators);

  /// Scalar-converting copy constructor.  See @ref
  /// system_scalar_conversion.  The new instance will need be
  /// finalised.
  template<typename U>
  ModelInstance(const ModelInstance<U>& other)
      : ModelInstance<T>(other.bodies_, other.joint_actuators_) {}

  ~ModelInstance();

  /// Finalize the ModelInstance using the information in @p tree.  @p
  /// tree must be finalized prior to calling this method.
  void Finalize(const MultibodyTree<T>& tree);

  int num_positions() const { return num_positions_; }
  int num_velocities() const { return num_velocities_; }
  int num_actuated_dofs() const { return num_actuated_dofs_; }

  /// Populates @p q with the subset of @p q_array corresponding to the
  /// positions of the bodies in this model instance.
  ///
  /// @param[in] q_array A vector of generalized positions for the entire
  ///   %MultibodyTree model.
  ///
  /// @param [out] q A pointer to a vector which will contain the generalized
  ///   positions of the bodies in this model instance.
  void get_positions_from_array(
      const Eigen::Ref<const VectorX<T>>& q_array,
      EigenPtr<VectorX<T>> q) const {
    DRAKE_DEMAND(q_array.size() == num_tree_positions_);
    DRAKE_DEMAND(q->size() == num_positions_);
    for (int i = 0; i < num_positions_; ++i) {
      (*q)(i) = q_array(position_map_[i]);
    }
  }

  /// Populates @p v with the subset of @p v_array corresponding to the
  /// velocities of the bodies in this model instance.
  ///
  /// @param[in] v_array A vector of generalized velocities for the entire
  ///   %MultibodyTree model.
  ///
  /// @param [out] v A pointer to a vector which will contain the generalized
  ///   velocities of the bodies in this model instance.
  void get_velocities_from_array(
      const Eigen::Ref<const VectorX<T>>& v_array,
      EigenPtr<VectorX<T>> v) const {
    DRAKE_DEMAND(v_array.size() == num_tree_velocities_);
    DRAKE_DEMAND(v->size() == num_velocities_);
    for (int i = 0; i < num_velocities_; ++i) {
      (*v)(i) = v_array(velocity_map_[i]);
    }
  }

  /// Given the actuation values @p u_a for all actuators in this model
  /// instance, this method sets the actuation vector u for the entire
  /// %MultibodyTree model to which this actuator belongs to.
  ///
  /// @param[in] u_a Actuation values for the actuators. It must be of size
  ///   equal to the number of degrees of freedom of all of the actuated
  ///   Joints in this model instance.
  ///
  /// @param[out] u
  ///   The vector containing the actuation values for the entire MultibodyTree
  ///   model to which the actuators belongs to.
  /// @throws if `u_a.size() != this->joint().num_dofs()`.
  /// @throws if u is nullptr.
  /// @throws if `u.size() != this->num_actuated_dofs()`.
  void set_actuation_vector(
      const Eigen::Ref<const VectorX<T>>& u_a, EigenPtr<VectorX<T>> u) const {
    DRAKE_DEMAND(u_a.size() == num_actuated_dofs_);
    DRAKE_DEMAND(u->size() == num_tree_actuated_dofs_);
    for (int i = 0; i < num_actuated_dofs_; ++i) {
      (*u)(actuator_map_[i]) = u_a(i);
    }
  }

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class ModelInstance;

  int num_positions_{0};
  int num_velocities_{0};
  int num_actuated_dofs_{0};

  int num_tree_positions_{0};
  int num_tree_velocities_{0};
  int num_tree_actuated_dofs_{0};

  const std::vector<BodyIndex> bodies_;
  const std::vector<JointActuatorIndex> joint_actuators_;
  std::vector<int> position_map_;
  std::vector<int> velocity_map_;
  std::vector<int> actuator_map_;
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
