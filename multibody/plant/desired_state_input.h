#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/* This class stores the desired state inputs for all model instances.

 Desired states for a given model instance are stored with calls to
 SetModelInstanceDesiredStates(). Model instances with desired states are marked
 as "armed", see is_armed().

 This struct is the result of MultibodyPlant::AssembleDesiredStateInput().
 See also @ref pd_controllers_and_ports for further details.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class DesiredStateInput {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DesiredStateInput);

  /* Constructor for the desired states of `num_model_instances`. */
  explicit DesiredStateInput(int num_model_instances) {
    positions_.resize(num_model_instances);
    velocities_.resize(num_model_instances);
  }

  int num_model_instances() const { return positions_.size(); }

  /* Sets `this` class to store the desired positions `qd` and velocities `vd`
   for the given `model_instance`. Subsequent calls to is_armed() for the given
   model instance will return `true`. */
  void SetModelInstanceDesiredStates(ModelInstanceIndex model_instance,
                                     VectorX<T> qd, VectorX<T> vd) {
    positions_[model_instance] = std::move(qd);
    velocities_[model_instance] = std::move(vd);
  }

  /* Returns `true` if `model_instance` is armed. */
  bool is_armed(ModelInstanceIndex model_instance) const {
    return positions_[model_instance].has_value();
  }

  /* Returns the desired positions for `model_instance`.
   @pre SetModelInstanceDesiredStates() was previously called for
   `model_instance.` */
  const VectorX<T>& positions(ModelInstanceIndex model_instance) const {
    DRAKE_DEMAND(is_armed(model_instance));
    return *positions_[model_instance];
  }

  /* Returns the desired velocities for `model_instance`.
   @pre SetModelInstanceDesiredStates() was previously called for
   `model_instance.` */
  const VectorX<T>& velocities(ModelInstanceIndex model_instance) const {
    DRAKE_DEMAND(is_armed(model_instance));
    return *velocities_[model_instance];
  }

 private:
  std::vector<std::optional<VectorX<T>>> positions_;
  std::vector<std::optional<VectorX<T>>> velocities_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::DesiredStateInput);
