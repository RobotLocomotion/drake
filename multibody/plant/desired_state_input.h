#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/* This struct stores the desired state inputs for all model instances.

 This struct is the result of MultibodyPlant::EvalDesiredStateInput().
 See also @ref pd_controllers_and_ports for further details.

 Only effective inputs are stored: if there is no PD controller or if the PD
 controller is disabled because its joint is locked, then this struct will not
 contain an item for that actuator, even if its desired state input port is
 connected.

 @tparam_default_scalar */
template <typename T>
struct DesiredStateInput {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DesiredStateInput);

  DesiredStateInput();
  ~DesiredStateInput();

  /* A single element in a DesiredStateInput collection, denoting the desired
  state for a single actuator. */
  struct Item {
    JointActuatorIndex actuator_index;
    T qd{};
    T vd{};
  };

  std::vector<Item> items;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::DesiredStateInput);
