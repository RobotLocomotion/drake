#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace meshcat {

/** Adds one Meshcat slider per joint of the MultibodyPlant.

Any positions that are not associated with joints (e.g., floating-base
"mobilizers") are held fixed at a nominal value.  In the future, this
class might also start to offer sliders for the floating base as well.

@system
name: JointSliders
output_ports:
- positions
@endsystem

Beware that the output port of this system always provides the sliders' current
values, even if evaluated by multiple different downstream input ports during a
single computation.  If you need to have a synchronized view of the slider data,
place a ZeroOrderHold system between the sliders and downstream calculations.

@tparam_nonsymbolic_scalar
@ingroup visualization */
template <typename T>
class JointSliders final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointSliders)

  /** Creates a meshcat slider for each joint in the plant.

  @param meshcat the Meshcat instance where the sliders will be added.

  @param plant the MultibodyPlant to create sliders for.
  This pointer is aliased for the lifetime of this object.

  @param initial_value (Optional) if provided, the sliders' initial values
  will be as given; otherwise, the plant's default values will be used.

  @param lower_limit (Optional) the lower limit of the slider will be the
  maximum value of this number and any limit specified in the Joint.
  May be a single value or else a vector of length plant.num_positions().

  @param upper_limit (Optional) the upper limit of the slider will be the
  minimum value of this number and any limit specified in the Joint.
  May be a single value or else a vector of length plant.num_positions().

  @param step (Optional) the step argument of the slider.
  May be a single value or else a vector of length plant.num_positions().
  */
  JointSliders(
      std::shared_ptr<geometry::Meshcat> meshcat,
      const MultibodyPlant<T>* plant,
      std::optional<Eigen::VectorXd> initial_value = {},
      std::variant<std::monostate, double, Eigen::VectorXd> lower_limit = {},
      std::variant<std::monostate, double, Eigen::VectorXd> upper_limit = {},
      std::variant<std::monostate, double, Eigen::VectorXd> step = {});

  ~JointSliders() final;

 private:
  void CalcOutput(const systems::Context<T>&, systems::BasicVector<T>*) const;

  std::shared_ptr<geometry::Meshcat> meshcat_;
  const MultibodyPlant<T>* const plant_;
  const std::map<int, std::string> position_names_;
  const Eigen::VectorXd initial_value_;
};

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::JointSliders)
