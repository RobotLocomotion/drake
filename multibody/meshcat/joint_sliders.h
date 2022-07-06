#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace meshcat {

/** %JointSliders adds slider bars to the Meshcat control panel for the joints
of a MultibodyPlant. These might be useful for interactive or teleoperation
demos. The sliders' current values are available on the `positions` output port
of this system.

@system
name: JointSliders
output_ports:
- positions
@endsystem

The output port is of size `plant.num_positions()`, and the order of its
elements matches `plant.GetPositions()`.

At the moment, any positions that are not associated with joints (e.g.,
floating-base "mobilizers") are held fixed at a nominal value.  In the future,
this class might add in sliders for a floating base as well.

Beware that the output port of this system always provides the sliders' current
values, even if evaluated by multiple different downstream input ports during a
single computation.  If you need to have a synchronized view of the slider data,
place a systems::ZeroOrderHold system between the sliders and downstream
calculations.

@tparam_nonsymbolic_scalar
@ingroup visualization */
template <typename T>
class JointSliders final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointSliders)

  /** Creates a meshcat slider for each joint in the given plant.

  @param meshcat The Meshcat instance where the sliders will be added.

  @param plant The MultibodyPlant to create sliders for.
  The plant pointer is aliased for the lifetime of this %JointSliders object.

  @param initial_value (Optional) If provided, the sliders' initial values
  will be as given; otherwise, the plant's default values will be used.

  @param lower_limit (Optional) The lower limit of the slider will be the
  maximum value of this number and any limit specified in the Joint.
  May be a single value or else a vector of length plant.num_positions().
  If no value is provided, a sensible default will be used.

  @param upper_limit (Optional) The upper limit of the slider will be the
  minimum value of this number and any limit specified in the Joint.
  May be a single value or else a vector of length plant.num_positions().
  If no value is provided, a sensible default will be used.

  @param step (Optional) The step argument of the slider, which is the
  smallest increment by which the slider can change values (and therefore
  update our output port's value).
  May be a single value or else a vector of length plant.num_positions().
  If no value is provided, a sensible default will be used.
  */
  JointSliders(
      std::shared_ptr<geometry::Meshcat> meshcat,
      const MultibodyPlant<T>* plant,
      std::optional<Eigen::VectorXd> initial_value = {},
      std::variant<std::monostate, double, Eigen::VectorXd> lower_limit = {},
      std::variant<std::monostate, double, Eigen::VectorXd> upper_limit = {},
      std::variant<std::monostate, double, Eigen::VectorXd> step = {});

  /** Removes our sliders from the associated meshcat instance.

  From this point on, our output port produces the initial_value from the
  constructor, not any slider values.

  @warning It is not safe to call this when a CalcOutput might be happening on
  this instance concurrently on another thread. */
  void Delete();

  /** Performs a Delete(), if one has not already been done. */
  ~JointSliders() final;

  /** Publishes the given Diagram (typically, to cause it to be visualized)
  whenever our sliders' values change.  Blocks until the user clicks a "Stop"
  button in the MeshCat control panel, or if the timeout limit is reached.

  @param timeout (Optional) In the absence of a button click, the duration (in
  seconds) to wait before returning. If the button is clicked, this function
  will return promptly, without waiting for the timeout. When no timeout is
  given, this function will block indefinitely.

  @pre `diagram` must be a top-level (i.e., "root") diagram.
  @pre `diagram` must contain this JointSliders system.
  @pre `diagarm` must contain the `plant` that was passed into this
  JointSliders system's constructor.
  */
  void Run(
      const systems::Diagram<T>& diagram,
      std::optional<double> timeout = std::nullopt) const;

  /** Sets all robot positions (corresponding to joint positions and potentially
  positions not associated with any joint) to the values in `q`.  The meshcat
  sliders associated with any joint positions described by `q` will have their
  value updated.  Additionally, the "initial state" vector of positions tracked
  by this instance will be updated to the values in `q`.  This "initial state"
  vector update will persist even if sliders are removed (e.g., via Delete).

  @param q A vector whose length is equal to the associated
  MultibodyPlant::num_positions().
  */
  void SetPositions(const Eigen::VectorXd& q);

 private:
  void CalcOutput(const systems::Context<T>&, systems::BasicVector<T>*) const;

  std::shared_ptr<geometry::Meshcat> meshcat_;
  const MultibodyPlant<T>* const plant_;
  const std::map<int, std::string> position_names_;
  Eigen::VectorXd initial_value_;
  std::atomic<bool> is_registered_;
};

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::JointSliders)
