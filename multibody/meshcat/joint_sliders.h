#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"
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

If the plant has constraints (@see AddMultibodyPlantConstraints), then a
MathematicalProgram is used to resolve those constraints. The sliders in the
gui will be updated (via a periodic or forced publish event) to reflect the
resolved values. There is one nuance: the slider values in the gui only take
values that are integer multiples of the slider step size; the constraint
resolver solves to normal precision and publishes the nearest rounded values.

@tparam_nonsymbolic_scalar
@ingroup visualization */
template <typename T>
class JointSliders final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointSliders);

  /** Creates a meshcat slider for each joint in the given plant.

  @param meshcat The Meshcat instance where the sliders will be added.

  @param plant The MultibodyPlant to create sliders for. The plant pointer is
  aliased for the lifetime of this %JointSliders object.

  @param initial_value (Optional) If provided, the sliders' initial values will
  be as given; otherwise, the plant's default values will be used.

  @param lower_limit (Optional) The lower limit of the slider will be the
  maximum value of this number and any limit specified in the Joint. May be a
  single value or else a vector of length plant.num_positions(). If no value is
  provided, a sensible default will be used.

  @param upper_limit (Optional) The upper limit of the slider will be the
  minimum value of this number and any limit specified in the Joint. May be a
  single value or else a vector of length plant.num_positions(). If no value is
  provided, a sensible default will be used.

  @param step (Optional) The step argument of the slider, which is the smallest
  increment by which the slider can change values (and therefore update our
  output port's value). May be a single value or else a vector of length
  plant.num_positions(). If no value is provided, a sensible default will be
  used.

  @param decrement_keycodes (Optional) A vector of length plant.num_positions()
  with keycodes to assign to decrement the value of each individual joint
  slider. See Meshcat::AddSlider for more details.

  @param increment_keycodes (Optional) A vector of length plant.num_positions()
  with keycodes to assign to increment the value of each individual joint
  slider. See Meshcat::AddSlider for more details.

  @param time_step (Optional). The slider values are updated at discrete
  intervals of this duration.
  */
  JointSliders(
      std::shared_ptr<geometry::Meshcat> meshcat,
      const MultibodyPlant<T>* plant,
      std::optional<Eigen::VectorXd> initial_value = {},
      std::variant<std::monostate, double, Eigen::VectorXd> lower_limit = {},
      std::variant<std::monostate, double, Eigen::VectorXd> upper_limit = {},
      std::variant<std::monostate, double, Eigen::VectorXd> step = {},
      std::vector<std::string> decrement_keycodes = {},
      std::vector<std::string> increment_keycodes = {},
      double time_step = 1 / 32.0);

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

  @param stop_button_keycode a keycode that will be assigned to the "Stop"
  button.  Setting this to the empty string means no keycode. See
  Meshcat::AddButton for details. @default "Escape".

  @returns the output of plant.GetPositions() given the most recently published
  value of the plant Context.

  @pre `diagram` must be a top-level (i.e., "root") diagram.
  @pre `diagram` must contain the `plant` that was passed into this
  JointSliders system's constructor.
  @pre `diagram` must contain this JointSliders system, however the output of
  these sliders need not be connected (even indirectly) to any `plant` input
  port. The positions of the `plant` will be updated directly using a call to
  `plant.SetPositions(...)` when the slider values change.
  */
  Eigen::VectorXd Run(const systems::Diagram<T>& diagram,
                      std::optional<double> timeout = std::nullopt,
                      std::string stop_button_keycode = "Escape") const;

  /** Sets all robot positions (corresponding to joint positions and
  potentially positions not associated with any joint) to the values in `q`.
  The meshcat sliders associated with any joint positions described by `q`
  will have their value updated. This _does not_ update the value of the
  sliders in the context; see the SetPositions() overload which takes a
  Context for that.

  @param q A vector whose length is equal to the associated
  MultibodyPlant::num_positions().
  */
  DRAKE_DEPRECATED("2025-05-01", "Use SetPositions(context, q) instead.")
  void SetPositions(const Eigen::VectorXd& q);

  /** Sets all robot positions (corresponding to joint positions and potentially
  positions not associated with any joint) to the values in `q`.  The meshcat
  sliders associated with any joint positions described by `q` will have their
  value updated.

  @param q A vector whose length is equal to the associated
  MultibodyPlant::num_positions().
  */
  void SetPositions(systems::Context<T>* context, const Eigen::VectorXd& q);

  /** Returns true if the sliders have constraints to solve, not including the
  slider range limits which are enforced automatically. */
  bool has_constraints_to_solve() const { return has_constraints_to_solve_; }

 private:
  std::pair<Eigen::VectorXd, solvers::SolutionResult> ResolveConstraints(
      const Eigen::Ref<const Eigen::VectorXd>& target_values,
      const Eigen::Ref<const Eigen::VectorXd>& previous_values,
      const Eigen::Ref<const Eigen::VectorXd>& initial_guess) const;

  Eigen::VectorXd RoundSliderValues(
      const Eigen::Ref<const Eigen::VectorXd>& values) const;

  systems::EventStatus Update(const systems::Context<T>& context,
                              systems::DiscreteValues<T>* updates) const;

  systems::EventStatus Publish(const systems::Context<T>& context) const;

  typename systems::LeafSystem<T>::GraphvizFragment DoGetGraphvizFragment(
      const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
      const final;

  std::shared_ptr<geometry::Meshcat> meshcat_;
  const MultibodyPlant<T>* const plant_;
  std::shared_ptr<const MultibodyPlant<double>> double_plant_{nullptr};
  const std::map<int, std::string> position_names_;
  Eigen::VectorXd lower_{}, upper_{}, step_{};
  std::atomic<bool> is_registered_;
  bool has_constraints_to_solve_{false};
  systems::DiscreteStateIndex slider_values_index_;
  /* If has_constraints_to_solve_ == true, then we keep an additional copy of
  the positions as state. The slider values are restricted to multiples of the
  step size, but may not satisfy the plant constraints. The constrained_values
  are approximations of the slider values which do satisfy the plant
  constraints. */
  systems::DiscreteStateIndex constrained_values_index_;
  systems::DiscreteStateIndex result_index_;
  solvers::MathematicalProgram prog_;
  solvers::VectorXDecisionVariable q_;
  std::unique_ptr<const solvers::SolverInterface> solver_{};
  std::unique_ptr<systems::Context<double>> double_plant_context_{};
};

}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::meshcat::JointSliders);
