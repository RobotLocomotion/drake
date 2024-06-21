#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace visualization {

// TODO(rcory): Support MeshCat gamepad as an additional input source.

/** %MeshcatPoseSliders adds slider bars to the MeshCat control panel for the
roll, pitch, yaw, x, y, z control of a pose. These might be useful for
interactive or teleoperation demos. The abstract-value pose is available on the
`pose` output port of this system.

@system
name: MeshcatPoseSliders
input_ports:
- pose (optional)
output_ports:
- pose
@endsystem

The optional `pose` input port is used ONLY at initialization; it can be
used to set the pose during an Initialization event. If connected, it will take
precedence over any @p initial_pose specified in the constructor.

Beware that the output port of this system always provides the sliders' current
values, even if evaluated by multiple different downstream input ports during a
single computation.  If you need to have a synchronized view of the slider
data, place a systems::ZeroOrderHold system between the sliders and downstream
calculations.

@tparam_nonsymbolic_scalar
@ingroup visualization */
template <typename T>
class MeshcatPoseSliders final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatPoseSliders);

  /** Creates MeshCat sliders for a roll-pitch-yaw-x-y-z parameterization of a
  pose (aka RigidTransform).

  @param meshcat The MeshCat instance where the sliders will be added.

  @param initial_pose (Optional) If provided, the sliders' initial values will
  be as given.

  @param lower_limit (Optional) The lower limit of the sliders for roll, pitch,
  yaw, x, y, z.

  @param upper_limit (Optional) The upper limit of the sliders for roll, pitch,
  yaw, x, y, z.

  @param step (Optional) The step argument of the slider, which is the smallest
  increment by which the slider can change values (and therefore update our
  output port's value). May be a single value or else a vector of length 6.

  @param decrement_keycodes (Optional) A vector of length 6 with keycodes to
  assign to decrement the value of each slider (roll, pitch, yaw, x, y, z). See
  Meshcat::AddSlider for more details.

  @param increment_keycodes (Optional) A vector of length 6 with keycodes to
  assign to increment the value of each slider (roll, pitch, yaw, x, y, z). See
  Meshcat::AddSlider for more details.

  @param prefix (Optional) By default, the sliders will be named "roll, pitch,
  yaw, x, y, z". If prefix is non-empty, the the sliders will be named
  "{prefix}_roll, {prefix}_pitch", etc.

  @param visible (Optional) If provided, determines which of the sliders (roll,
  pitch, yaw, x, y, z) are visible in the Meshcat window. This can be useful,
  for instance, if one only wants to control the pose in 2D (and therefore only
  show the sliders for e.g. yaw, x, and y).

  The default keycodes provide the following keyboard mapping:
  @verbatim
  ┌───────┬┬───────┬┬───────┐        ┌───────┬┬───────┬┬───────┐
  │   Q   ││   W   ││   E   │        │   U   ││   I   ││   O   │
  │ roll- ││ pitch+││ roll+ │        │   z-  ││   y+  ││   z+  │
  ├───────┼┼───────┼┼───────┤        ├───────┼┼───────┼┼───────┤
  ├───────┼┼───────┼┼───────┤        ├───────┼┼───────┼┼───────┤
  │   A   ││   S   ││   D   │        │   J   ││   K   ││   L   │
  │  yaw- ││ pitch-││  yaw+ │        │   x-  ││   y-  ││   x+  │
  └───────┴┴───────┴┴───────┘        └───────┴┴───────┴┴───────┘
  @endverbatim

  */
  MeshcatPoseSliders(
      std::shared_ptr<geometry::Meshcat> meshcat,
      const math::RigidTransform<double>& initial_pose =
          math::RigidTransform<double>(),
      const Eigen::Ref<const Vector6d>& lower_limit =
          (Vector6d() << -M_PI, -M_PI, -M_PI, -1, -1, -1).finished(),
      const Eigen::Ref<const Vector6d>& upper_limit =
          (Vector6d() << M_PI, M_PI, M_PI, 1, 1, 1).finished(),
      const Eigen::Ref<const Vector6d>& step = Vector6d::Constant(0.01),
      std::vector<std::string> decrement_keycodes = {"KeyQ", "KeyS", "KeyA",
                                                     "KeyJ", "KeyK", "KeyU"},
      std::vector<std::string> increment_keycodes = {"KeyE", "KeyW", "KeyD",
                                                     "KeyL", "KeyI", "KeyO"},
      std::string prefix = "",
      const Eigen::Ref<const Vector6<bool>>& visible =
          Vector6<bool>::Constant(true));

  /** Removes our sliders from the associated meshcat instance.

  From this point on, our output port produces the initial_value from the
  constructor, not any slider values.

  @warning It is not safe to call this when a CalcOutput might be happening on
  this instance concurrently on another thread. */
  void Delete();

  /** Performs a Delete(), if one has not already been done. */
  ~MeshcatPoseSliders() final;

  /** Publishes the given systems::System (typically, a Diagram including
  visualizers, to cause it to be visualized) whenever our sliders' values
  change.  Blocks until the user clicks a "Stop" button in the MeshCat control
  panel, or if the timeout limit is reached.

  @param context The systems::Context for the systems::Diagram.

  @param timeout (Optional) In the absence of a button click, the duration (in
  seconds) to wait before returning. If the button is clicked, this function
  will return promptly, without waiting for the timeout. When no timeout is
  given, this function will block indefinitely.

  @param stop_button_keycode a keycode that will be assigned to the "Stop"
  button.  Setting this to the empty string means no keycode. See
  Meshcat::AddButton for details. @default "Escape".

  @returns the most recently output pose.

  @pre `system` must be this MeshcatPoseSliders system or be a top-level (i.e.,
  "root") diagram that contains this MeshcatPoseSliders system.
  */
  math::RigidTransform<double> Run(
      const systems::System<T>& system, const systems::Context<T>& context,
      std::optional<double> timeout = std::nullopt,
      std::string stop_button_keycode = "Escape") const;

  /** Sets the pose to `X`.  The MeshCat sliders will have their value updated.
  Additionally, the "initial pose" tracked by this instance will be updated to
  `X`.  This "initial pose" update will persist even if sliders are removed
  (e.g., via Delete).
  */
  void SetPose(const math::RigidTransform<double>& X) const;

 private:
  void CalcOutput(const systems::Context<T>&,
                  math::RigidTransform<double>*) const;

  /* Handles the initialization event. */
  systems::EventStatus OnInitialization(const systems::Context<T>&) const;

  typename systems::LeafSystem<T>::GraphvizFragment DoGetGraphvizFragment(
      const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
      const final;

  std::shared_ptr<geometry::Meshcat> meshcat_;
  mutable math::RigidTransform<double> nominal_pose_;
  std::atomic<bool> is_registered_;

  std::vector<std::string> slider_names_;
  Vector6d lower_limit_;
  Vector6d upper_limit_;
  Vector6<bool> visible_;
};

}  // namespace visualization
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::visualization::MeshcatPoseSliders);
