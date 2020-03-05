#pragma once

#include <memory>
#include <optional>
#include <set>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/joint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace util {

/// Given a `MultibodyPlant<double>` and its accompanying
/// `SceneGraph<double>`, this class provides a mechanism for adjusting a
/// `Context<double>` of that plant so that that specificed bodies are no
/// longer colliding.  This is intended for heuristically projecting randomly
/// generated states to similar but collision-free neighbors.
///
/// Because the IK problems involved can be complex, callers with complex
/// scenes will wish to call `AdjustPositions` repeatedly with chosen subsets
/// of the joints and bodies to work the scene out of collision one element at
/// a time.
///
/// This currently works only for `double`.
class CollisionRemover {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CollisionRemover);

  /// Construct a CollisionRemover that can adjust robots out of colliding
  /// positions.
  ///
  /// @p diagram The root system of the simulation
  /// @p plant The MultibodyPlant on which IK will be performed to adjust
  ///    joint and floating body positions
  /// @p scene_graph the SceneGraph used to detect collisions
  CollisionRemover(
      const systems::Diagram<double>* diagram,
      const multibody::MultibodyPlant<double>* plant,
      const geometry::SceneGraph<double>* scene_graph);

  ~CollisionRemover();

  /// Given a @p root_context, try to adjust the position DOFs of @p
  /// joints_to_adjust and @p floating_bodies_to_adjust so that there are no
  /// longer any collisions between the bodies listed in @p
  /// bodies_to_deconflict.
  ///
  /// Callers must provide a @p known_valid_position -- a position that
  /// satisfies the constraints.  This should be the Q value for `plant` in a
  /// valid configuration, eg via:
  /// `plant->GetPositions(diagram->GetSubsystemContext(*plant, context))`
  /// from a known good context value.
  ///
  /// If `bodies_to_deconflict` is set to `nullopt` then uses the context's
  /// pre-existing collision graph.  This is typically clumsy and expensive to
  /// run and can be quite brittle or even nondeterministic; you should run it
  /// only with a small number of DOFs in the `..._to_adjust` parameters.
  ///
  /// @return true on success, ie no collisions remain among the bodies.
  ///
  /// `root_context` may be updated to a less-colliding state even on failure.
  bool AdjustPositions(
      systems::Context<double>* root_context,
      const std::set<multibody::JointIndex>& joints_to_adjust,
      const std::set<multibody::BodyIndex>& floating_bodies_to_adjust,
      const std::optional<const std::set<multibody::BodyIndex>>&
      bodies_to_deconflict,
      const VectorX<double>& known_valid_position);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
