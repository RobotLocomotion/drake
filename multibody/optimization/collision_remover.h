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
namespace multibody {

/// Given a MultibodyPlant, this class provides a mechanism for adjusting
/// a `Context` on that plant to remove collisions.  This is intended for
/// replacing randomly generated states with their nearest collision-free
/// neighbors.
///
/// @warn Empirically this class is very good at finding bugs in collision
/// libraries.  If you get an exception from FCL (the collision library used
/// by our inverse kinematics optimization) please report it to the FCL team
/// at https://github.com/flexible-collision-library/fcl/issues
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
      const drake::systems::Diagram<double>* diagram,
      const drake::multibody::MultibodyPlant<double>* plant,
      const drake::geometry::SceneGraph<double>* scene_graph);

  ~CollisionRemover();

  /// Given a @p root_context, try to adjust the position DOFs of @p
  /// joints_to_adjust and @p floating_bodies_to_adjust to remove collisions
  /// among the bodies listed in @p bodies_to_deconflict.
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
      drake::systems::Context<double>* root_context,
      const std::set<drake::multibody::JointIndex>& joints_to_adjust,
      const std::set<drake::multibody::BodyIndex>& floating_bodies_to_adjust,
      const std::optional<const std::set<drake::multibody::BodyIndex>>&
      bodies_to_deconflict,
      const drake::VectorX<double>& known_valid_position) const;

  /// For convenience (and unit testing), determine whether any of @p bodies
  /// (or, if `nullopt`, anything at all) collide if the plant of this
  /// `CollisionRemover` had context @p plant_context.
  bool Collides(
      const drake::systems::Context<double>& plant_context,
      const std::optional<const std::set<drake::multibody::BodyIndex>>&
      bodies = std::nullopt) const;

 private:
  class Impl;
  const std::unique_ptr<Impl> impl_;
};

}  // namespace multibody
}  // namespace drake
