#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/idm_planner.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// IdmController -- an IDM (Intelligent Driver Model) planner.
///
/// IDM: Intelligent Driver Model:
///    https://en.wikipedia.org/wiki/Intelligent_driver_model
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::TaylorVarXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
///
/// Inputs:
///   Port 0:
///      @p pose_ego PoseBundle for the ego car.
///   Port 1:
///      @p pose_agent PoseBundle for the traffic cars.
/// Outputs:
///   Port 0:
///      @p vdot_ego linear acceleration of the ego car (scalar) [m/s^2].
template <typename T>
class IdmController : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmController)

  /// Constructor. @p road the pre-defined RoadGeometry.
  IdmController(const maliput::api::RoadGeometry* road);
  ~IdmController() override;

  /// Returns the port to the ego car input subvector.
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::InputPortDescriptor<T>& agent_pose_bundle_input() const;

  // System<T> overrides.
  // The output of this system is an algebraic relation of its inputs.
  bool has_any_direct_feedthrough() const override { return true; }

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override;

  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

 private:
  /// Selects nearest agent pose that is ahead of the ego car.  If there are no
  /// qualifying poses, a pose at an x-value of infinity is returned.
  const Isometry3<T> SelectNearestTargetAhead(
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& agent_poses) const;

  /// Retrieves the current RoadPosition for a given road and PoseVector.
  const maliput::api::RoadPosition GetRoadPosition(const Isometry3<T>& pose)
      const;

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcOutput(const systems::rendering::PoseVector<T>& ego_pose,
                        const maliput::api::RoadPosition& agent_pose,
                        const IdmPlannerParameters<T>& params,
                        DrivingCommand<T>* output) const;

  const maliput::api::RoadGeometry* road_;
};

}  // namespace automotive
}  // namespace drake
