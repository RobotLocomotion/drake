#pragma once

#include <map>
#include <memory>

#include "drake/automotive/car_vis.h"
#include "drake/common/drake_copyable.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// CarVisApplicator takes as input a PoseVector containing vehicle poses. For
/// each vehicle, it outputs the poses of all visual geometries associated with
/// the vehicle's visualization.
///
/// Prior to instantiating this system's systems::Context and
/// systems::SystemOutput, a CarVis object must be provided for each vehicle in
/// the simulation using AddCarVis().
///
/// This system is stateless and is direct feed-through.
///
/// Input port getters:
///  - get_car_poses_input_port() - Contains a PoseBundle of every vehicle's
///    pose in the world frame (i.e., `X_WM_W` where `W` stands for "world" and
///    `M` stands for "Model"). The vehicle IDs and names contained within this
///    PoseBundle must match the IDs and names contained within the CarVis
///    objects that were supplied via calls to AddCarVis().
///
/// Output port getters:
///  - get_visual_geometry_poses_output_port() - Contains a PoseBundle of visual
///    geometry poses in the world frame.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class CarVisApplicator : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CarVisApplicator)

  CarVisApplicator();
  ~CarVisApplicator() override {}

  /// Returns a descriptor of the input port that contains the vehicle poses in
  /// the form of a PoseBundle.
  const systems::InputPortDescriptor<T>& get_car_poses_input_port() const;

  /// Returns a descriptor of the output port that contains the visual geometry
  /// poses of all vehicle visualizations.
  const systems::OutputPortDescriptor<T>&
  get_visual_geometry_poses_output_port() const;

  /// Adds a CarVis object for a vehicle. The ID returned by CarVis::id() must
  /// be unique among the CarVis objects added to this method. A
  /// std::runtime_error is thrown if the provided CarVis object's ID is a
  /// duplicate of a previously provided CarVis object's ID.
  ///
  /// @pre The context for this system has not been created.
  void AddCarVis(std::unique_ptr<CarVis<T>> vis);

  /// Returns an lcmt_viewer_load_robot message containing the geometries of the
  /// bodies being visualized.
  lcmt_viewer_load_robot get_load_robot_message() const;

  /// Returns the number vehicles being visualized.
  int num_cars() const { return static_cast<int>(visualizers_.size()); }

  /// Returns the total number of poses of bodies being visualized.
  int num_vis_poses() const;

 protected:
  std::unique_ptr<systems::AbstractValue> AllocateOutputAbstract(
      const systems::OutputPortDescriptor<double>& descriptor) const override;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  // The key is the car ID.
  std::map<int, std::unique_ptr<const CarVis<T>>> visualizers_;

  // The key is the car ID and the value is the starting index within the output
  // PoseBundle.
  mutable std::map<int, int> starting_indices_;

  int input_port_index_{};
  int output_port_index_{};
  bool context_allocated_{false};
};

}  // namespace automotive
}  // namespace drake
