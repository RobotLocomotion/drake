#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// CarVis is a base class that provides visualization geometries and their
/// poses.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
///
template <typename T>
class CarVis {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CarVis)

  /// The constructor.
  ///
  /// @param model_instance_id The model instance ID of the model being
  /// visualized.
  ///
  /// @param name The name of the vehicle being visualized. This can be any
  /// user-defined value.
  ///
  CarVis(int model_instance_id, const std::string& name)
      : model_instance_id_(model_instance_id), name_(name) {}

  virtual ~CarVis() {}

  /// Returns the visualization elements.
  virtual const std::vector<lcmt_viewer_link_data>& GetVisElements() const = 0;

  /// Computes and returns the poses of the bodies that are part of the
  /// visualization. The provided `root_pose` is the pose of the model
  /// instance in the world frame. It typically serves as the "root" or
  /// "base frame" of a visualization model. The poses in the returned
  /// PoseBundle are for the visualization model's elements, and are also in the
  /// world frame. The size of this bundle is the value returned by num_poses().
  virtual systems::rendering::PoseBundle<T> GetPoses(
      const Isometry3<T>& root_pose) const = 0;

  /// Returns the model instance ID that was supplied to the constructor.
  int model_instance_id() const { return model_instance_id_; }

  /// Returns the name that was supplied to the constructor.
  const std::string& name() const { return name_; }

  /// Returns the number of visualization poses.
  int num_poses() const;

 private:
  const int model_instance_id_;
  const std::string name_;
};

/// BoxCarVis displays a box for each vehicle.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
///
template <typename T>
class BoxCarVis : public CarVis<T> {
 public:
  BoxCarVis(int model_instance_id, const std::string& name);

  const std::vector<lcmt_viewer_link_data>& GetVisElements() const override;

  systems::rendering::PoseBundle<T> GetPoses(
      const Isometry3<T>& root_pose) const override;

 private:
  std::vector<lcmt_viewer_link_data> vis_elements_;
};

/// CarVisApplicator applies, for each vehicle instance whose pose is included
/// in an input PoseVector, the poses of all bodies belonging to the vehicle
/// instance's visualization.
///
/// Prior to the start of simulation, a CarVis visualizer must be provided for
/// each vehicle instance using AddCarVis().
///
/// This system is stateless and is direct feed-through.
///
/// Input port getters:
///  - get_pose_input_port() - Contains a PoseBundle of vehicle instance poses,
///    in the world frame. The model instance IDs and names contained within
///    this PoseBundle must match the model instance IDs and names contained
///    within the CarVis objects that were supplied vis calls to AddCarVis(). In
///    addition, the PoseBundle must be sorted by increasing model instance IDs
///    starting from zero.
///
/// Output port getters:
///  - get_output_port() - Contains a PoseBundle of body poses. The bodies
///    comprise the visualization.
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

  /// Returns a descriptor of the input port that contains the poses of the
  /// vehicle instances, in the form of a PoseBundle.
  const systems::InputPortDescriptor<T>& get_pose_input_port() const;

  /// Returns a descriptor of the output port that contains the poses of the
  /// bodies that are part of all vehicle instance visualizations.
  const systems::OutputPortDescriptor<T>& get_output_port() const;

  /// Adds a CarVis for a vehicle model instance. The ID of the model instance
  /// is defined by the provided CarVis object using
  /// CarVis::model_instance_id(). CarVis objects must be added in ascending
  /// order by model instance ID starting at zero. A std::runtime_error
  /// exception is thrown if the user attempts to add a CarVis object out of
  /// model instance ID order.
  void AddCarVis(std::unique_ptr<CarVis<T>> vis);

  /// Returns an lcmt_viewer_load_robot message containing the geometries of the
  /// bodies being visualized.
  lcmt_viewer_load_robot get_load_robot_message() const;

  /// Returns the number vehicles being visualized.
  int num_cars() const { return static_cast<int>(visualizers_.size()); }

  /// Returns the total number of poses of bodies being visualized.
  int num_vis_poses() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  // The index is the model instance ID.
  std::vector<std::unique_ptr<const CarVis<T>>> visualizers_;
  int pose_input_port_index_{};
  int output_port_index_{};
};

}  // namespace automotive
}  // namespace drake
