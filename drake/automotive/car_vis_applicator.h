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
/// AddVisualizationElements() must be called before AddPoses() because that
/// determines the starting index at which both the visualization geometries and
/// their poses are stored. Specifically, AddVisualizationElements() adds the
/// visual elements to the end of the provided `lcmt_viewer_load_robot` message.
/// AddPoses saves the poses in the same indices as the visual elements within
/// the provided PoseBundle.
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
  /// @param name The name of the vehicle bing visualized. This can be any
  /// user-defined value.
  ///
  CarVis(int model_instance_id, const std::string& name)
      : model_instance_id_(model_instance_id), name_(name) {}

  virtual ~CarVis() {}

  /// Adds the visualization elements to @p message. See this class's
  /// description for more details.
  ///
  /// @pre This method was not previously called.
  void AddVisualizationElements(lcmt_viewer_load_robot* message);

  /// Computes the poses of the bodies that are part of the model instance's
  /// visualization, and adds them to @p pose_bundle. See this class's
  /// description for more details.
  ///
  /// @pre AddVisualizationElements() was called.
  void AddPoses(const Isometry3<T>& root_pose,
    systems::rendering::PoseBundle<T>* pose_bundle) const;

  /// Returns the number of poses in the visualization, which is equal to the
  /// number of bodies in the visualization.
  virtual int num_poses() const = 0;

  /// Returns the model instance ID that was supplied to the constructor.
  int model_instance_id() const { return model_instance_id_; }

  /// Returns the name that was supplied to the constructor.
  const std::string& name() const { return name_; }

 protected:
  /// The implementation of AddVisualizationElements() to be defined by
  /// child classes. Child classes may assume this method is only called once,
  /// and that starting_index() returns a valid value.
  virtual void AddVisualizationElementsImpl(lcmt_viewer_load_robot* message)
      = 0;

  /// The implementation of AddPoses() to be defined by child classes. Child
  /// classes may assume AddVisualizationElements() was previously called,
  // meaning starting_index() returns a valid value.
  virtual void AddPosesImpl(const Isometry3<T>& root_pose,
    systems::rendering::PoseBundle<T>* pose_bundle) const = 0;

  /// Returns the starting index. This value will only be valid after
  /// AddVisualizationElements() is called.
  int starting_index() const { return starting_index_; }

 private:
  static const int kInvalidIndex{-1};
  const int model_instance_id_;
  const std::string name_;

  // The starting index. See documentation for set_starting_index().
  int starting_index_{kInvalidIndex};
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
  BoxCarVis(int model_instance_id, const std::string& name)
      : CarVis<T>(model_instance_id, name) {}

  void AddVisualizationElementsImpl(lcmt_viewer_load_robot* message) override;

  void AddPosesImpl(const Isometry3<T>& root_pose,
      systems::rendering::PoseBundle<T>* pose_bundle) const override;

  int num_poses() const override { return 1; }
};

/// CarVisApplicator applies, for each vehicle instance whose pose is described
/// in a PoseVector, the poses of all bodies belonging to the vehicle instance's
/// visualization.
///
/// Prior to the start of simulation, a CarVis visualizer must be provided for
/// each vehicle instance using AddCarVis(). The total number of poses defined
/// among all CarVis visualizers must equal the number of visualization poses
/// provided to the constructor.
///
/// This system is stateless and is direct feed-through.
///
/// Input port getters:
///  - get_pose_input_port() - Contains a PoseBundle of vehicle instance poses.
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

  /// The constructor.
  ///
  /// @param[in] num_car_poses The total number of cars to visualize.
  ///
  /// @param[in] num_vis_poses The total number of poses across all car
  /// visualizations.
  explicit CarVisApplicator(int num_car_poses, int num_vis_poses);
  ~CarVisApplicator() override {}

  /// Returns a descriptor of the input port that contains the poses of the
  /// vehicle instances, in the form of a PoseVector.
  const systems::InputPortDescriptor<T>& get_pose_input_port() const;

  /// Returns a descriptor of the output port that contains the poses of the
  /// bodies that represent visualizations of the vehicle instances.
  const systems::OutputPortDescriptor<T>& get_output_port() const;

  /// Adds a CarVis for a model instance. The ID of the model instance is
  /// defined by the provided CarVis object using CarVis::model_instance_id().
  /// A CarVis must be provided for each model instance. A std::runtime_error
  /// exception is thrown if more than one CarVis is provided for a particular
  /// model instance ID. CarVis objects must be added in ascending order by
  /// model instance ID starting at zero.
  void AddCarVis(std::unique_ptr<CarVis<T>> vis);

  /// Returns an lcmt_viewer_load_robot message containing the geometries of the
  /// bodies being visualized.
  const lcmt_viewer_load_robot& get_load_robot_message() const {
    return load_robot_message_;
  }

  // System<T> overrides.
  // Declare that the output is algebraically connected to the input.
  bool has_any_direct_feedthrough() const override { return true; }

  /// Returns the total number of vehicle model instances being visualized by
  /// this CarVisApplicator.
  int num_model_instances() const { return car_visualizers_.size(); }

  /// Returns the total number of poses of vehicles being visualized.
  int num_car_poses() const { return num_car_poses_; }

  /// Returns the total number of poses of bodies being visualized.
  int num_vis_poses() const { return num_vis_poses_; }

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  // Returns the total number of poses across all vehicle visualizations.
  int num_poses_provided_by_visualizers() const;

  // The index is the model instance ID.
  std::vector<std::unique_ptr<const CarVis<T>>> car_visualizers_;
  const int num_car_poses_;
  const int num_vis_poses_;
  lcmt_viewer_load_robot load_robot_message_;
  int pose_input_port_index_{};
  int output_port_index_{};
};

}  // namespace automotive
}  // namespace drake
