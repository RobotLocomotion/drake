#include "drake/automotive/car_vis_applicator.h"

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/multibody/shapes/visual_element.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/rendering/drake_visualizer_client.h"

using std::unique_ptr;
using std::vector;

namespace drake {

using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;
using systems::Value;

namespace automotive {

template <typename T>
CarVisApplicator<T>::CarVisApplicator() {
  input_port_index_ =
      systems::LeafSystem<T>::DeclareAbstractInputPort(
          Value<PoseBundle<T>>(0)).get_index();
  output_port_index_ =
      systems::LeafSystem<T>::DeclareAbstractOutputPort().get_index();
}

template <typename T>
const systems::InputPortDescriptor<T>&
CarVisApplicator<T>::get_car_poses_input_port() const {
  return systems::System<T>::get_input_port(input_port_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
CarVisApplicator<T>::get_visual_geometry_poses_output_port() const {
  return systems::System<T>::get_output_port(output_port_index_);
}

template <typename T>
void CarVisApplicator<T>::AddCarVis(std::unique_ptr<CarVis<T>> vis) {
  const int id = vis->id();
  if (visualizers_.find(id) != visualizers_.end()) {
    throw std::runtime_error("CarVisApplicator::AddCarVis(): Attempted to add "
        "CarVis with duplicate ID of " + std::to_string(id) + ".");
  }
  visualizers_[id] = std::move(vis);
}

template <typename T>
lcmt_viewer_load_robot CarVisApplicator<T>::get_load_robot_message() const {
  lcmt_viewer_load_robot result;
  for (const auto& visualizer : visualizers_) {
    const std::vector<lcmt_viewer_link_data>& vis_elements =
        visualizer.second->GetVisElements();
    for (const auto& vis_element : vis_elements) {
      result.link.push_back(vis_element);
    }
  }
  result.num_links = result.link.size();
  return result;
}

template <typename T>
void CarVisApplicator<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  // Obtains the input and output.
  const PoseBundle<T>& vehicle_poses =
      systems::System<T>::EvalAbstractInput(context,
          input_port_index_)->template GetValue<PoseBundle<T>>();
  DRAKE_ASSERT(vehicle_poses.get_num_poses() == num_cars());
  PoseBundle<T>& visualization_poses =
      output->GetMutableData(output_port_index_)->
          template GetMutableValue<PoseBundle<T>>();
  // Resize the output PoseBundle as necessary.
  if (visualization_poses.get_num_poses() != num_vis_poses()) {
    visualization_poses = PoseBundle<T>(num_vis_poses());
  }

  if (vehicle_poses.get_num_poses() != static_cast<int>(visualizers_.size())) {
    throw std::runtime_error("CarVisApplicator::DoCalcOutput(): Input "
        "PoseBundle has " + std::to_string(vehicle_poses.get_num_poses()) +
        " poses. Expected " + std::to_string(visualizers_.size()) + ".");
  }

  for (int i = 0; i < vehicle_poses.get_num_poses(); ++i) {
    const int id = vehicle_poses.get_model_instance_id(i);
    const auto vis_iterator = visualizers_.find(id);
    const auto index_iterator = starting_indices_.find(id);
    if (vis_iterator == visualizers_.end()) {
      throw std::runtime_error("CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid ID of " + std::to_string(id) + ". No CarVis "
          "matches this ID.");
    }
    if (index_iterator == starting_indices_.end()) {
      throw std::runtime_error("CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid ID of " + std::to_string(id) + ". No "
          "starting index matches this ID.");
    }
    const auto& car_vis = vis_iterator->second;
    int index = index_iterator->second;
    const std::string& name = vehicle_poses.get_name(i);
    const std::string& expected_name = car_vis->name();
    if (name != expected_name) {
      throw std::runtime_error("CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid model name with ID " +
          std::to_string(id) + ". Expected \"" + expected_name +
          "\" but got \"" + name + "\".");
    }
    const Isometry3<T>& root_pose = vehicle_poses.get_pose(i);
    const PoseBundle<T> model_vis_poses = car_vis->CalcPoses(root_pose);
    for (int j = 0; j < model_vis_poses.get_num_poses(); ++j) {
      visualization_poses.set_pose(index, model_vis_poses.get_pose(j));
      index++;
    }
  }
}

template <typename T>
std::unique_ptr<systems::AbstractValue>
CarVisApplicator<T>::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  PoseBundle<T> pose_bundle(num_vis_poses());
  int index{0};
  for (const auto& v : visualizers_) {
    const int id = v.first;
    const auto& car_vis = v.second;
    // Storing the index in a member variable is OK since it's done prior to
    // the start of simulation and remains constant throughout the simulation.
    starting_indices_[id] = index;
    const std::vector<lcmt_viewer_link_data>& link_data =
        car_vis->GetVisElements();
    for (const auto& data : link_data) {
      DRAKE_DEMAND(index < num_vis_poses());
      pose_bundle.set_name(index, data.name);
      pose_bundle.set_model_instance_id(index, id);
      ++index;
    }
  }
  return systems::AbstractValue::Make<PoseBundle<T>>(pose_bundle);
}

template <typename T>
int CarVisApplicator<T>::num_vis_poses() const {
  int result{0};
  for (const auto& v : visualizers_) {
    result += v.second->num_poses();
  }
  return result;
}

// These instantiations must match the API documentation in
// car_vis_applicator.h.
template class CarVisApplicator<double>;

}  // namespace automotive
}  // namespace drake
