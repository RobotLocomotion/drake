#include "drake/automotive/car_vis_applicator.h"

#include <utility>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/multibody/shapes/visual_element.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/rendering/drake_visualizer_client.h"

using std::unique_ptr;
using std::vector;

namespace drake {

using systems::rendering::MakeGeometryData;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;
using systems::Value;

namespace automotive {

template <typename T>
int CarVis<T>::num_poses() const {
  return static_cast<int>(GetVisElements().size());
}

template <typename T>
BoxCarVis<T>::BoxCarVis(int model_instance_id, const std::string& name)
    : CarVis<T>(model_instance_id, name) {
  DrakeShapes::VisualElement box_vis(
      DrakeShapes::Box(Eigen::Vector3d(1.0, 1.0, 1.0)),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector4d(0.7, 0.7, 0.7, 1));

  lcmt_viewer_link_data link_data;
  link_data.name = CarVis<T>::name();
  link_data.robot_num = CarVis<T>::model_instance_id();
  link_data.num_geom = 1;
  link_data.geom.resize(1);
  link_data.geom[0] = MakeGeometryData(box_vis);

  vis_elements_.push_back(link_data);
}

template <typename T>
const vector<lcmt_viewer_link_data>& BoxCarVis<T>::GetVisElements() const {
  return vis_elements_;
}

template <typename T>
PoseBundle<T> BoxCarVis<T>::GetPoses(const Isometry3<T>& root_pose) const {
  PoseBundle<T> result(1);
  result.set_pose(0, root_pose);
  result.set_name(0, this->name());
  result.set_model_instance_id(0, this->model_instance_id());
  return result;
}

template <typename T>
CarVisApplicator<T>::CarVisApplicator() {
  pose_input_port_index_ =
      systems::LeafSystem<T>::DeclareAbstractInputPort(
          Value<PoseBundle<T>>(0)).get_index();
  output_port_index_ =
      systems::LeafSystem<T>::DeclareAbstractOutputPort(
          Value<PoseBundle<T>>(0)).get_index();
}

template <typename T>
const systems::InputPortDescriptor<T>&
CarVisApplicator<T>::get_pose_input_port() const {
  return systems::System<T>::get_input_port(pose_input_port_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
CarVisApplicator<T>::get_output_port() const {
  return systems::System<T>::get_output_port(output_port_index_);
}

template <typename T>
void CarVisApplicator<T>::AddCarVis(std::unique_ptr<CarVis<T>> vis) {
  // TODO(liang.fok) Generalize this class to not require CarVis objects to be
  // added in model instance ID order.
  const int id = vis->model_instance_id();
  if (static_cast<int>(visualizers_.size()) != id) {
    throw std::runtime_error("CarVisApplicator::AddCarVis(): Attempted to add "
        "CarVis out of model instance ID order. Expected model instance ID " +
        std::to_string(visualizers_.size()) + ", but got " +
        std::to_string(id) + ".");
  }
  visualizers_.push_back(std::move(vis));
}

template <typename T>
lcmt_viewer_load_robot CarVisApplicator<T>::get_load_robot_message() const {
  lcmt_viewer_load_robot result;
  for (const auto& visualizer : visualizers_) {
    const std::vector<lcmt_viewer_link_data>& vis_elements =
        visualizer->GetVisElements();
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
          pose_input_port_index_)->template GetValue<PoseBundle<T>>();
  DRAKE_ASSERT(vehicle_poses.get_num_poses() == num_cars());
  PoseBundle<T>& visualization_poses =
      output->GetMutableData(output_port_index_)->
          template GetMutableValue<PoseBundle<T>>();
  // Resize the output PoseBundle as necessary.
  if (visualization_poses.get_num_poses() != num_vis_poses()) {
    visualization_poses = PoseBundle<T>(num_vis_poses());
  }

  // Defines an index into the output PoseBundle.
  int pose_index{0};

  for (int i = 0; i < vehicle_poses.get_num_poses(); ++i) {
    const int id = vehicle_poses.get_model_instance_id(i);
    // TODO(liang.fok) Generalize the following to support input PoseVector
    // objects that are out of order with respect to the model instance ID.
    if (id != i) {
      throw std::runtime_error("CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid model instance ID at index " +
          std::to_string(i) + ". Expected " +
          std::to_string(i) + " but got " + std::to_string(id));
    }
    const unique_ptr<const CarVis<T>>& car_vis = visualizers_.at(i);
    const std::string& name = vehicle_poses.get_name(i);
    const std::string& expected_name = car_vis->name();
    if (name != expected_name) {
      throw std::runtime_error("CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid model name at index " +
          std::to_string(i) + ". Expected \"" + expected_name +
          "\" but got \"" + name + "\".");
    }
    const Isometry3<T>& root_pose = vehicle_poses.get_pose(i);
    const PoseBundle<T> model_vis_poses = car_vis->GetPoses(root_pose);
    for (int j = 0; j < model_vis_poses.get_num_poses(); ++j) {
      visualization_poses.set_pose(pose_index, model_vis_poses.get_pose(j));
      visualization_poses.set_name(pose_index, model_vis_poses.get_name(j));
      visualization_poses.set_model_instance_id(pose_index,
          model_vis_poses.get_model_instance_id(j));
      pose_index++;
    }
  }
}

template <typename T>
int CarVisApplicator<T>::num_vis_poses() const {
  int result{0};
  for (const auto& v : visualizers_) {
    result += v->num_poses();
  }
  return result;
}

// These instantiations must match the API documentation in
// car_vis_applicator.h.
template class CarVisApplicator<double>;
template class CarVis<double>;
template class BoxCarVis<double>;

}  // namespace automotive
}  // namespace drake
