#include "drake/automotive/car_vis_applicator.h"

#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/multibody/shapes/visual_element.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/rendering/drake_visualizer_client.h"

namespace drake {

using systems::rendering::MakeGeometryData;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;
using systems::Value;

namespace automotive {

template <typename T> const int CarVis<T>::kInvalidIndex;

template <typename T>
void CarVis<T>::AddVisualizationElements(lcmt_viewer_load_robot* message) {
  if (starting_index() != kInvalidIndex) {
    throw std::runtime_error("CarVis::AddVisualizationElements(): Attempted "
        "to call this method more than once.");
  }
  starting_index_ = message->num_links;
  AddVisualizationElementsImpl(message);
}

template <typename T>
void CarVis<T>::AddPoses(const Isometry3<T>& root_pose,
  systems::rendering::PoseBundle<T>* pose_bundle) const {
  if (this->starting_index() == -1) {
    throw std::runtime_error("CarVis::AddPoses(): Please call "
      "AddVisualizationElements() first.");
  }
  AddPosesImpl(root_pose, pose_bundle);
}

template <typename T>
void BoxCarVis<T>::AddVisualizationElementsImpl(
    lcmt_viewer_load_robot* message) {
  DrakeShapes::VisualElement box_vis(
      DrakeShapes::Box(Eigen::Vector3d(1.0, 1.0, 1.0)),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector4d(0.7, 0.7, 0.7, 1));

  drake::lcmt_viewer_link_data link_data;
  link_data.name = CarVis<T>::name();
  link_data.robot_num = CarVis<T>::model_instance_id();
  link_data.num_geom = 1;
  link_data.geom.resize(1);
  link_data.geom[0] = MakeGeometryData(box_vis);

  message->num_links++;
  message->link.push_back(link_data);
}

template <typename T>
void BoxCarVis<T>::AddPosesImpl(const Isometry3<T>& root_pose,
    PoseBundle<T>* pose_bundle) const {
  const int index = this->starting_index();
  DRAKE_ASSERT(index < pose_bundle->get_num_poses());
  pose_bundle->set_pose(index, root_pose);
  pose_bundle->set_name(index, this->name());
  pose_bundle->set_model_instance_id(index, this->model_instance_id());
}

template <typename T>
CarVisApplicator<T>::CarVisApplicator(int num_car_poses, int num_vis_poses)
    : num_car_poses_(num_car_poses), num_vis_poses_(num_vis_poses) {
  pose_input_port_index_ =
      systems::LeafSystem<T>::DeclareAbstractInputPort(
          Value<PoseBundle<T>>(num_car_poses)).get_index();
  output_port_index_ =
      systems::LeafSystem<T>::DeclareAbstractOutputPort(
          Value<PoseBundle<T>>(num_vis_poses)).get_index();
  // These are intialized to zero because they will be increased to the correct
  // values after AddCarVis() is called the necessary number of times.
  load_robot_message_.num_links = 0;
  load_robot_message_.link.resize(0);
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
  const int id = vis->model_instance_id();
  if (static_cast<int>(car_visualizers_.size()) != id) {
    throw std::runtime_error("CarVisApplicator::AddCarVis(): Attempted to add "
        "CarVis out of model instance ID order. Expected model instance ID " +
        std::to_string(car_visualizers_.size()) + ", but got " +
        std::to_string(id) + ".");
  }
  vis->AddVisualizationElements(&load_robot_message_);
  car_visualizers_.push_back(std::move(vis));
}

template <typename T>
void CarVisApplicator<T>::DoCalcOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  // Obtains the input and output.
  const PoseBundle<T>& vehicle_poses =
      systems::System<T>::EvalAbstractInput(context,
          pose_input_port_index_)->template GetValue<PoseBundle<T>>();
  DRAKE_ASSERT(vehicle_poses.get_num_poses() == num_model_instances());
  PoseBundle<T>& visualization_poses =
      output->GetMutableData(output_port_index_)->
          template GetMutableValue<PoseBundle<T>>();
  DRAKE_ASSERT(visualization_poses.get_num_poses() ==
      num_poses_provided_by_visualizers());

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
    const std::string& name = vehicle_poses.get_name(i);
    const std::string& expected_name = car_visualizers_.at(i)->name();
    if (name != expected_name) {
      throw std::runtime_error("CarVisApplicator::DoCalcOutput(): Input "
          "PoseBundle has invalid model name at index " +
          std::to_string(i) + ". Expected \"" + expected_name +
          "\" but got \"" + name + "\".");
    }
    const Isometry3<T>& root_pose = vehicle_poses.get_pose(i);
    car_visualizers_.at(i)->AddPoses(root_pose, &visualization_poses);
  }
}

template <typename T>
int CarVisApplicator<T>::num_poses_provided_by_visualizers() const {
  int result{0};
  for (const auto &v : car_visualizers_) {
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
