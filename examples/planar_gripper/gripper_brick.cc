#include "drake/examples/planar_gripper/gripper_brick.h"

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/examples/planar_gripper/planar_gripper_common.h"

namespace drake {
namespace examples {
namespace planar_gripper {

std::string to_string(Finger finger) {
  switch (finger) {
    case Finger::kFinger1: {
      return "finger 1";
    }
    case Finger::kFinger2: {
      return "finger 2";
    }
    case Finger::kFinger3: {
      return "finger 3";
    }
    default:
      throw std::runtime_error("Should not reach here.");
  }
}

template<typename T>
void AddDrakeVisualizer(systems::DiagramBuilder<T> *,
                        const geometry::SceneGraph<T> &) {
  // Disabling visualization for non-double scalar type T.
}

template<>
void AddDrakeVisualizer<double>(
    systems::DiagramBuilder<double> *builder,
    const geometry::SceneGraph<double> &scene_graph) {
  geometry::ConnectDrakeVisualizer(builder, scene_graph);
}

template<typename T>
void InitializeDiagramSimulator(const systems::Diagram<T> &) {}

template<>
void InitializeDiagramSimulator<double>(
    const systems::Diagram<double> &diagram) {
  systems::Simulator<double>(diagram).Initialize();
}

template<typename T>
std::unique_ptr<systems::Diagram<T>> ConstructDiagram(
    multibody::MultibodyPlant<T> **plant,
    geometry::SceneGraph<T> **scene_graph) {
  systems::DiagramBuilder<T> builder;
  std::tie(*plant, *scene_graph) =
      multibody::AddMultibodyPlantSceneGraph(&builder);
  const std::string gripper_path =
      FindResourceOrThrow("drake/examples/planar_gripper/planar_gripper.sdf");
  multibody::Parser parser(*plant, *scene_graph);
  parser.AddModelFromFile(gripper_path, "gripper");
  examples::planar_gripper::WeldGripperFrames(*plant);
  const std::string brick_path =
      FindResourceOrThrow("drake/examples/planar_gripper/planar_brick.sdf");
  parser.AddModelFromFile(brick_path, "brick");
  (*plant)->WeldFrames((*plant)->world_frame(),
                       (*plant)->GetFrameByName("brick_base"),
                       math::RigidTransformd());

  (*plant)->Finalize();

  AddDrakeVisualizer<T>(&builder, **scene_graph);
  return builder.Build();
}

template<typename T>
GripperBrickSystem<T>::GripperBrickSystem() {
  diagram_ = ConstructDiagram<T>(&plant_, &scene_graph_);
  InitializeDiagramSimulator(*diagram_);

  for (int i = 0; i < 3; ++i) {
    finger_shoulder_position_indices_[i] =
        plant_
            ->GetJointByName("finger" + std::to_string(i + 1) +
                "_ShoulderJoint")
            .position_start();
    finger_elbow_position_indices_[i] =
        plant_->GetJointByName("finger" + std::to_string(i + 1) + "_ElbowJoint")
            .position_start();
    finger_link2_frames_[i] =
        &(plant_->GetFrameByName("finger" + std::to_string(i + 1) + "_link2"));
  }
  brick_translate_y_position_index_ =
      plant_->GetJointByName("brick_translate_y_joint").position_start();
  brick_translate_z_position_index_ =
      plant_->GetJointByName("brick_translate_z_joint").position_start();
  brick_revolute_x_position_index_ =
      plant_->GetJointByName("brick_revolute_x_joint").position_start();
  brick_frame_ = &(plant_->GetFrameByName("brick_link"));
}

template<typename T>
const multibody::Frame<double> &GripperBrickSystem<T>::finger_link2_frame(
    Finger finger) const {
  switch (finger) {
    case Finger::kFinger1: {
      return *(finger_link2_frames_[0]);
    }
    case Finger::kFinger2: {
      return *(finger_link2_frames_[1]);
    }
    case Finger::kFinger3: {
      return *(finger_link2_frames_[2]);
    }
    default:throw std::invalid_argument("finger_link2_frame(), unknown finger.");
  }
}

template<typename T>
int GripperBrickSystem<T>::finger_shoulder_position_index(Finger finger) const {
  switch (finger) {
    case Finger::kFinger1:return finger_shoulder_position_indices_[0];
    case Finger::kFinger2:return finger_shoulder_position_indices_[1];
    case Finger::kFinger3:return finger_shoulder_position_indices_[2];
    default:
      throw std::invalid_argument(
          "finger_shoulder_position_index(): unknown finger");
  }
}

template<typename T>
int GripperBrickSystem<T>::finger_elbow_position_index(Finger finger) const {
  switch (finger) {
    case Finger::kFinger1:return finger_elbow_position_indices_[0];
    case Finger::kFinger2:return finger_elbow_position_indices_[1];
    case Finger::kFinger3:return finger_elbow_position_indices_[2];
    default:
      throw std::invalid_argument(
          "finger_elbow_position_index(): unknown finger");
  }
}

// Explicit instantiation
template
class GripperBrickSystem<double>;
}  // planar_gripper
}  // namespace examples
}  // namespace drake
