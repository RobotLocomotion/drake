#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace planar_gripper {

enum class Finger {
  kFinger1,
  kFinger2,
  kFinger3,
};

std::string to_string(Finger finger);

enum class BrickFace {
  kPosZ,
  kNegZ,
  kPosY,
  kNegY,
};

template <typename T>
class GripperBrickSystem {
 public:
  GripperBrickSystem();

  const systems::Diagram<T>& diagram() const { return *diagram_; }

  systems::Diagram<T>* get_mutable_diagram() { return diagram_.get(); }

  const multibody::MultibodyPlant<T>& plant() const { return *plant_; }

  multibody::MultibodyPlant<T>* get_mutable_plant() { return plant_; }

  int finger_shoulder_position_index(Finger finger) const;

  int finger_elbow_position_index(Finger finger) const;

  int brick_translate_y_position_index() const {
    return brick_translate_y_position_index_;
  }

  int brick_translate_z_position_index() const {
    return brick_translate_z_position_index_;
  }

  int brick_revolute_x_position_index() const {
    return brick_revolute_x_position_index_;
  }

  // Position of the finger tip sphere center "Tip" in the finger frame.
  Eigen::Vector3d p_F2Tip() const { return Eigen::Vector3d(0, 0, -0.086); }

  const multibody::Frame<double>& brick_frame() const { return *brick_frame_; }

  const multibody::Frame<double>& finger_link2_frame(Finger finger) const;

  double finger_tip_radius() const { return 0.015; }

  Eigen::Vector3d brick_size() const {
    return Eigen::Vector3d(0.025, 0.092, 0.092);
  }

 private:
  std::unique_ptr<systems::Diagram<T>> diagram_;
  multibody::MultibodyPlant<T>* plant_;
  geometry::SceneGraph<T>* scene_graph_;
  std::array<int, 3> finger_shoulder_position_indices_;
  std::array<int, 3> finger_elbow_position_indices_;
  int brick_translate_y_position_index_;
  int brick_translate_z_position_index_;
  int brick_revolute_x_position_index_;
  const multibody::Frame<double>* brick_frame_;
  std::array<const multibody::Frame<double>*, 3> finger_link2_frames_;
};
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
