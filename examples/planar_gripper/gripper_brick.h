#pragma once

#include <memory>
#include <string>

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

/**
 * The factory class that contains the diagram of the planar gripper (3 planar
 * fingers) with a brick.
 */
template <typename T>
class GripperBrickFactory {
 public:
  GripperBrickFactory();

  const systems::Diagram<T>& diagram() const { return *diagram_; }

  systems::Diagram<T>* get_mutable_diagram() { return diagram_.get(); }

  const multibody::MultibodyPlant<T>& plant() const { return *plant_; }

  multibody::MultibodyPlant<T>* get_mutable_plant() { return plant_; }

  /** The index of the base joint for a given finger in MultibodyPlant. */
  int finger_base_position_index(Finger finger) const;

  /** The index of the middle joint for a given finger in MultibodyPlant. */
  int finger_mid_position_index(Finger finger) const;

  int brick_translate_y_position_index() const {
    return brick_translate_y_position_index_;
  }

  int brick_translate_z_position_index() const {
    return brick_translate_z_position_index_;
  }

  int brick_revolute_x_position_index() const {
    return brick_revolute_x_position_index_;
  }

  /** Position of the finger tip sphere center "Tip" in the finger_link2 frame.
   */
  Eigen::Vector3d p_L2Tip() const { return p_L2Tip_; }

  const multibody::Frame<double>& brick_frame() const { return *brick_frame_; }

  const multibody::Frame<double>& finger_link2_frame(Finger finger) const;

  double finger_tip_radius() const { return finger_tip_radius_; }

  Eigen::Vector3d brick_size() const { return brick_size_; }

 private:
  std::unique_ptr<systems::Diagram<T>> diagram_;
  multibody::MultibodyPlant<T>* plant_;
  geometry::SceneGraph<T>* scene_graph_;
  std::array<int, 3> finger_base_position_indices_;
  std::array<int, 3> finger_mid_position_indices_;
  int brick_translate_y_position_index_;
  int brick_translate_z_position_index_;
  int brick_revolute_x_position_index_;
  const multibody::Frame<double>* brick_frame_;
  std::array<const multibody::Frame<double>*, 3> finger_link2_frames_;

  Eigen::Vector3d p_L2Tip_;
  double finger_tip_radius_;
  Eigen::Vector3d brick_size_;
};
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
