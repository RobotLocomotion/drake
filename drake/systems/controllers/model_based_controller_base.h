#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/state_feedback_controller_base.h"

namespace drake {
namespace systems {

/**
 * Interface for model based state feedback controllers. This class needs to be
 * extended by concrete implementations. This class maintains an instance of a
 * RigidBodyTree, which is used for control.
 */
template <typename T>
class ModelBasedController : public StateFeedbackController<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModelBasedController)

  /**
   * Returns a constant reference to the RigidBodyTree used for control.
   */
  const RigidBodyTree<T>& get_robot_for_control() const {
    return *robot_for_control_;
  }

 protected:
  /**
   * Constructs the controller that instantiates a RigidBodyTree directly
   * from a model file.
   * @param model_path Path to the model file.
   * @param world_offset X_WB, where B is the base frame of the model.
   * @param floating_base_type Type of the joint between model's base link
   * and the world. Defaults to multibody::joints::kFixed.
   */
  ModelBasedController(const std::string& model_path,
                       std::shared_ptr<RigidBodyFrame<double>> world_offset,
                       const multibody::joints::FloatingBaseType
                           floating_base_type = multibody::joints::kFixed) {
    robot_for_control_ = std::make_unique<RigidBodyTree<T>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        model_path, floating_base_type, world_offset, robot_for_control_.get());
  }

  /**
   * Constructs the controller that clones a given RigidBodyTree.
   * @param robot Reference to the RigidBodyTree to be cloned.
   */
  explicit ModelBasedController(const RigidBodyTree<T>& robot) {
    robot_for_control_ = robot.Clone();
  }

  /**
   * Constructs the controller that takes ownership of a given RigidBodyTree
   * unique pointer.
   * @param robot Unique pointer whose ownership will be transfered to this
   * instance.
   */
  explicit ModelBasedController(std::unique_ptr<RigidBodyTree<T>> robot)
      : robot_for_control_(std::move(robot)) {}

 private:
  std::unique_ptr<RigidBodyTree<T>> robot_for_control_{nullptr};
};

}  // namespace systems
}  // namespace drake
