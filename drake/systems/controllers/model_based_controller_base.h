#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/controller_base.h"

namespace drake {
namespace systems {

template<typename T>
class ModelBasedController : public Controller<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModelBasedController)

  ModelBasedController(const std::string& model_path,
      std::shared_ptr<RigidBodyFrame<double>> world_offset) {
    robot_for_control_ = std::make_unique<RigidBodyTree<T>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        model_path, multibody::joints::kFixed, world_offset,
        robot_for_control_.get());
  }

  explicit ModelBasedController(const RigidBodyTree<T>& robot) {
    robot_for_control_ = robot.Clone();
  }

  explicit ModelBasedController(std::unique_ptr<RigidBodyTree<T>> robot)
      : robot_for_control_(std::move(robot)) {}

  const RigidBodyTree<T>& get_robot_for_control() const {
    return *robot_for_control_;
  }

 protected:
  ModelBasedController() {}

 private:
  std::unique_ptr<RigidBodyTree<T>> robot_for_control_{nullptr};
};

}  // namespace systems
}  // namespace drake
