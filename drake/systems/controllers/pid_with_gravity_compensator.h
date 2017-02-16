#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/model_based_controller_base.h"

namespace drake {
namespace systems {

template<typename T>
class PidWithGravityCompensator : public ModelBasedController<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidWithGravityCompensator)

  PidWithGravityCompensator(const std::string& model_path,
      std::shared_ptr<RigidBodyFrame<double>> world_offset,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);

  PidWithGravityCompensator(const RigidBodyTree<T>& robot,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);

  PidWithGravityCompensator(std::unique_ptr<RigidBodyTree<T>> robot,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);

 private:
  void SetUp(const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);
};

}  // namespace systems
}  // namespace drake
