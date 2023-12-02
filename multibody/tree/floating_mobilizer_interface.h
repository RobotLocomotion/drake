#pragma once

#include <memory>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class FloatingMobilizerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FloatingMobilizerInterface)
  virtual ~FloatingMobilizerInterface() = default;

  //virtual const FloatingMobilizerInterface<T>& SetFromRotationMatrix(
  //    systems::Context<T>* context,
  //    const math::RotationMatrix<T>& R_FM) const = 0;

  virtual const math::RigidTransform<T> CalcPose(
      const systems::Context<T>& context) const = 0;  

  virtual const FloatingMobilizerInterface<T>& SetFromRigidTransform(
      const systems::Context<T>& context, const math::RigidTransform<T>& X_FM,
      systems::State<T>* state) const = 0;

  const FloatingMobilizerInterface<T>& SetFromRigidTransform(
      systems::Context<T>* context, const math::RigidTransform<T>& X_FM) const {
    return SetFromRigidTransform(*context, X_FM, &context->get_mutable_state());
  }

  virtual const FloatingMobilizerInterface<T>& SetSpatialVelocity(
      const systems::Context<T>& context, const SpatialVelocity<T>& V_WB,
      systems::State<T>* state) const {
    (void)context;
    (void)V_WB;
    (void)state;
  }

 protected:
  FloatingMobilizerInterface() = default;
};

}
}
}
