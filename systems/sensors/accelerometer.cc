#include "drake/systems/sensors/accelerometer.h"

#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace systems {
namespace sensors {

using math::RigidTransform;
using multibody::SpatialVelocity;
using multibody::SpatialAcceleration;

template <typename T>
Accelerometer<T>::Accelerometer(multibody::BodyIndex index, RigidTransform<T> X_BC) :
    index_(index),
    X_BC_(X_BC) {

  // Declare measurement output port.
  this->DeclareVectorOutputPort("acceleration", BasicVector<T>(3),
                                &Accelerometer<T>::CalcOutput);

  body_poses_input_port_ = &this->DeclareAbstractInputPort(
      "body_poses", Value<std::vector<RigidTransform<T>>>());
  body_velocities_input_port_ = &this->DeclareAbstractInputPort(
      "body_velocities", Value<std::vector<SpatialVelocity<T>>>());
  body_accelerations_input_port_ = &this->DeclareAbstractInputPort(
      "body_accelerations", Value<std::vector<SpatialAcceleration<T>>>());
}

template <typename T>
void Accelerometer<T>::CalcOutput(const Context<T>& context,
    BasicVector<T>* output) const {
  const AbstractValue* poses_input = this->EvalAbstractInput(context,
      body_poses_input_port_->get_index());
  const AbstractValue* velocities_input = this->EvalAbstractInput(context,
      body_velocities_input_port_->get_index());
  const AbstractValue* accelerations_input = this->EvalAbstractInput(context,
      body_accelerations_input_port_->get_index());
  DRAKE_ASSERT(velocities_input != nullptr);
  DRAKE_ASSERT(accelerations_input != nullptr);

  // Extract poses spatial velocity/acceleration inputs
  const auto& X_WB = poses_input->
      get_value<std::vector<RigidTransform<T>>>()[index_];
  const auto& V_WB = velocities_input->
      get_value<std::vector<SpatialVelocity<T>>>()[index_];
  const auto& A_WB = accelerations_input->
      get_value<std::vector<SpatialAcceleration<T>>>()[index_];

  // Acceleration of the accelerometer expressed in world coordinates
  auto A_WC = A_WB.Shift(X_BC_.translation(), V_WB.rotational());

  // Rotation from world to accelerometer
  auto R_CW = X_BC_.rotation().transpose() * X_WB.rotation().transpose();
   
  // Rotate to local frame and return
  output->SetFromVector(R_CW * A_WC.translational());
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::sensors::Accelerometer)

}  // namespace sensors
}  // namespace systems
}  // namespace drake
