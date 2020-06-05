#include "drake/systems/sensors/accelerometer.h"

#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace systems {
namespace sensors {

using math::RigidTransform;
using math::RotationMatrix;
using multibody::SpatialVelocity;
using multibody::SpatialAcceleration;

template <typename T>
Accelerometer<T>::Accelerometer(multibody::BodyIndex index,
    RigidTransform<T> X_BC, Eigen::Vector3d gravitational_acceleration) :
    body_index_(index),
    X_BS_(X_BC),
    gravitational_acceleration_(gravitational_acceleration) {

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
  const auto& X_WB =
      (get_body_poses_input_port(). template
          Eval<std::vector<RigidTransform<T>>>(context))[body_index_];
  const auto& V_WB =
      (get_body_velocities_input_port(). template
          Eval<std::vector<SpatialVelocity<T>>>(context))[body_index_];
  const auto& A_WB =
      (get_body_accelerations_input_port(). template
          Eval<std::vector<SpatialAcceleration<T>>>(context))[body_index_];

  // Kinematic term expressed in world coordinates unless specified
  // Sensor frame position and orientation.
  const RotationMatrix<T>& R_WB = X_WB.rotation();

  // Express sensor position in world coordinates
  const Vector3<T>& p_BS_W = R_WB * X_BS_.translation();

  const RotationMatrix<T>& R_BS = X_BS_.rotation();

  // Acceleration of the accelerometer expressed in world coordinates
  const auto A_WS = A_WB.Shift(p_BS_W, V_WB.rotational());

  // Rotation from world to accelerometer
  const auto R_SW = R_BS.inverse() * X_WB.rotation().inverse();
   
  // Rotate to local frame and return
  output->SetFromVector(R_SW * A_WS.translational());
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::sensors::Accelerometer)

}  // namespace sensors
}  // namespace systems
}  // namespace drake
