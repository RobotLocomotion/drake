/* clang-format off to disable clang-format-includes */ 
#include "drake/systems/sensors/accelerometer_sensor.h"
/* clang-format on */ 
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace systems {
namespace sensors {

using math::RigidTransform;
using math::RotationMatrix;
using multibody::SpatialAcceleration;
using multibody::SpatialVelocity;

template <typename T>
Accelerometer<T>::Accelerometer(multibody::BodyIndex index,
                                const RigidTransform<double>& X_BC,
                                const Eigen::Vector3d& gravity_vector)
    : body_index_(index),
      X_BS_(X_BC),
      gravity_vector_(gravity_vector) {
  // Declare measurement output port.
  this->DeclareVectorOutputPort("measured_acceleration", BasicVector<T>(3),
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
  const auto& X_WB = get_body_poses_input_port()
                          .template Eval<std::vector<RigidTransform<T>>>(
                              context)[body_index_];
  const auto& V_WB = get_body_velocities_input_port()
                          .template Eval<std::vector<SpatialVelocity<T>>>(
                              context)[body_index_];
  const auto& A_WB = get_body_accelerations_input_port()
                          .template Eval<std::vector<SpatialAcceleration<T>>>(
                              context)[body_index_];

  // Kinematic term expressed in world coordinates unless specified
  // Sensor frame position and orientation.
  const RotationMatrix<T>& R_WB = X_WB.rotation();

  // Express sensor position in world coordinates
  const Vector3<T>& p_BS_W = R_WB * X_BS_.translation().template cast<T>();

  const RotationMatrix<T>& R_BS = X_BS_.rotation().template cast<T>();

  // Acceleration of the accelerometer expressed in world coordinates
  const auto A_WS = A_WB.Shift(p_BS_W, V_WB.rotational());

  // Rotation from world to accelerometer
  const auto R_SW = R_BS.inverse() * R_WB.inverse();

  // Rotate to local frame and return
  output->SetFromVector(R_SW * (A_WS.translational() - gravity_vector_));
}

template <typename T>
const Accelerometer<T>& Accelerometer<T>::AddToDiagram(
    multibody::BodyIndex body_index,
    const RigidTransform<double>& X_BS,
    const Eigen::Vector3d& gravity_vector,
    const multibody::MultibodyPlant<T>& plant,
    DiagramBuilder<T>* builder) {
  const auto& accelerometer =
      *builder->template AddSystem<Accelerometer<T>>(body_index, X_BS,
          gravity_vector);
  builder->Connect(plant.get_body_poses_output_port(),
                   accelerometer.get_body_poses_input_port());
  builder->Connect(plant.get_body_spatial_velocities_output_port(),
                   accelerometer.get_body_velocities_input_port());
  builder->Connect(plant.get_body_spatial_accelerations_output_port(),
                   accelerometer.get_body_accelerations_input_port());
  return accelerometer;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::sensors::Accelerometer)

}  // namespace sensors
}  // namespace systems
}  // namespace drake
