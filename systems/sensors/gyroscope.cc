/* clang-format off to disable clang-format-includes */
#include "drake/systems/sensors/gyroscope_sensor.h"
/* clang-format on */
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace systems {
namespace sensors {

using math::RigidTransform;
using math::RotationMatrix;
using multibody::SpatialVelocity;

template <typename T>
Gyroscope<T>::Gyroscope(const multibody::Body<T>& body,
                        const RotationMatrix<double>& R_BS)
    : Gyroscope(body.index(), R_BS) {}

template <typename T>
Gyroscope<T>::Gyroscope(const multibody::BodyIndex& body_index,
                        const RotationMatrix<double>& R_BS)
    : LeafSystem<T>(SystemTypeTag<Gyroscope>{}),
      body_index_(body_index),
      R_BS_(R_BS) {
  // Declare measurement output port.
  this->DeclareVectorOutputPort("measured_angular_velocity", BasicVector<T>(3),
                                &Gyroscope<T>::CalcOutput);

  body_poses_input_port_ = &this->DeclareAbstractInputPort(
      "body_poses", Value<std::vector<RigidTransform<T>>>());
  body_velocities_input_port_ = &this->DeclareAbstractInputPort(
      "body_velocities", Value<std::vector<SpatialVelocity<T>>>());
}

template <typename T>
void Gyroscope<T>::CalcOutput(const Context<T>& context,
                              BasicVector<T>* output) const {
  const auto& X_WB =
      get_body_poses_input_port().template Eval<std::vector<RigidTransform<T>>>(
          context)[body_index_];
  const auto& V_WB =
      get_body_velocities_input_port()
          .template Eval<std::vector<SpatialVelocity<T>>>(context)[body_index_];

  // Calculate rotation from world to gyroscope
  const auto R_SW =
      R_BS_.template cast<T>().inverse() * X_WB.rotation().inverse();

  // Re-express in local frame and return
  output->SetFromVector(R_SW * V_WB.rotational());
}

template <typename T>
const Gyroscope<T>& Gyroscope<T>::AddToDiagram(
    const multibody::Body<T>& body, const RotationMatrix<double>& R_BS,
    const multibody::MultibodyPlant<T>& plant, DiagramBuilder<T>* builder) {
  const auto& gyroscope =
      *builder->template AddSystem<Gyroscope<T>>(body, R_BS);
  builder->Connect(plant.get_body_poses_output_port(),
                   gyroscope.get_body_poses_input_port());
  builder->Connect(plant.get_body_spatial_velocities_output_port(),
                   gyroscope.get_body_velocities_input_port());
  return gyroscope;
}

template <typename T>
template <typename U>
Gyroscope<T>::Gyroscope(const Gyroscope<U>& other)
    : Gyroscope(other.body_index_, other.R_BS_) {}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::sensors::Gyroscope)

}  // namespace sensors
}  // namespace systems
}  // namespace drake
