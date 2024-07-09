#include "drake/multibody/plant/wing.h"

#include "drake/math/differentiable_norm.h"

namespace drake {
namespace multibody {

using math::RigidTransform;

template <typename T>
Wing<T>::Wing(BodyIndex body_index, double surface_area,
              const math::RigidTransform<double>& X_BodyWing,
              double fluid_density)
    : systems::LeafSystem<T>(systems::SystemTypeTag<Wing>{}),
      body_index_(body_index),
      X_BodyWing_(X_BodyWing),
      surface_area_(surface_area),
      default_fluid_density_(fluid_density) {
  const systems::InputPortIndex body_poses_index =
      this->DeclareAbstractInputPort("body_poses",
                                     Value<std::vector<RigidTransform<T>>>())
          .get_index();

  this->DeclareAbstractInputPort("body_spatial_velocities",
                                 Value<std::vector<SpatialVelocity<T>>>());

  this->DeclareVectorInputPort("wind_velocity_at_aerodynamic_center", 3);

  this->DeclareVectorInputPort("fluid_density", 1);

  this->DeclareAbstractOutputPort("spatial_force", &Wing<T>::CalcSpatialForce);

  this->DeclareVectorOutputPort("aerodynamic_center", 3,
                                &Wing<T>::CalcAerodynamicCenter,
                                {this->input_port_ticket(body_poses_index)});
}

template <typename T>
Wing<T>::~Wing() = default;

template <typename T>
Wing<T>* Wing<T>::AddToBuilder(systems::DiagramBuilder<T>* builder,
                               const multibody::MultibodyPlant<T>* plant,
                               const BodyIndex& body_index, double surface_area,
                               const math::RigidTransform<double>& X_BodyWing,
                               double fluid_density) {
  Wing<T>* wing = builder->template AddSystem<Wing<T>>(
      body_index, surface_area, X_BodyWing, fluid_density);
  builder->Connect(plant->get_body_poses_output_port(),
                   wing->get_body_poses_input_port());
  builder->Connect(plant->get_body_spatial_velocities_output_port(),
                   wing->get_body_spatial_velocities_input_port());
  builder->Connect(wing->get_spatial_force_output_port(),
                   plant->get_applied_spatial_force_input_port());
  return wing;
}

template <typename T>
void Wing<T>::CalcSpatialForce(
    const systems::Context<T>& context,
    std::vector<ExternallyAppliedSpatialForce<T>>* spatial_force) const {
  spatial_force->resize(1);
  const RigidTransform<T>& X_WorldBody =
      get_body_poses_input_port().template Eval<std::vector<RigidTransform<T>>>(
          context)[body_index_];
  const SpatialVelocity<T>& V_WorldBody =
      get_body_spatial_velocities_input_port()
          .template Eval<std::vector<SpatialVelocity<T>>>(context)[body_index_];

  const T fluid_density = get_fluid_density_input_port().HasValue(context)
                              ? get_fluid_density_input_port().Eval(context)[0]
                              : default_fluid_density_;
  Vector3<T> v_WorldWind = Vector3<T>::Zero();
  if (get_wind_velocity_input_port().HasValue(context)) {
    v_WorldWind = get_wind_velocity_input_port().Eval(context);
  }
  const Vector3<T> v_WindBody_World =
      -v_WorldWind + V_WorldBody.translational();

  const math::RotationMatrix<T> R_WorldWing =
      X_WorldBody.rotation() * X_BodyWing_.rotation().template cast<T>();

  const Vector3<T> v_WindBody_Wing = R_WorldWing.transpose() * v_WindBody_World;

  // The aerodynamic force from a flat plate is summarized by a single force at
  // the aerodynamic center acting normal to the wing.  See
  // http://underactuated.csail.mit.edu/trajopt.html#perching and the
  // corresponding references.
  SpatialForce<T> F_Wing_Wing = SpatialForce<T>::Zero();
  const T longitudinal_velocity_norm = math::DifferentiableNorm(
      Vector2<T>(v_WindBody_Wing[0], v_WindBody_Wing[2]));
  F_Wing_Wing.translational()[2] = -fluid_density * surface_area_ *
                                   v_WindBody_Wing[2] *
                                   longitudinal_velocity_norm;

  ExternallyAppliedSpatialForce<T>& force = spatial_force->at(0);
  force.body_index = body_index_;
  force.p_BoBq_B = X_BodyWing_.translation();
  force.F_Bq_W = R_WorldWing * F_Wing_Wing;
}

template <typename T>
void Wing<T>::CalcAerodynamicCenter(
    const systems::Context<T>& context,
    systems::BasicVector<T>* aerodynamic_center) const {
  const RigidTransform<T>& X_WorldBody =
      get_body_poses_input_port().template Eval<std::vector<RigidTransform<T>>>(
          context)[body_index_];

  aerodynamic_center->SetFromVector(
      X_WorldBody * X_BodyWing_.translation().template cast<T>());
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::Wing);
