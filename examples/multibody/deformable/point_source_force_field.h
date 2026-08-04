#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/force_density_field.h"

namespace drake {
namespace examples {
namespace deformable {

/* A custom external force density field that applies external force to
 deformable bodies. The force density points towards a point C affixed to a
 rigid body. The magnitude of the force density decays linearly with the
 distance to the point C and floors at 0. The maximum force density is read from
 a double-valued input port (see maximum_force_density_input_port()). If the
 port is unconnected, it reads as zero. */
class PointSourceForceField final
    : public multibody::ForceDensityField<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointSourceForceField);

  /* Constructs a new PointSourceForceField object
   @param plant             The MultibodyPlant that owns this force field.
   @param body              The body to which the source of the force field C is
                            affixed to.
   @param p_BC              The fixed offset from the body origin to the point
                            source of the force field.
   @param falloff_distance  The force density decays linearly.
                            `falloff_distance` in meters is the distance from
                            the point source beyond which the force density is
                            zero. Must be positive and finite. */
  PointSourceForceField(const multibody::MultibodyPlant<double>& plant,
                        const multibody::RigidBody<double>& body,
                        const Vector3<double>& p_BC, double falloff_distance);

  /* Input port for desired maximum force density with units of N/m³.
   The port belongs to the MultibodyPlant that owns this density field. */
  const systems::InputPort<double>& maximum_force_density_input_port() const {
    return parent_system_or_throw().get_input_port(
        maximum_force_density_port_index_);
  }

 private:
  /* Computes the world frame position of the center of the point source force
   field. */
  Vector3<double> CalcPointSourceLocation(
      const systems::Context<double>& context) const {
    return plant_->EvalBodyPoseInWorld(context, plant_->get_body(body_)) *
           p_BC_;
  }

  /* Eval version of `CalcPointSourceLocation()`. */
  const Vector3<double>& EvalPointSourceLocation(
      const systems::Context<double>& context) const {
    return parent_system_or_throw()
        .get_cache_entry(point_source_position_cache_index_)
        .template Eval<Vector3<double>>(context);
  }

  Vector3<double> DoEvaluateAt(const systems::Context<double>& context,
                               const Vector3<double>& p_WQ) const final;

  std::unique_ptr<ForceDensityFieldBase<double>> DoClone() const final;

  void DoDeclareCacheEntries(multibody::MultibodyPlant<double>* plant) final;

  void DoDeclareInputPorts(multibody::MultibodyPlant<double>* plant) final;

  const multibody::MultibodyPlant<double>* plant_{};
  const multibody::BodyIndex body_;
  Vector3<double> p_BC_;
  double falloff_distance_{};
  systems::CacheIndex point_source_position_cache_index_;
  systems::InputPortIndex maximum_force_density_port_index_;
};

}  // namespace deformable
}  // namespace examples
}  // namespace drake
