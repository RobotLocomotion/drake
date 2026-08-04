#include "drake/examples/multibody/deformable/point_source_force_field.h"

namespace drake {
namespace examples {
namespace deformable {

using multibody::ForceDensityFieldBase;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using systems::BasicVector;
using systems::Context;

PointSourceForceField::PointSourceForceField(
    const MultibodyPlant<double>& plant, const RigidBody<double>& body,
    const Vector3<double>& p_BC, double falloff_distance)
    : plant_(&plant),
      body_(body.index()),
      p_BC_(p_BC),
      falloff_distance_(falloff_distance) {
  DRAKE_THROW_UNLESS(falloff_distance_ > 0);
}

Vector3<double> PointSourceForceField::DoEvaluateAt(
    const Context<double>& context, const Vector3<double>& p_WQ) const {
  const Vector3<double>& p_WC = EvalPointSourceLocation(context);
  Vector3<double> p_QC_W = p_WC - p_WQ;
  const double dist = p_QC_W.norm();
  if (dist == 0 || dist > falloff_distance_) {
    return Vector3<double>::Zero();
  }
  const double max_value =
      maximum_force_density_input_port().HasValue(context)
          ? maximum_force_density_input_port()
                .Eval<systems::BasicVector<double>>(context)[0]
          : 0.0;
  const double magnitude =
      (falloff_distance_ - dist) * max_value / falloff_distance_;
  return magnitude * p_QC_W / p_QC_W.norm();
}

std::unique_ptr<ForceDensityFieldBase<double>> PointSourceForceField::DoClone()
    const {
  return std::make_unique<PointSourceForceField>(
      *plant_, plant_->get_body(body_), p_BC_, falloff_distance_);
}

void PointSourceForceField::DoDeclareCacheEntries(
    MultibodyPlant<double>* plant) {
  /* We store the location of the point source C in the world frame as a cache
   entry so we don't need to repeatedly compute it. */
  point_source_position_cache_index_ =
      this->DeclareCacheEntry(
              plant, "point source of the force field",
              systems::ValueProducer(
                  this, &PointSourceForceField::CalcPointSourceLocation),
              {systems::System<double>::xd_ticket()})
          .cache_index();
}

void PointSourceForceField::DoDeclareInputPorts(MultibodyPlant<double>* plant) {
  maximum_force_density_port_index_ =
      this->DeclareVectorInputPort(plant,
                                   "maximum force density magnitude in N/m³",
                                   BasicVector<double>(1))
          .get_index();
}

}  // namespace deformable
}  // namespace examples
}  // namespace drake
