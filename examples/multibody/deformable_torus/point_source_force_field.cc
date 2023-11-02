#include "drake/examples/multibody/deformable_torus/point_source_force_field.h"

namespace drake {
namespace examples {
namespace deformable_torus {

using multibody::Body;
using multibody::ForceDensityField;
using multibody::MultibodyPlant;
using systems::BasicVector;
using systems::Context;

PointSourceForceField::PointSourceForceField(
    const MultibodyPlant<double>& plant, const Body<double>& body,
    const Vector3<double>& p_BC, double max_value, double distance)
    : plant_(&plant),
      body_(&body),
      p_BC_(p_BC),
      max_value_(max_value),
      distance_(distance) {
  DRAKE_THROW_UNLESS(max_value_ > 0);
  DRAKE_THROW_UNLESS(distance_ > 0);
}

Vector3<double> PointSourceForceField::DoEvaluateAt(
    const Context<double>& context, const Vector3<double>& p_WQ) const {
  const Vector3<double>& p_WC = EvalPointSourceLocation(context);
  Vector3<double> p_QC_W = p_WC - p_WQ;
  const double dist = p_QC_W.norm();
  if (dist == 0 || dist > distance_) {
    return Vector3<double>::Zero();
  }
  const double magnitude =
      IsForceOn(context) ? (distance_ - dist) * max_value_ / distance_ : 0;
  return magnitude * p_QC_W / p_QC_W.norm();
}

std::unique_ptr<ForceDensityField<double>> PointSourceForceField::DoClone()
    const {
  return std::make_unique<PointSourceForceField>(*this);
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
  signal_port_index_ =
      this->DeclareVectorInputPort(plant, "on/off signal for the force field",
                                   BasicVector<double>(1))
          .get_index();
}

}  // namespace deformable_torus
}  // namespace examples
}  // namespace drake
