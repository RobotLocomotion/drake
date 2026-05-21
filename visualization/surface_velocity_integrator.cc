#include "drake/visualization/surface_velocity_integrator.h"

#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/value.h"

namespace drake {
namespace visualization {

using systems::BusValue;
using systems::DiscreteValues;

SurfaceVelocityIntegrator::SurfaceVelocityIntegrator(
    std::vector<std::string> body_names, double period)
    : body_names_(std::move(body_names)), period_(period) {
  DRAKE_THROW_UNLESS(period > 0.0);

  surface_speeds_input_port_index_ =
      this->DeclareAbstractInputPort("surface_speeds", Value<BusValue>())
          .get_index();

  // Pre-populate the model BusValue with double-typed signals so that the
  // output port's type is well-defined before the first update.
  BusValue model_bus;
  for (const auto& name : body_names_) {
    model_bus.Set(name, Value<double>(0.0));
  }
  surface_displacements_output_port_index_ =
      this->DeclareAbstractOutputPort("surface_displacements", model_bus,
                                      &SurfaceVelocityIntegrator::CalcOutputBus)
          .get_index();

  // One discrete state variable per body (accumulated displacement).
  state_index_ = this->DeclareDiscreteState(
      Eigen::VectorXd::Zero(static_cast<int>(body_names_.size())));

  this->DeclarePeriodicDiscreteUpdateEvent(
      period_, 0.0, &SurfaceVelocityIntegrator::DiscreteUpdate);
}

SurfaceVelocityIntegrator::~SurfaceVelocityIntegrator() = default;

SurfaceVelocityIntegrator& SurfaceVelocityIntegrator::AddToBuilder(
    systems::DiagramBuilder<double>* builder,
    const multibody::MultibodyPlant<double>& plant,
    const systems::OutputPort<double>& surface_speeds_port,
    geometry::MeshcatVisualizer<double>& visualizer, double period) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(plant.is_finalized());

  std::vector<std::string> body_names;
  for (multibody::BodyIndex i(0); i < plant.num_bodies(); ++i) {
    const auto& body = plant.get_body(i);
    if (plant.HasSurfaceVelocity(body)) {
      body_names.push_back(body.scoped_name().to_string());
    }
  }

  auto& integrator = *builder->template AddSystem<SurfaceVelocityIntegrator>(
      std::move(body_names), period);
  integrator.set_name("surface_velocity_integrator");
  builder->Connect(surface_speeds_port, integrator.surface_speeds_input_port());
  builder->Connect(integrator.surface_displacements_output_port(),
                   visualizer.surface_displacement_input_port());
  builder->Connect(plant.get_surface_velocity_axes_output_port(),
                   visualizer.surface_velocity_axes_input_port());
  return integrator;
}

void SurfaceVelocityIntegrator::DiscreteUpdate(
    const systems::Context<double>& context,
    DiscreteValues<double>* updates) const {
  Eigen::VectorXd next = context.get_discrete_state(state_index_).value();

  const auto& port = this->get_input_port(surface_speeds_input_port_index_);
  if (port.HasValue(context)) {
    const auto& bus = port.Eval<BusValue>(context);
    for (int i = 0; i < static_cast<int>(body_names_.size()); ++i) {
      const AbstractValue* v = bus.Find(body_names_[i]);
      if (v == nullptr) continue;
      next[i] += v->get_value<double>() * period_;
    }
  }

  updates->get_mutable_vector(state_index_).SetFromVector(next);
}

void SurfaceVelocityIntegrator::CalcOutputBus(
    const systems::Context<double>& context, BusValue* output) const {
  const Eigen::VectorXd& state =
      context.get_discrete_state(state_index_).value();
  for (int i = 0; i < static_cast<int>(body_names_.size()); ++i) {
    output->Set(body_names_[i], Value<double>(state[i]));
  }
}

}  // namespace visualization
}  // namespace drake
