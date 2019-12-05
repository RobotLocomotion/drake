#include "drake/examples/planar_gripper/planar_manipuland_lcm.h"

namespace drake {
namespace examples {
namespace planar_gripper {
PlanarManipulandStatusDecoder::PlanarManipulandStatusDecoder() {
  this->DeclareVectorOutputPort(systems::BasicVector<double>(6),
                                &PlanarManipulandStatusDecoder::OutputStatus);
  this->DeclareAbstractInputPort("manipuland_state",
                                 Value<lcmt_planar_manipuland_status>{});
  this->DeclareDiscreteState(6);
  this->DeclarePeriodicDiscreteUpdateEvent(
      kPlanarManipulandStatusPeriod, 0.,
      &PlanarManipulandStatusDecoder::UpdateDiscreteState);
  // Register a forced discrete state update event. It is added for unit test,
  // or for potential users who require forced updates.
  this->DeclareForcedDiscreteUpdateEvent(
      &PlanarManipulandStatusDecoder::UpdateDiscreteState);
}

systems::EventStatus PlanarManipulandStatusDecoder::UpdateDiscreteState(
    const systems::Context<double>& context,
    systems::DiscreteValues<double>* discrete_state) const {
  const AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& status = input->get_value<lcmt_planar_manipuland_status>();

  systems::BasicVector<double>& state = discrete_state->get_mutable_vector(0);
  auto state_value = state.get_mutable_value();

  state_value(0) = status.position[0];
  state_value(1) = status.position[1];
  state_value(2) = status.theta;
  state_value(3) = status.velocity[0];
  state_value(4) = status.velocity[1];
  state_value(5) = status.thetadot;

  return systems::EventStatus::Succeeded();
}

void PlanarManipulandStatusDecoder::OutputStatus(
    const systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  Eigen::VectorBlock<VectorX<double>> output_vec = output->get_mutable_value();
  output_vec = context.get_discrete_state(0).get_value();
}

PlanarManipulandStatusEncoder::PlanarManipulandStatusEncoder() {
  this->DeclareInputPort(systems::kVectorValued, 6);
  this->DeclareAbstractOutputPort(
      &PlanarManipulandStatusEncoder::MakeOutputStatus,
      &PlanarManipulandStatusEncoder::OutputStatus);
}

lcmt_planar_manipuland_status PlanarManipulandStatusEncoder::MakeOutputStatus()
    const {
  lcmt_planar_manipuland_status msg{};
  msg.utime = 0;
  return msg;
}

void PlanarManipulandStatusEncoder::OutputStatus(
    const systems::Context<double>& context,
    lcmt_planar_manipuland_status* output) const {
  output->utime = context.get_time() * 1e6;
  const systems::BasicVector<double>* input = this->EvalVectorInput(context, 0);
  output->position[0] = input->GetAtIndex(0);
  output->position[1] = input->GetAtIndex(1);
  output->theta = input->GetAtIndex(2);
  output->velocity[0] = input->GetAtIndex(3);
  output->velocity[1] = input->GetAtIndex(4);
  output->thetadot = input->GetAtIndex(5);
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
