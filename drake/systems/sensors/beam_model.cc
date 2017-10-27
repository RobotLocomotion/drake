#include "drake/systems/sensors/beam_model.h"

#include <memory>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
BeamModel<T>::BeamModel(int num_depth_readings, double max_range)
    : max_range_(max_range) {
  DRAKE_DEMAND(num_depth_readings > 0);
  DRAKE_DEMAND(max_range >= 0.0);
  // Declare depth input port.
  this->DeclareInputPort(kVectorValued, num_depth_readings);
  // Declare "event" random input port.
  this->DeclareInputPort(kVectorValued, num_depth_readings,
                         RandomDistribution::kUniform);
  // Declare "hit" random input port.
  this->DeclareInputPort(kVectorValued, num_depth_readings,
                         RandomDistribution::kGaussian);
  // Declare "short" random input port.
  this->DeclareInputPort(kVectorValued, num_depth_readings,
                         RandomDistribution::kExponential);
  // Declare "uniform" random input port.
  this->DeclareInputPort(kVectorValued, num_depth_readings,
                         RandomDistribution::kUniform);
  // Declare measurement output port.
  this->DeclareVectorOutputPort(BasicVector<T>(num_depth_readings),
                                &BeamModel<T>::CalcOutput);

  this->DeclareNumericParameter(BeamModelParams<T>());

  // Add a constraint that the "event" probabilities in BeamModelParams sum to
  // one.   Since probability_hit() is defined implicitly, this becomes the
  // inequality constraint:
  //   1 - probability_short() - probability_miss() - probability_uniform() ≥ 0.
  typename SystemConstraint<T>::CalcCallback
      calc_event_probabilities_constraint =
          [](const Context<T>& context, VectorX<T>* value) {
            const auto* params = dynamic_cast<const BeamModelParams<T>*>(
                &context.get_numeric_parameter(0));
            DRAKE_DEMAND(params != nullptr);
            (*value)[0] = 1.0 - params->probability_short() -
                          params->probability_miss() -
                          params->probability_uniform();
          };
  this->AddConstraint(std::make_unique<SystemConstraint<T>>(
      calc_event_probabilities_constraint, 1, SystemConstraintType::kInequality,
      "event probabilities sum to one"));
}

template <typename T>
BeamModel<T>::BeamModel(const DepthSensorSpecification& specification)
    : BeamModel(specification.num_depth_readings(), specification.max_range()) {
}

template <typename T>
BeamModelParams<T>& BeamModel<T>::get_mutable_parameters(
    Context<T>* context) const {
  return this->template GetMutableNumericParameter<BeamModelParams>(context, 0);
}

template <typename T>
void BeamModel<T>::CalcOutput(const systems::Context<T>& context,
                              BasicVector<T>* output) const {
  const auto params = dynamic_cast<const BeamModelParams<T>*>(
      &context.get_numeric_parameter(0));
  DRAKE_DEMAND(params != nullptr);
  const auto& depth = this->EvalEigenVectorInput(context, 0);
  const auto& w_event = this->EvalEigenVectorInput(context, 1);
  const auto& w_hit = this->EvalEigenVectorInput(context, 2);
  const auto& w_short = this->EvalEigenVectorInput(context, 3);
  const auto& w_uniform = this->EvalEigenVectorInput(context, 4);

  auto measurement = output->get_mutable_value();

  // Loop through depth inputs and compute a noisy measurement based on the
  // mixture model.
  for (int i = 0; i < static_cast<int>(depth.size()); i++) {
    if (w_event[i] <= params->probability_uniform()) {
      // Then "uniform".
      measurement[i] = max_range_ * w_uniform[i];
    } else if (w_event[i] <=
               params->probability_uniform() + params->probability_miss()) {
      // Then "miss".
      measurement[i] = max_range_;
    } else if (w_event[i] <= params->probability_uniform() +
                                 params->probability_miss() +
                                 params->probability_short() &&
        (w_short[i] / params->lambda_short()) <= depth[i]) {
      // Then "short".
      // Note: Returns that would have been greater than depth[i] are instead
      // evaluated as "hit".
      measurement[i] = w_short[i] / params->lambda_short();
    } else {
      // Then "hit".
      // Note: The tails of the Gaussian distribution are truncated to return
      // a value in [0, max_range_].  (Both) tails of the distribution are
      // treated as missed returns, so return max_range_.
      measurement[i] = depth[i] + params->sigma_hit() * w_hit[i];
      if (measurement[i] < 0.0 || measurement[i] > max_range_) {
        measurement[i] = max_range_;
      }
    }
  }
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::sensors::BeamModel)
