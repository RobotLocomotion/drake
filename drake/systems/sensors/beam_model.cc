#include "drake/systems/sensors/beam_model.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/sensors/gen/beam_model_params.h"

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
BeamModel<T>::BeamModel(int num_depth_readings, double max_range)
    : max_range_(max_range) {
  DRAKE_DEMAND(num_depth_readings > 0);
  DRAKE_DEMAND(max_range >= 0.0);
  this->DeclareInputPort(kVectorValued, num_depth_readings);
  this->DeclareInputPort(kVectorValued, 2 * num_depth_readings,
                         RandomDistribution::kUniform);
  this->DeclareInputPort(kVectorValued, num_depth_readings,
                         RandomDistribution::kGaussian);
  this->DeclareInputPort(kVectorValued, num_depth_readings,
                         RandomDistribution::kExponential);
  this->DeclareVectorOutputPort(BasicVector<T>(num_depth_readings),
                                &BeamModel<T>::CalcOutput);
  this->DeclareNumericParameter(BeamModelParams<T>());
}

template <typename T>
BeamModel<T>::BeamModel(const DepthSensorSpecification& specification)
    : BeamModel(specification.num_depth_readings(),
                specification.max_range()) {}

template <typename T>
void BeamModel<T>::CalcOutput(const systems::Context<T>& context,
                              BasicVector<T>* output) const {
  const auto params =
      dynamic_cast<const BeamModelParams<T>*>(context.get_numeric_parameter(0));
  DRAKE_DEMAND(params != nullptr);
  const auto& depth = this->EvalEigenVectorInput(context, 0);
  const auto& w_event =
      this->EvalEigenVectorInput(context, 1).head(depth.size());
  const auto& w_random =
      this->EvalEigenVectorInput(context, 1).tail(depth.size());
  const auto& w_hit = this->EvalEigenVectorInput(context, 2);
  const auto& w_short = this->EvalEigenVectorInput(context, 3);

  auto measurement = output->get_mutable_value();

  for (int i = 0; i < static_cast<int>(depth.size()); i++) {
    if (w_event[i] <= params->probability_random()) {
      // Then "random".
      measurement[i] = max_range_ * w_random[i];
    } else if (w_event[i] <=
               params->probability_random() + params->probability_miss()) {
      // Then "miss".
      measurement[i] = max_range_;
    } else if (w_event[i] <= params->probability_random() +
                                 params->probability_miss() +
                                 params->probability_short() &&
               w_short[i] <= depth[i] * params->lambda_short()) {
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
