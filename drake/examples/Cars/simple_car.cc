#include "drake/examples/Cars/simple_car-inl.h"

#include "drake/core/Gradient.h"

namespace drake {

const double kInchToMeter = 0.0254;
const double kDegToRadian = 0.0174532925199;

// kDefaultConfig approximates a 2010 Toyota Prius.
const lcmt_simple_car_config_t SimpleCar::kDefaultConfig = {
  0,
  106.3 * kInchToMeter,
  59.9 * kInchToMeter,
  27 * kDegToRadian,
  45,  // meters/second
  4,  // meters/second**2
  1,  // second
  1   // Hz
};

// Explicitly instantiate all ScalarType-using definitions.
#define DRAKE_INSTANTIATE(ScalarType)                           \
template DRAKECARS_EXPORT                                       \
SimpleCar::StateVector<ScalarType> SimpleCar::dynamics(         \
      const ScalarType&,                                        \
      const SimpleCar::StateVector<ScalarType>&,                \
      const SimpleCar::InputVector<ScalarType>&) const;         \
template DRAKECARS_EXPORT                                       \
SimpleCar::OutputVector<ScalarType> drake::SimpleCar::output(   \
    const ScalarType&,                                          \
    const SimpleCar::StateVector<ScalarType>&,                  \
    const SimpleCar::InputVector<ScalarType>&) const;

// These instantiations must match the API documentation in simple_car.h.
DRAKE_INSTANTIATE(double)
DRAKE_INSTANTIATE(Drake::TaylorVarXd)

#undef DRAKE_INSTANTIATE

}  // namespace drake
