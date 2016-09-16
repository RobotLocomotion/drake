#include "drake/automotive/simple_car-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

const double kInchToMeter = 0.0254;
const double kDegToRadian = 0.0174532925199;

// kDefaultConfig approximates a 2010 Toyota Prius.
const lcmt_simple_car_config_t SimpleCarDefaults::kDefaultConfig = {
  0,
  106.3 * kInchToMeter,
  59.9 * kInchToMeter,
  27 * kDegToRadian,
  45,  // meters/second
  4,  // meters/second**2
  1,  // second
  1   // Hz
};

// These instantiations must match the API documentation in simple_car.h.
template class DRAKEAUTOMOTIVE_EXPORT SimpleCar<double>;
template class DRAKEAUTOMOTIVE_EXPORT SimpleCar<drake::TaylorVarXd>;

}  // namespace automotive
}  // namespace drake
