#include "drake/examples/Cars/simple_car-inl.h"

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

}

DRAKE_SIMPLE_CAR_INSTANTIATE_SCALAR_TYPE(double)
