#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace systems {
namespace controllers {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template class CartesianSetpoint<double>;
template class VectorSetpoint<double>;
#pragma GCC diagnostic pop

}  // namespace controllers
}  // namespace systems
}  // namespace drake
