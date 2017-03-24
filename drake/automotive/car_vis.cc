#include "drake/automotive/car_vis.h"

namespace drake {
namespace automotive {

template <typename T>
int CarVis<T>::num_poses() const {
  return static_cast<int>(GetVisElements().size());
}

// These instantiations must match the API documentation in
// car_vis.h.
template class CarVis<double>;

}  // namespace automotive
}  // namespace drake
