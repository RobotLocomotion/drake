#include "drake/multibody_world/multibody_world.h"

namespace drake {

template class MultibodyWorld<double>;
template class MultibodyWorld<AutoDiffXd>;

}  // namespace drake
