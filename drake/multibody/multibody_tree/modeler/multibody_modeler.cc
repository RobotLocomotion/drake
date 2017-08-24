#include "drake/multibody/multibody_tree/modeler/multibody_modeler.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class MultibodyModeler<double>;
//template class MultibodyModeler<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
