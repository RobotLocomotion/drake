#include "drake/systems/plants/affine_linear_system/affine_system_plant.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake{
namespace systems {
template <typename T>
AffineSystemPlant<T>::AffineSystemPlant(bool system_is_forced) {

}



template class DRAKEAFFINELINEARSYSTEMPLANT_EXPORT AffineSystemPlant<double>;
template class DRAKEAFFINELINEARSYSTEMPLANT_EXPORT AffineSystemPlant<AutoDiffXd>;
}
}
