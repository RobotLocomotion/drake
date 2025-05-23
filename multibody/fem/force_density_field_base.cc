#include "drake/multibody/fem/force_density_field_base.h"

namespace drake {
namespace multibody {

template <typename T>
ForceDensityFieldBase<T>::~ForceDensityFieldBase() = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ForceDensityFieldBase);
