#include "drake/multibody/contact_solvers/icf/patch_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
PatchConstraintsDataPool<T>::~PatchConstraintsDataPool() = default;

template <typename T>
void PatchConstraintsDataPool<T>::Resize(std::span<const int> patch_sizes) {
  num_patches_ = ssize(patch_sizes);
  num_pairs_ = std::accumulate(patch_sizes.begin(), patch_sizes.end(), 0);

  // Data per patch.
  cost_pool_.resize(num_patches_);
  G_Bp_pool_.Resize(num_patches_, 6, 6);
  Gamma_Bo_W_.Resize(num_patches_, 6, 1);

  // Data per pair.
  v_AcBc_W_.Resize(num_pairs_, 3, 1);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        PatchConstraintsDataPool);
