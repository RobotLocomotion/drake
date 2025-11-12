#include "drake/multibody/contact_solvers/icf/icf_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
void IcfData<T>::Cache::Resize(int num_bodies, int num_velocities,
                               int num_couplers,
                               std::span<const int> gain_sizes,
                               std::span<const int> limit_sizes,
                               std::span<const int> patch_sizes) {
  Av.resize(num_velocities);
  gradient.resize(num_velocities);
  spatial_velocities.Resize(num_bodies);

  coupler_constraints_data.Resize(num_couplers);
  gain_constraints_data.Resize(gain_sizes);
  limit_constraints_data.Resize(limit_sizes);
  patch_constraints_data.Resize(patch_sizes);
}

template <typename T>
void IcfData<T>::Scratch::Clear() {
  V_WB_alpha.Clear();
  U_AbB_W_pool.Clear();
  H_BB_pool.Clear();
  H_AA_pool.Clear();
  H_AB_pool.Clear();
  H_BA_pool.Clear();
  GJa_pool.Clear();
  GJb_pool.Clear();
}

template <typename T>
void IcfData<T>::Scratch::Resize(int num_bodies, int num_velocities,
                                 int max_clique_size, int num_couplers,
                                 std::span<const int> gain_sizes,
                                 std::span<const int> limit_sizes,
                                 std::span<const int> patch_sizes) {
  Clear();
  Av_minus_r.resize(num_velocities);

  V_WB_alpha.Resize(num_bodies);
  v_alpha.resize(num_velocities);

  Gw_gain.resize(num_velocities);
  Gw_limit.resize(num_velocities);
  U_AbB_W_pool.Resize(patch_sizes.size());

  coupler_constraints_data.Resize(num_couplers);
  gain_constraints_data.Resize(gain_sizes);
  limit_constraints_data.Resize(limit_sizes);
  patch_constraints_data.Resize(patch_sizes);

  H_BB_pool.Resize(1, max_clique_size, max_clique_size);
  H_AA_pool.Resize(1, max_clique_size, max_clique_size);
  H_AB_pool.Resize(1, max_clique_size, max_clique_size);
  H_BA_pool.Resize(1, max_clique_size, max_clique_size);
  GJa_pool.Resize(1, 6, max_clique_size);
  GJb_pool.Resize(1, 6, max_clique_size);
}

template <typename T>
void IcfData<T>::Resize(int num_bodies, int num_velocities, int max_clique_size,
                        int num_couplers, std::span<const int> gain_sizes,
                        std::span<const int> limit_sizes,
                        std::span<const int> patch_sizes) {
  v_.resize(num_velocities);
  cache_.Resize(num_bodies, num_velocities, num_couplers, gain_sizes,
                limit_sizes, patch_sizes);
  scratch_.Resize(num_bodies, num_velocities, num_couplers, max_clique_size,
                  gain_sizes, limit_sizes, patch_sizes);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfData);
