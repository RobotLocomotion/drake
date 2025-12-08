#include "drake/multibody/contact_solvers/icf/icf_data.h"

#include <limits>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
void IcfData<T>::Scratch::Resize(int num_bodies, int num_velocities,
                                 int max_clique_size, int num_couplers,
                                 std::span<const int> gain_sizes,
                                 std::span<const int> limit_sizes,
                                 std::span<const int> patch_sizes) {
  Av_minus_r.Resize(1, num_velocities, 1);

  V_WB_alpha.Resize(num_bodies, 6, 1);
  U_AbB_W.Resize(ssize(patch_sizes), 6, 1);
  v_alpha.Resize(1, num_velocities, 1);

  Gw_gain.Resize(1, max_clique_size, 1);
  Gw_limit.Resize(1, max_clique_size, 1);

  coupler_constraints_data.Resize(num_couplers);
  gain_constraints_data.Resize(gain_sizes);
  limit_constraints_data.Resize(limit_sizes);
  patch_constraints_data.Resize(patch_sizes);

  H_cc_pool.Resize(1, max_clique_size, max_clique_size);

  H_BB_pool.Resize(1, max_clique_size, max_clique_size);
  H_AA_pool.Resize(1, max_clique_size, max_clique_size);
  H_AB_pool.Resize(1, max_clique_size, max_clique_size);
  H_BA_pool.Resize(1, max_clique_size, max_clique_size);
  GJa_pool.Resize(1, 6, max_clique_size);
  GJb_pool.Resize(1, 6, max_clique_size);
}

template <typename T>
IcfData<T>::~IcfData() = default;

template <typename T>
void IcfData<T>::Resize(int num_bodies, int num_velocities, int max_clique_size,
                        int num_couplers, std::span<const int> gain_sizes,
                        std::span<const int> limit_sizes,
                        std::span<const int> patch_sizes) {
  v_.resize(num_velocities);
  V_WB_.Resize(num_bodies, 6, 1);
  Av_.resize(num_velocities);
  gradient_.resize(num_velocities);
  coupler_constraints_data_.Resize(num_couplers);
  gain_constraints_data_.Resize(gain_sizes);
  limit_constraints_data_.Resize(limit_sizes);
  patch_constraints_data_.Resize(patch_sizes);
  scratch_.Resize(num_bodies, num_velocities, max_clique_size, num_couplers,
                  gain_sizes, limit_sizes, patch_sizes);
}

template <typename T>
void IcfData<T>::set_v(const VectorX<T>& v) {
#ifdef DRAKE_ASSERT_IS_ARMED
  V_WB_.SetZero();
  Av_.setConstant(std::numeric_limits<T>::quiet_NaN());
  momentum_cost_ = std::numeric_limits<T>::quiet_NaN();
  cost_ = std::numeric_limits<T>::quiet_NaN();
  gradient_.setConstant(std::numeric_limits<T>::quiet_NaN());
#endif
  v_ = v;
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfData);
