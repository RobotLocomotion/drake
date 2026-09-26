#include "drake/multibody/contact_solvers/icf/icf_data.h"

#include <iterator>
#include <limits>
#include <memory>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
void IcfData<T>::Scratch::Resize(const ResizeParams& params) {
  Av_minus_r.Resize(1, params.num_velocities, 1);

  V_WB_alpha.Resize(params.num_bodies, 6, 1);
  U_AbB_W.Resize(ssize(params.patch_sizes), 6, 1);
  v_alpha.Resize(1, params.num_velocities, 1);

  Gw_gain.Resize(1, params.max_clique_size, 1);
  Gw_limit.Resize(1, params.max_clique_size, 1);

  ball_constraints_data.Resize(params.num_ball_constraints);
  coupler_constraints_data.Resize(params.num_couplers);
  distance_constraints_data.Resize(params.num_distance_constraints);
  gain_constraints_data.Resize(params.gain_sizes);
  limit_constraints_data.Resize(params.limit_sizes);
  patch_constraints_data.Resize(params.patch_sizes);
  weld_constraints_data.Resize(params.num_welds);

  H_cc_pool.Resize(1, params.max_clique_size, params.max_clique_size);

  H_BB_pool.Resize(1, params.max_clique_size, params.max_clique_size);
  H_AA_pool.Resize(1, params.max_clique_size, params.max_clique_size);
  H_AB_pool.Resize(1, params.max_clique_size, params.max_clique_size);
  H_BA_pool.Resize(1, params.max_clique_size, params.max_clique_size);
  GJa_pool.Resize(1, 6, params.max_clique_size);
  GJb_pool.Resize(1, 6, params.max_clique_size);
}

template <typename T>
IcfData<T>::~IcfData() = default;

template <typename T>
void IcfData<T>::Resize(const ResizeParams& params) {
  v_.resize(params.num_velocities);
  V_WB_.Resize(params.num_bodies, 6, 1);
  Av_.resize(params.num_velocities);
  gradient_.resize(params.num_velocities);
  ball_constraints_data_.Resize(params.num_ball_constraints);
  coupler_constraints_data_.Resize(params.num_couplers);
  distance_constraints_data_.Resize(params.num_distance_constraints);
  gain_constraints_data_.Resize(params.gain_sizes);
  limit_constraints_data_.Resize(params.limit_sizes);
  patch_constraints_data_.Resize(params.patch_sizes);
  weld_constraints_data_.Resize(params.num_welds);
  scratch_.Resize(params);

  // Per-island storage. Grow-only: keep prior capacity (and any prior scratch
  // allocations) and only extend when the island count grows.
  DRAKE_DEMAND(params.num_islands > 0 || params.num_velocities == 0);
  island_cost_.resize(params.num_islands);
  island_scratch_.reserve(params.num_islands);
  while (std::ssize(island_scratch_) < params.num_islands) {
    island_scratch_.push_back(std::make_unique<Scratch>());
  }
  for (int i = 0; i < params.num_islands; ++i) {
    island_scratch_[i]->Resize(params);
  }
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
