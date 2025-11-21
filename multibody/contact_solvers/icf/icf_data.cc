#include "drake/multibody/contact_solvers/icf/icf_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
void IcfData<T>::Scratch::Clear() {
  V_WB_alpha.Clear();
  H_BB_pool.Clear();
  H_AA_pool.Clear();
  H_AB_pool.Clear();
  H_BA_pool.Clear();
  GJa_pool.Clear();
  GJb_pool.Clear();
}

template <typename T>
void IcfData<T>::Scratch::Resize(int num_bodies, int num_velocities,
                                 int max_clique_size) {
  Clear();
  Av_minus_r.resize(num_velocities);

  V_WB_alpha.Resize(num_bodies, 6, 1);
  v_alpha.resize(num_velocities);

  H_BB_pool.Resize(1, max_clique_size, max_clique_size);
  H_AA_pool.Resize(1, max_clique_size, max_clique_size);
  H_AB_pool.Resize(1, max_clique_size, max_clique_size);
  H_BA_pool.Resize(1, max_clique_size, max_clique_size);
  GJa_pool.Resize(1, 6, max_clique_size);
  GJb_pool.Resize(1, 6, max_clique_size);
}

template <typename T>
void IcfData<T>::Resize(int num_bodies, int num_velocities,
                        int max_clique_size) {
  v_.resize(num_velocities);
  V_WB_.Resize(num_bodies, 6, 1);
  Av_.resize(num_velocities);
  gradient_.resize(num_velocities);
  scratch_.Resize(num_bodies, num_velocities, max_clique_size);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfData);
