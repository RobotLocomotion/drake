#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/*
 @tparam_double_only */
template <typename T>
class DeformableDriver : public ScalarConvertibleComponent<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableDriver)

  /* Constructs a deformable driver that solves for the dynamics of the given
   `deformable_model`. The newly constructed driver is used in the given
   `manager` to perform discrete updates. The given `deformable_model` and
   `manager` must outlive this driver.
   @pre deformable_model != nullptr.
   @pre manager != nullptr. */
  DeformableDriver(const DeformableModel<T>* deformable_model,
                   const DiscreteUpdateManager<T>* manager);

  ~DeformableDriver() = default;

  bool is_cloneable_to_double() const final { return true; }
  bool is_cloneable_to_autodiff() const final { return false; }
  bool is_cloneable_to_symbolic() const final { return false; }

  /* Declare cache entries used by this DeformableDriver through the given
   manager.
   @pre `manager` is not nullptr and points to the same DiscreteUpdateManager
   provided at construction. */
  void DeclareCacheEntries(DiscreteUpdateManager<T>* manager);

 private:
  /* Struct used to conglomerate the indexes of cache entries declared by
   the manager. */
  struct CacheIndexes {
    /* Per body cache entries indexed by DeformableBodyIndex. */
    std::vector<systems::CacheIndex> fem_states;
  };

  /* Provide private access for unit testing only. */
  friend class DeformableDriverTester;

  // TODO(xuchenhan-tri): Implement this.
  std::unique_ptr<DiscreteUpdateManager<double>> CloneToDouble() const;

  /* Copies the state of the deformable body with `id` in the given `context`
   to the `fem_state`.
   @pre fem_state != nullptr and has size compatible with the state of the
        deformable body with the given `id`. */
  void CalcFemState(const systems::Context<T>& context,
                    DeformableBodyIndex index,
                    fem::FemState<T>* fem_state) const;

  /* Eval version of CalcFemState(). */
  const fem::FemState<T>& EvalFemState(const systems::Context<T>& context,
                                       DeformableBodyIndex index) const;

  CacheIndexes cache_indexes_;
  /* Modelling information about all deformable bodies. */
  const DeformableModel<T>* deformable_model_{};
  const DiscreteUpdateManager<T>* manager_{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::DeformableDriver);
