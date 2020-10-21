#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/element_cache.h"
#include "drake/multibody/fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** The states in the FEM simulation that are associated with the nodes and the
 elements. The states include the positions of the nodes, `x`, and their time
 derivatives, `v`. FemState also contains the cached quantities that are
 associated with the elements whose values depend on the states. See
 ElementCache.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class FemState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  FemState() = default;

  /** Resize the position and velocity states to the input `size`. The existing
   values are unchanged if `size` is greater than or equal to the size of the
   existing states. */
  void Resize(int size) {
    DRAKE_DEMAND(size >= 0);
    v_.conservativeResize(size);
    x_.conservativeResize(size);
  }

  /// State getters.
  /// @{
  const VectorX<T>& v() const { return v_; }

  const VectorX<T>& x() const { return x_; }
  /// @}

  /// State setters. The value provided must match the current size of the
  /// states.
  /// @{
  void set_v(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(value.size() == v_.size());
    if (value == v_) return;
    v_ = value;
  }

  void set_x(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(value.size() == x_.size());
    if (value == x_) return;
    x_ = value;
  }
  /// @}

  /// Mutable state getters. The value of the states is mutable but the sizes of
  /// the states are not allowed to change.
  /// @{
  Eigen::VectorBlock<VectorX<T>> mutable_v() { return v_.head(v_.size()); }

  Eigen::VectorBlock<VectorX<T>> mutable_x() { return x_.head(x_.size()); }
  /// @}

  /// Getters and mutable getters for cached quantities.
  /// @{
  const std::vector<std::unique_ptr<ElementCache<T>>>& cache() const {
    return cache_;
  }

  std::vector<std::unique_ptr<ElementCache<T>>>& mutable_cache() const {
    return cache_;
  }

  const ElementCache<T>& cache_at(ElementIndex e) const { return *cache_[e]; }

  ElementCache<T>& mutable_cache_at(ElementIndex e) const { return *cache_[e]; }
  /// @}

 private:
  // Node velocities.
  VectorX<T> v_;
  // Node positions.
  VectorX<T> x_;
  // Owned cached quantities.
  mutable std::vector<std::unique_ptr<ElementCache<T>>> cache_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
