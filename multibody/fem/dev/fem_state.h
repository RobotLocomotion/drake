#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/element_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** The states in the FEM simulation that live on element vertices. The states
 include the position of the vertices `x` and its time derivative `v` as well
 as the same quantities evaluated at the previous time step. FemState also
 contains the cached quantities that live on the element and whose values
 depend on the states.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain. */
template <typename T, int SpatialDim>
class FemState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  FemState() = default;

  using VectorD = Eigen::Matrix<T, SpatialDim, 1>;
  using ConstVectorBlockD = Eigen::VectorBlock<const VectorX<T>, SpatialDim>;

  int num_verts() const { return num_verts_; }

  /// Resize the FemState to hold the states for `v` vertices. The existing
  /// values are unchanged if `v` is greater than or equal to `num_verts()`.
  void resize(int num_verts) {
    DRAKE_DEMAND(num_verts >= 0);
    num_verts_ = num_verts;
    v0_.conservativeResize(num_verts * SpatialDim);
    v_.conservativeResize(num_verts * SpatialDim);
    x0_.conservativeResize(num_verts * SpatialDim);
    x_.conservativeResize(num_verts * SpatialDim);
  }

  /// Getters.
  /// @{
  const VectorX<T>& get_v0() const { return v0_; }

  const VectorX<T>& get_v() const { return v_; }

  const VectorX<T>& get_x0() const { return x0_; }

  const VectorX<T>& get_x() const { return x_; }
  /// @}

  /// Setters. The value provided must match the current size of the states.
  /// @{
  void set_v0(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(value.size() == num_verts_ * SpatialDim);
    if (value == v0_) return;
    mark_v0_cache_stale();
    v0_ = value;
  }

  void set_v(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(value.size() == num_verts_ * SpatialDim);
    if (value == v_) return;
    mark_v_cache_stale();
    v_ = value;
  }

  void set_x0(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(value.size() == num_verts_ * SpatialDim);
    if (value == x0_) return;
    mark_x0_cache_stale();
    x0_ = value;
  }

  void set_x(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_DEMAND(value.size() == num_verts_ * SpatialDim);
    if (value == x_) return;
    mark_x_cache_stale();
    x_ = value;
  }
  /// @}

  /// Convenient getters for when the index of the vertex is known.
  /// These methods take the index of the vertex `i` as an argument.
  /// @{
  ConstVectorBlockD get_v0_at(int i) const {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    return v0_.template segment<SpatialDim>(SpatialDim * i);
  }

  ConstVectorBlockD get_v_at(int i) const {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    return v_.template segment<SpatialDim>(SpatialDim * i);
  }

  ConstVectorBlockD get_x0_at(int i) const {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    return x0_.template segment<SpatialDim>(SpatialDim * i);
  }

  ConstVectorBlockD get_x_at(int i) const {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    return x_.template segment<SpatialDim>(SpatialDim * i);
  }
  /// @}

  /// Convenient setters for when the index of the vertex is known.
  /// These methods take the index of the vertex `v` as an argument.
  void set_v0_at(int i, const Eigen::Ref<const VectorD>& value) {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    if (value == get_v0_at(i)) return;
    mark_v0_cache_stale();
    v0_.template segment<SpatialDim>(SpatialDim * i) = value;
  }

  void set_v_at(int i, const Eigen::Ref<const VectorD>& value) {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    if (value == get_v_at(i)) return;
    mark_v_cache_stale();
    v_.template segment<SpatialDim>(SpatialDim * i) = value;
  }

  void set_x0_at(int i, const Eigen::Ref<const VectorD>& value) {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    if (value == get_x0_at(i)) return;
    mark_x0_cache_stale();
    x0_.template segment<SpatialDim>(SpatialDim * i) = value;
  }

  void set_x_at(int i, const Eigen::Ref<const VectorD>& value) {
    DRAKE_DEMAND(0 <= i);
    DRAKE_DEMAND(i < num_verts_);
    if (value == get_x_at(i)) return;
    mark_x_cache_stale();
    x_.template segment<SpatialDim>(SpatialDim * i) = value;
  }
  /// @}

  /// Mutable Getters. The value of the states is mutable but the size of the
  /// states is not allowed to change.
  /// @{
  Eigen::VectorBlock<VectorX<T>> get_mutable_v0() {
    mark_v0_cache_stale();
    return v0_.head(v0_.rows());
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_v() {
    mark_v_cache_stale();
    return v_.head(v_.rows());
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_x0() {
    mark_x0_cache_stale();
    return x0_.head(x0_.rows());
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_x() {
    mark_x_cache_stale();
    return x_.head(x_.rows());
  }
  /// @}

  /// Getters and mutable getters for cached quantities.
  /// @{
  const std::vector<std::unique_ptr<ElementCache<T, SpatialDim>>>& get_cache()
      const {
    return cache_;
  }

  /// (Advanced) The caller is responsible for correctly setting the staleness
  /// flag for the cached quantities.
  std::vector<std::unique_ptr<ElementCache<T, SpatialDim>>>& get_mutable_cache()
      const {
    return cache_;
  }

  const std::unique_ptr<ElementCache<T, SpatialDim>>& get_cache_at(
      int e) const {
    DRAKE_DEMAND(e < static_cast<int>(cache_.size()));
    DRAKE_DEMAND(0 <= e);
    return cache_[e];
  }

  /// (Advanced) The caller is responsible for correctly setting the staleness
  /// flag for the cached quantities.
  std::unique_ptr<ElementCache<T, SpatialDim>>& get_mutable_cache_at(
      int e) const {
    DRAKE_DEMAND(e < static_cast<int>(cache_.size()));
    DRAKE_DEMAND(0 <= e);
    return cache_[e];
  }
  /// @}

 private:
  void mark_v0_cache_stale() {
    for (int e = 0; e < static_cast<int>(cache_.size()); ++e) {
      cache_[e]->mark_v0_cache_stale();
    }
  }

  void mark_v_cache_stale() {
    for (int e = 0; e < static_cast<int>(cache_.size()); ++e) {
      cache_[e]->mark_v_cache_stale();
    }
  }

  void mark_x0_cache_stale() {
    for (int e = 0; e < static_cast<int>(cache_.size()); ++e) {
      cache_[e]->mark_x0_cache_stale();
    }
  }

  void mark_x_cache_stale() {
    for (int e = 0; e < static_cast<int>(cache_.size()); ++e) {
      cache_[e]->mark_x_cache_stale();
    }
  }

  // Number of vertices.
  int num_verts_{0};
  // Vertex velocity from previous time step.
  VectorX<T> v0_;
  // Vertex velocities.
  VectorX<T> v_;
  // Vertex position from previous time step.
  VectorX<T> x0_;
  // Vertex positions.
  VectorX<T> x_;
  // Owned cached quantities.
  mutable std::vector<std::unique_ptr<ElementCache<T, SpatialDim>>> cache_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
