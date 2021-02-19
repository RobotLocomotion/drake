#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Stores the fem model state and per-element state-dependent quantities.The
 states include the generalized positions associated with each node, `q`, and
 optionally, their first and second time derivatives, `qdot` and `qddot`.
 %FemState also stores the per-element state-dependent quantities for its
 corresponding elements (see ElementCacheEntry).
 @tparam Element The type of FemElement that consumes this %FemState. This
 template parameter provides the scalar type, the type of per-element data
 this %FemState stores and the order of the ODE after FEM spatial
 discretization. */
template <typename Element>
class FemState {
 public:
  using T = typename Element::T;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  /** The order of the ODE problem after FEM spatial discretization. */
  static constexpr int ode_order() {
    constexpr int order = Element::Traits::kOdeOrder;
    static_assert(order == 0 || order == 1 || order == 2);
    return order;
  }

  /** Constructs an %FemState of a zero-th order equation with prescribed
   generalized positions.
   @param[in] q    The prescribed generalized positions.
   @pre ode_order() == 0. */
  explicit FemState(const Eigen::Ref<const VectorX<T>>& q) : q_(q) {
    DRAKE_THROW_UNLESS(ode_order() == 0);
    DRAKE_ASSERT(qdot_.size() == 0);
    DRAKE_ASSERT(qddot_.size() == 0);
  }

  /** Constructs an %FemState of a first order equation with prescribed
   generalized positions and their time derivatives.
   @param[in] q    The prescribed generalized positions.
   @param[in] qdot    The prescribed time derivatives of generalized positions.
   @pre ode_order() == 1.
   @pre q.size() == qdot.size(). */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& qdot)
      : q_(q), qdot_(qdot) {
    DRAKE_THROW_UNLESS(ode_order() == 1);
    DRAKE_THROW_UNLESS(q_.size() == qdot_.size());
    DRAKE_ASSERT(qddot_.size() == 0);
  }

  /** Constructs an %FemState of a second order equation with prescribed
   generalized positions and their first and second order time derivatives.
   @param[in] q    The prescribed generalized positions.
   @param[in] qdot    The prescribed time derivatives of generalized positions.
   @param[in] qddot    The prescribed time second derivatives of generalized
   positions.
   @pre ode_order() == 2.
   @pre q.size() == qdot.size().
   @pre q.size() == qddot.size(). */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& qdot,
           const Eigen::Ref<const VectorX<T>>& qddot)
      : q_(q), qdot_(qdot), qddot_(qddot) {
    DRAKE_THROW_UNLESS(ode_order() == 2);
    DRAKE_THROW_UNLESS(q_.size() == qdot_.size());
    DRAKE_THROW_UNLESS(q_.size() == qddot_.size());
  }

  /** Creates the per-element state-dependent data for the given `elements`. The
  following invariant must be satisfied: `elements[i].element_index() == i` --
  the i-th element reports an element index of `i`, as %FemState assumes this
  invariant when the Element::Traits::Data are accessed via element_data().
  @throw std::exception if elements[i].element_index() != i for some `i` = 0,
  ..., `element.size()-1`. */
  void MakeElementData(const std::vector<Element>& elements) {
    element_cache_.clear();
    for (int i = 0; i < static_cast<int>(elements.size()); ++i) {
      if (elements[i].element_index() != ElementIndex(i)) {
        throw std::runtime_error(
            "Input element entry at " + std::to_string(i) + " has index " +
            std::to_string(elements[i].element_index()) + " instead of " +
            std::to_string(i) +
            ". The entry with index i must be stored at position i.");
      }
    }
    element_cache_.resize(elements.size());
  }

  int num_generalized_positions() const { return q_.size(); }

  int element_cache_size() const { return element_cache_.size(); }
  /** `q`, `qdot`, and `qddot` are resized (if they exist) with the semantics
  outlined in <a
  href="https://eigen.tuxfamily.org/dox/classEigen_1_1PlainObjectBase.html#a78a42a7c0be768374781f67f40c9ab0d">
  Eigen::conservativeResize</a>. */
  void Resize(int num_generalized_positions) {
    DRAKE_ASSERT(num_generalized_positions >= 0);
    q_.conservativeResize(num_generalized_positions);
    if constexpr (ode_order() >= 1)
      qdot_.conservativeResize(num_generalized_positions);
    if constexpr (ode_order() == 2)
      qddot_.conservativeResize(num_generalized_positions);
  }

  /** @name State getters.
   @{ */
  const VectorX<T>& q() const { return q_; }

  const VectorX<T>& qdot() const {
    DRAKE_THROW_UNLESS(ode_order() >= 1);
    return qdot_;
  }

  const VectorX<T>& qddot() const {
    DRAKE_THROW_UNLESS(ode_order() == 2);
    return qddot_;
  }
  /** @} */

  /** @name State setters.
   The size of the values provided must match the current size of the states.
   @{ */
  void set_q(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_THROW_UNLESS(value.size() == q_.size());
    mutable_q() = value;
  }

  void set_qdot(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_THROW_UNLESS(ode_order() >= 1);
    DRAKE_THROW_UNLESS(value.size() == qdot_.size());
    mutable_qdot() = value;
  }

  void set_qddot(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_THROW_UNLESS(ode_order() == 2);
    DRAKE_THROW_UNLESS(value.size() == qddot_.size());
    mutable_qddot() = value;
  }
  /** @} */

  /** @name Mutable state getters.
   The values of the states are mutable but the sizes of the states are not
   allowed to change.
   @{ */
  Eigen::VectorBlock<VectorX<T>> mutable_q() { return q_.head(q_.size()); }

  Eigen::VectorBlock<VectorX<T>> mutable_qdot() {
    DRAKE_THROW_UNLESS(ode_order() >= 1);
    return qdot_.head(qdot_.size());
  }

  Eigen::VectorBlock<VectorX<T>> mutable_qddot() {
    DRAKE_THROW_UNLESS(ode_order() == 2);
    return qddot_.head(qddot_.size());
  }
  /** @} */

  /** Getter for element state-dependpent quantities. */
  const typename Element::Traits::Data& element_data(
      const Element& element) const {
    ElementIndex id = element.element_index();
    DRAKE_ASSERT(id.is_valid() && id < element_cache_size());
    // TODO(xuchenhan-tri): Currently the data is always recomputed when this
    //  method is invoked. Cache these data in the future.
    typename Element::Traits::Data& data =
        element_cache_[id].mutable_element_data();
    data = element.ComputeData(*this);
    return data;
  }

  /** Calculates the norm of the state with the highest order. */
  T HighestOrderStateNorm() const {
    if constexpr (ode_order() == 0) return q_.norm();
    if constexpr (ode_order() == 1) return qdot_.norm();
    if constexpr (ode_order() == 2) return qddot_.norm();
    DRAKE_UNREACHABLE();
  }

 private:
  /* Generalized node positions. */
  VectorX<T> q_{};
  /* Time derivatives of generalized node positions. */
  VectorX<T> qdot_{};
  /* Time second derivatives of generalized node positions. */
  VectorX<T> qddot_{};
  /* Owned element cache entries. */
  mutable std::vector<ElementCacheEntry<typename Element::Traits::Data>>
      element_cache_{};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
