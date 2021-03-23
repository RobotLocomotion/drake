#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/fixed_fem/dev/dirichlet_boundary_condition.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

template <typename T>
class DirichletBoundaryCondition;

/** An abstract state class that stores the fem states. The states include the
 generalized positions associated with each node, `q`, and optionally, their
 first and second time derivatives, `qdot` and `qddot`.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class FemStateBase {
 public:
  virtual ~FemStateBase() = default;

  /** @name State getters. Throw an exception if the state doesn't exist.
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
   Throw an exception otherwise. Throw an exception if the state being set does
   not exist.
   @{ */
  void SetQ(const Eigen::Ref<const VectorX<T>>& value);

  void SetQdot(const Eigen::Ref<const VectorX<T>>& value);

  void SetQddot(const Eigen::Ref<const VectorX<T>>& value);
  /** @} */

  // TODO(xuchenhan-tri): Change the API to calculate the norm of the unknown
  //  instead.
  /** Calculates the norm of the state with the highest order. */
  T HighestOrderStateNorm() const { return get_highest_order_state().norm(); }

  // TODO(xuchenhan-tri): Change the API to get the the unknown state
  //  instead.
  const VectorX<T>& get_highest_order_state() const {
    if (ode_order() == 0) return q_;
    if (ode_order() == 1) return qdot_;
    if (ode_order() == 2) return qddot_;
    DRAKE_UNREACHABLE();
  }

  int num_generalized_positions() const { return q_.size(); }

  /** The order of the ODE problem after FEM spatial discretization. */
  int ode_order() const { return ode_order_; }

  // TODO(xuchenhan-tri): Consider whether this method should belong in
  //  DirichletBoundaryCondition along with the methods that apply the BC to
  //  residuals and tangent matrices.
  /** Modifies `this` FEM state so that it complies with the given boundary
   conditions.
   @throw std::exception if the any of the indexes of the dofs under the
   boundary condition specified by the given DirichletBoundaryCondition does
   not exist in `this` FEM state`. */
  void ApplyBoundaryCondition(const DirichletBoundaryCondition<T>& bc);

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemStateBase);

  /** Constructs an %FemStateBase of a zero-th order equation with prescribed
   generalized positions.
   @param[in] q    The prescribed generalized positions. */
  explicit FemStateBase(const Eigen::Ref<const VectorX<T>>& q)
      : ode_order_(0), q_(q) {
    DRAKE_DEMAND(qdot_.size() == 0);
    DRAKE_DEMAND(qddot_.size() == 0);
  }

  /** Constructs an %FemStateBase of a first order equation with prescribed
   generalized positions and their time derivatives.
   @param[in] q    The prescribed generalized positions.
   @param[in] qdot    The prescribed time derivatives of generalized positions.
   @pre q.size() == qdot.size(). */
  FemStateBase(const Eigen::Ref<const VectorX<T>>& q,
               const Eigen::Ref<const VectorX<T>>& qdot)
      : ode_order_(1), q_(q), qdot_(qdot) {
    DRAKE_DEMAND(q_.size() == qdot_.size());
    DRAKE_DEMAND(qddot_.size() == 0);
  }

  /** Constructs an %FemStateBase of a second order equation with prescribed
   generalized positions and their first and second order time derivatives.
   @param[in] q    The prescribed generalized positions.
   @param[in] qdot    The prescribed time derivatives of generalized positions.
   @param[in] qddot    The prescribed time second derivatives of generalized
   positions.
   @pre q.size() == qdot.size().
   @pre q.size() == qddot.size(). */
  FemStateBase(const Eigen::Ref<const VectorX<T>>& q,
               const Eigen::Ref<const VectorX<T>>& qdot,
               const Eigen::Ref<const VectorX<T>>& qddot)
      : ode_order_(2), q_(q), qdot_(qdot), qddot_(qddot) {
    DRAKE_DEMAND(q_.size() == qdot_.size());
    DRAKE_DEMAND(q_.size() == qddot_.size());
  }

 private:
  /* Invalidate state-dependent quantities. Should be called on state changes.
   */
  virtual void InvalidateAllCacheEntries() = 0;

  /* The order of ODE arising from the discretized FEM model. */
  int ode_order_{-1};
  /* Generalized node positions. */
  VectorX<T> q_{};
  /* Time derivatives of generalized node positions. */
  VectorX<T> qdot_{};
  /* Time second derivatives of generalized node positions. */
  VectorX<T> qddot_{};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::FemStateBase);
