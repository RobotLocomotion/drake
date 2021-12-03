#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// This class specifies the dynamics of the physical system as needed by
// ContactSolver. Refer to ContactSolver's class documentation for details.
template <typename T>
class SystemDynamicsData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemDynamicsData)

  // Specifies the dynamics of the system by providing Ainv, a linear operator
  // form of the inverse of the system dynamics Jacobian matrix A and the
  // vector of predicted generalized velocities v^*. Refer to ContactSolver for
  // details. This class will keep a reference to the input data Ainv and
  // v_star and therefore it is required that they outlive this object.
  //
  // @param Ainv The system's dynamics matrix inverse operator. Of size nv x
  // nv, with nv = num_velocities().
  // @param v_star Predictor's step velocity v*, of size nv.
  //
  // N.B. This object stores references to both Ainv and v_star and therefore
  // they must outlive this object's lifetime.
  //
  // @pre data must not be nullptr and must point to data with the documented
  // sizes.
  SystemDynamicsData(const LinearOperator<T>* Ainv, const VectorX<T>* v_star);

  // Operator A describes the inverse dynamics as A⋅(v−v*) = τ while
  // Ainv (A⁻¹) describes the forward dynamics as v = v* + A⁻¹⋅τ. Operator A and
  // its inverse have size nv x nv, with nv = num_velocities().
  // Either can be nullptr meaning that the operator is not available (some
  // formulations do not require it), see has_forward_dynamics() and
  // has_inverse_dynamics().
  //
  // @param A Operator describing the inverse dynamics of the system.
  // @param Ainv The inverse of operator A, describing the forward dynamics of
  // the system.
  // @param v_star Predictor's step velocity v*, of size nv.
  //
  // N.B. This object stores references to both the operators and v_star and
  // therefore they must outlive this object's lifetime.
  //
  // @pre At least one of A or Ainv must be a valid, non null, pointer.
  // @pre If both operators are provided, they both must have the same size.
  // @pre the provided operators must be square.
  // @pre v_star must have the same size as the provided operators.
  SystemDynamicsData(const LinearOperator<T>* A, const LinearOperator<T>* Ainv,
                     const VectorX<T>* v_star);

  // Returns the number of generalized velocities nv in accordance to the data
  // provided at construction.
  int num_velocities() const { return nv_; }

  // Forward dynamics operator Ainv was provided at construction.
  bool has_forward_dynamics() const { return Ainv_ != nullptr; }

  // Inverse dynamics operator A was provided at construction.
  bool has_inverse_dynamics() const { return A_ != nullptr; }

  /// Retrieve inverse dynamics operator A.
  const LinearOperator<T>& get_A() const {
    DRAKE_DEMAND(A_ != nullptr);
    return *A_;
  }

  /// Retrieve forward dynamics operator A⁻¹.
  const LinearOperator<T>& get_Ainv() const {
    DRAKE_DEMAND(Ainv_ != nullptr);
    return *Ainv_;
  }

  // Retrieve predicted velocity v*.
  const VectorX<T>& get_v_star() const { return *v_star_; }

 private:
  int nv_{0};
  const LinearOperator<T>* A_{nullptr};     // inverse dynamics.
  const LinearOperator<T>* Ainv_{nullptr};  // forward dynamics.
  const VectorX<T>* v_star_{nullptr};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SystemDynamicsData)
