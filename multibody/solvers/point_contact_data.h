#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace solvers {

template <typename T>
class PointContactData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointContactData)

  PointContactData(const VectorX<T>* phi0, const LinearOperator<T>* Jc,
                   const VectorX<T>* stiffness, const VectorX<T>* dissipation,
                   const VectorX<T>* mu)
      : phi0_(phi0),
        Jc_(Jc),
        stiffness_(stiffness),
        dissipation_(dissipation),
        mu_(mu) {
    DRAKE_DEMAND(phi0 != nullptr);
    DRAKE_DEMAND(Jc != nullptr);
    DRAKE_DEMAND(stiffness != nullptr);
    DRAKE_DEMAND(dissipation != nullptr);
    DRAKE_DEMAND(mu != nullptr);
    DRAKE_DEMAND(Jc->rows() == 3 * phi0->size());
    DRAKE_DEMAND(stiffness->size() == phi0->size());
    DRAKE_DEMAND(dissipation->size() == phi0->size());
    DRAKE_DEMAND(mu->size() == phi0->size());
    nc_ = phi0->size();
  }

  int num_contacts() const { return nc_; }

  /// @anchor point_contact_data_accessors
  /// @name   Data getters
  /// These method provide access to the underlying contact data. We purposedely
  /// use the naming convention `get_foo()` so that calling code can still use
  /// `foo` as a variable name.
  /// @{
  const VectorX<T>& get_phi0() const { return *phi0_; };
  const LinearOperator<T>& get_Jc() const { return *Jc_; }
  const VectorX<T>& get_stiffness() const { return *stiffness_; };
  const VectorX<T>& get_dissipation() const { return *dissipation_; };
  const VectorX<T>& get_mu() const { return *mu_; };
  /// @}

 private:
  int nc_{0};
  const VectorX<T>* phi0_{nullptr};
  const LinearOperator<T>* Jc_{nullptr};
  const VectorX<T>* stiffness_{nullptr};
  const VectorX<T>* dissipation_{nullptr};
  const VectorX<T>* mu_{nullptr};
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake