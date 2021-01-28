#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/linear_operator.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// This class specifies the set of parameters needed to describe contact
// constraints as needed by ContactSolver. Refer to ContactSolver's class
// documentation for details.
template <typename T>
class PointContactData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointContactData)

  // Specifies the set of possible discrete contacts characterized by:
  //  1. Signed distance phi0, negative when objects interpenetrate.
  //  2. Contact Jacobian s.t. contact velocity is given by vc = Jc⋅v. Refer to
  //     MergeNormalAndTangent() for the storage format of vc.
  //  3. Coefficients of Coulomb friction.
  //  4. Compliance. Refer to each solver's specific documentation for details.
  //     Usually `stiffness` refers to linear spring stiffness with units of
  //     N/m for each contact and `dissipation` refers to Hunt & Crossley
  //     dissipation with units of s/m.
  //
  // The number of velocities is denoted with nv and the number of discrete
  // contacts is denoted with nc.
  //
  // @param phi0 Signed distances before contact, of size nc.
  // @param Jc Contact Jacobian of size 3nc x nv.
  // @param stiffness Linear spring constants, of size nc.
  // @param dissipation Dissipation constants, of size nc.
  // @param mu Coefficients of Coulomb friction, of size nc.
  //
  // @pre Data pointers must not be nullptr and must point to data with the
  // documented sizes.
  //
  // @pre Jc must implement DoMultiplyByTranspose().
  //
  // This class will keep a reference to the input data and therefore it is
  // required that it outlives this object.
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

  // Returns the number of contacts nc in accordance to the data provided at
  // construction.
  int num_contacts() const { return nc_; }

  // @anchor point_contact_data_accessors
  // @name   Data getters
  // These methods provide access to the underlying contact data. We
  // purposely use the naming convention `get_foo()` so that calling code can
  // still use `foo` as a variable name.
  // @{
  const VectorX<T>& get_phi0() const { return *phi0_; }
  const LinearOperator<T>& get_Jc() const { return *Jc_; }
  const VectorX<T>& get_stiffness() const { return *stiffness_; }
  const VectorX<T>& get_dissipation() const { return *dissipation_; }
  const VectorX<T>& get_mu() const { return *mu_; }
  // @}

 private:
  int nc_{0};
  const VectorX<T>* phi0_{nullptr};
  const LinearOperator<T>* Jc_{nullptr};
  const VectorX<T>* stiffness_{nullptr};
  const VectorX<T>* dissipation_{nullptr};
  const VectorX<T>* mu_{nullptr};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::PointContactData)
