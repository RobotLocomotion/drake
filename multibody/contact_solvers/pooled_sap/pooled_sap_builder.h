#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
class PooledSapBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PooledSapBuilder);

  explicit PooledSapBuilder(const MultibodyPlant<T>& plant);

  PooledSapBuilder(const MultibodyPlant<T>& plant,
                   const systems::Context<T>& context);

  void UpdateModel(const systems::Context<T>& context, const T& time_step,
                   bool reuse_geometry_data, PooledSapModel<T>* model) const;

  /* Updates the dynamics matrix A and the linear term r to incorporate modeling
   of external forces according to:
     τ = −Kₑ⋅v + bₑ
   where Kₑ is a positive semi-definite gain matrix and bₑ a bias term. Matrix
   Kₑ is diagonal.

   More specifically, the model is updated according to:
     A += δt⋅Kₑ
     r += δt⋅bₑ

   Kₑ and bₑ are provided for the entire model. Kₑ can have zero entries.

   @pre Ke has positive (or zero) entries.
   @pre Both Ke and be are vectors of size model->num_velocities(). */
  void AddExternalGains(const VectorX<T>& Ke, const VectorX<T>& be,
                        PooledSapModel<T>* model) const;

  /* Adds constraints to model actuation with effort limits according to:
     τ = clamp(−Kᵤ⋅v + bᵤ, e)
    where Kᵤ is a positive semi-definite gain matrix,  bᵤ a bias term and e the
    effort limit. Matrix Kᵤ is diagonal.

    Kᵤ and bᵤ are provided for the entire model and Kᵤ can have zero entries.
    The effort limits are obtained from the plant() model provided at
    construction of `this` builder.
    Constraints are added on a per-clique basis, and no actuation constraints
    are added if a clique has no actuators. */
  void AddActuationGains(const VectorX<T>& Ku, const VectorX<T>& bu,
                         PooledSapModel<T>* model) const;

  const MultibodyPlant<T>& plant() const { return *plant_; }

 private:
  void AccumulateForceElementForces(const systems::Context<T>& context,
                                    VectorX<T>* r) const;
  void CalcActuationInput(const systems::Context<T>& context,
                          VectorX<T>* actuation_w_pd,
                          VectorX<T>* actuation_wo_pd) const;
  void CalcGeometryContactData(const systems::Context<T>& context) const;
  void AddPatchConstraintsForPointContact(const systems::Context<T>& context,
                                          PooledSapModel<T>* model) const;
  void AddPatchConstraintsForHydroelasticContact(
      const systems::Context<T>& context, PooledSapModel<T>* model) const;
  void AddCouplerConstraints(const systems::Context<T>& context,
                             PooledSapModel<T>* model) const;
  void AddLimitConstraints(const systems::Context<T>& context,
                           PooledSapModel<T>* model) const;

  const MultibodyPlant<T>* plant_{nullptr};

  /* Model properties that do not change unless the system changes. */
  std::map<geometry::GeometryId, CoulombFriction<double>> friction_;
  std::map<geometry::GeometryId, T> stiffness_;    // point contact.
  std::map<geometry::GeometryId, T> dissipation_;  // H&C dissipation.

  /* Scratch workspace data to build the model.  */
  struct Scratch {
    MatrixX<T> M;        // Dense mass matrix computed by MbP.
    Matrix6X<T> J_V_WB;  // Dense spatial velocity Jacobian.
    VectorX<T> u_no_pd;
    VectorX<T> u_w_pd;
    VectorX<T> tmp_v1;  // Scratch of size num_velocities.
    std::unique_ptr<MultibodyForces<T>> forces;
    std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
    std::vector<geometry::ContactSurface<T>> surfaces;
  };
  /* This space is not intended for long-term storage and is often cleared or
   overwritten as needed. */
  mutable Scratch scratch_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapBuilder);
