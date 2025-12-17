#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/contact_solvers/icf/icf_linear_feedback_gains.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
class IcfBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfBuilder);

  explicit IcfBuilder(const MultibodyPlant<T>& plant);

  /* Updates the given IcfModel to represent the ICF problem

      min ℓ(v; q₀, v₀, δt)

  @param context The MbP context storing the current state (q₀, v₀).
  @param time_step The requested time step δt.
  @param actuation_feedback Optional linearization data (Kᵤ, bᵤ) for
         actuation forces τ = clamp(-Kᵤ⋅v + b, e).
  @param external_feedback Optional linearization data (Kₑ, bₑ) for external
         forces τ = -Kₑ⋅v + bₑ.
  @param model The IcfModel to update.

  TODO(#23912): make sure to test that UpdateModel limits heap allocations. */
  void UpdateModel(const systems::Context<T>& context, const T& time_step,
                   const IcfLinearFeedbackGains<T>* actuation_feedback,
                   const IcfLinearFeedbackGains<T>* external_feedback,
                   IcfModel<T>* model);

  /* Updates the IcfModel for a problem without actuation or external force
  constraints. */
  void UpdateModel(const systems::Context<T>& context, const T& time_step,
                   IcfModel<T>* model) {
    UpdateModel(context, time_step, nullptr, nullptr, model);
  }

  /* Updates only the time step δt. All other model data remains unchanged. */
  void UpdateModel(const T& time_step, IcfModel<T>* model) const;

  /* Updates only the time step δt and feedback gains. All other model data
  remains unchanged.
  @pre The existence (i.e., nullness) of the two feedbacks values must match the
  existence of the most recent prior call to UpdateModel(). */
  void UpdateModel(const T& time_step,
                   const IcfLinearFeedbackGains<T>* actuation_feedback,
                   const IcfLinearFeedbackGains<T>* external_feedback,
                   IcfModel<T>* model) const;

 private:
  /* Scratch workspace data to build the model. */
  struct Scratch {
    explicit Scratch(const MultibodyPlant<T>& plant);

    Matrix6X<T> J_V_WB;        // size 6 x nv
    VectorX<T> accelerations;  // size nv
    MultibodyForces<T> forces;
  };

  /* Throws if the plant provided at construction is not compatible with ICF. */
  void ValidatePlant();

  /* Computes geometry data and store it internally for later use. */
  void CalcGeometryContactData(const systems::Context<T>& context);

  /* Allocates space for both point and hydroelastic contact constraints.
  @pre Geometry contact data has already been computed */
  void AllocatePatchConstraints(IcfModel<T>* model) const;

  /* Sets point contact constraints in the model
  @pre AllocatePatchConstraints() has already been called. */
  void SetPatchConstraintsForPointContact(const systems::Context<T>& context,
                                          IcfModel<T>* model) const;

  /* Sets hydroelastic contact constraints in the model
  @pre AllocatePatchConstraints() has already been called. */
  void SetPatchConstraintsForHydroelasticContact(
      const systems::Context<T>& context, IcfModel<T>* model) const;

  /* Resizes the model to accommodate coupler constraints. */
  void AllocateCouplerConstraints(IcfModel<T>* model) const;

  /* Sets coupler constraints in the model.
  @pre AllocateCouplerConstraints() has already been called. */
  void SetCouplerConstraints(const systems::Context<T>& context,
                             IcfModel<T>* model) const;

  /* Resizes the model to accommodate limit constraints. */
  void AllocateLimitConstraints(IcfModel<T>* model) const;

  /* Sets limit constraints in the model.
  @pre AllocateLimitConstraints() has already been called. */
  void SetLimitConstraints(const systems::Context<T>& context,
                           IcfModel<T>* model) const;

  /* Resizes the model to accommodate gain constraints. We assume that external
  force constraints come first, followed by actuator constraints. */
  void AllocateGainConstraints(IcfModel<T>* model, bool actuation,
                               bool external_forces) const;

  /* Sets external force constraints τ = −Kₑ⋅v + bₑ, where Kₑ is diagonal and
  non-negative.
  @pre AllocateGainConstraints() has already been called. */
  void SetExternalGainConstraints(const VectorX<T>& Ke, const VectorX<T>& be,
                                  IcfModel<T>* model) const;

  /* Sets actuation constraints τ = clamp(−Kᵤ⋅v + bᵤ, e), where Kᵤ is diagonal
  and non-negative. Note that effort limits e are enforced here.
  @pre AllocateGainConstraints() has already been called. */
  void SetActuationGainConstraints(const VectorX<T>& Ku, const VectorX<T>& bu,
                                   bool has_external_forces,
                                   IcfModel<T>* model) const;

  /* Maps a tree index to a clique index. Cliques are trees with nv > 0. If a
  tree has nv == 0, its clique index is -1. */
  int tree_to_clique(int tree_index) const {
    DRAKE_ASSERT(tree_index >= 0 && tree_index < ssize(tree_to_clique_));
    return tree_to_clique_[tree_index];
  }

  void RefreshGeometryDetails(const systems::Context<T>& context) const;

  const MultibodyPlant<T>& plant_;

  // Model properties that do not change unless the system changes.
  std::vector<int> tree_to_clique_;      // cliques are trees with nv > 0.
  std::vector<int> clique_sizes_;        // nv for each clique.
  std::vector<int> body_jacobian_cols_;  // cols of J_WB for each body.
  std::vector<int> body_to_clique_;      // clique index for each body.
  std::vector<int> body_is_floating_;    // 1 if body is floating, 0 otherwise.
  std::vector<T> body_mass_;             // mass of each body.
  VectorX<T> effort_limits_;             // actuator limits for each velocity.
  std::vector<int> clique_nu_;           // number of actuators per clique.
  int num_actuation_constraints_{};      // count of clique_nu_[k] > 0.

  std::vector<int> limited_clique_sizes_;        // nv in each limited clique.
  std::vector<int> clique_to_limit_constraint_;  // clique idx --> limit idx.

  // Cache of geometry properties. This is invalidated by changes of
  // geometry_version_;
  struct GeometryDetails {
    CoulombFriction<double> friction;
    T stiffness;    // point contact.
    T dissipation;  // H&C dissipation.
  };
  mutable std::unordered_map<geometry::GeometryId, GeometryDetails>
      geometry_details_;
  mutable geometry::GeometryVersion geometry_version_;

  // Internal storage for geometry query results.
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs_;
  std::vector<geometry::ContactSurface<T>> surfaces_;

  // Storage for intermediate computations, often overwritten.
  Scratch scratch_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfBuilder);
