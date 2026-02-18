#pragma once

#include <unordered_map>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
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

/* IcfBuilder constructs and updates an ICF problem, given a MultibodyPlant, its
Context, and information about external and actuation forces.
*/
template <typename T>
class IcfBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfBuilder);

  /* @param plant The plant from which to construct an ICF problem.
  @pre plant is finalized.

  IcfBuilder retains an alias to the plant, so the passed plant must outlive the
  builder.
  */
  explicit IcfBuilder(const MultibodyPlant<T>* plant);

  /* Updates the given IcfModel to represent the ICF problem

      min ℓ(v; q₀, v₀, δt)

  @param context The plant context storing the current state (q₀, v₀).
  @param time_step The requested time step δt.
  @param actuation_feedback linearization data (Kᵤ, bᵤ) for
         actuation forces τᵤ = clamp(-Kᵤ⋅v + b, e). May be nullptr.
  @param external_feedback linearization data (Kₑ, bₑ) for external
         forces τₑ = -Kₑ⋅v + bₑ. May be nullptr.
  @param model The IcfModel to update.

  Note that the presence/absence of actuation_feedback and external_feedback
  affects the allocation of gain constraints. In particular, passing nullptr to
  one of the feedback arguments will erase any previously constructed gain
  constraints for that set of forces. Passing a non-null argument on a
  subsequent step will cause the gain constraints for the relevant forces to
  rebuilt.

  TODO(#23912): make sure to test that UpdateModel limits heap allocations. */
  void UpdateModel(const systems::Context<T>& context, const T& time_step,
                   const IcfLinearFeedbackGains<T>* actuation_feedback,
                   const IcfLinearFeedbackGains<T>* external_feedback,
                   IcfModel<T>* model);

  /* Updates only the feedback gains. All other model data remains unchanged.
  @pre The existence (i.e., nullness) of the two feedbacks values must match the
  existence of the most recent prior call to UpdateModel(). */
  void UpdateFeedbackGains(const IcfLinearFeedbackGains<T>* actuation_feedback,
                           const IcfLinearFeedbackGains<T>* external_feedback,
                           IcfModel<T>* model) const;

 private:
  /* Scratch workspace data to build the model. */
  struct Scratch {
    explicit Scratch(const MultibodyPlant<T>& plant);

    VectorX<T> effort_limits;        // size nv
    Matrix6X<T> J_V_WB;              // size 6 x nv
    const VectorX<T> accelerations;  // size nv
    MultibodyForces<T> forces;
  };

  /* Sort bodies in a pair so that the second one is guaranteed to be not
  anchored. By convention, body B is always not-anchored. */
  struct BodiesSortedByAnchorage {
    BodiesSortedByAnchorage(const MultibodyPlant<T>& plant,
                            const RigidBody<T>* bodyM,
                            const RigidBody<T>* bodyN);
    const RigidBody<T>* bodyA{};
    const RigidBody<T>* bodyB{};
  };

  /* Throws if the plant provided at construction is not compatible with ICF. */
  void ValidatePlant();

  /* Throws if the context passed to UpdateModel is not compatible with ICF. */
  void ValidateContext(const systems::Context<T>& context);

  /* Computes geometry data and stores it internally for later use. */
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
    DRAKE_ASSERT(tree_index >= 0 &&
                 tree_index < ssize(plant_facts_.tree_to_clique));
    return plant_facts_.tree_to_clique[tree_index];
  }

  /* Update the GeometryDetails cache, only if it is stale as determined by
  checking geometry versions .*/
  void RefreshGeometryDetails(
      const geometry::SceneGraphInspector<T>& inspector) const;

  const MultibodyPlant<T>& plant_;

  // Model properties that cannot change since the plant is finalized.
  struct PlantFacts {
    explicit PlantFacts(const MultibodyPlant<T>& plant);

    std::vector<int> tree_to_clique;        // cliques are trees with nv > 0.
    std::vector<int> clique_sizes;          // nv for each clique.
    std::vector<int> body_jacobian_cols;    // cols of J_WB for each body.
    std::vector<int> body_to_clique;        // clique index for each body.
    std::vector<int> body_is_floating;      // 1 if body is floating, else 0.
    std::vector<int> clique_nu;             // number of actuators per clique.
    int num_actuation_constraints{};        // count of clique_nu_[k] > 0.
    std::vector<int> limited_clique_sizes;  // nv in each limited clique.
    std::vector<int> clique_to_limit_constraint;  // clique idx --> limit idx.
  };
  const PlantFacts plant_facts_{plant_};

  // Cache of geometry properties, invalidated by changes of geometry version.
  struct GeometryDetails {
    CoulombFriction<double> friction;
    T stiffness;    // point contact.
    T dissipation;  // H&C dissipation.
  };
  mutable std::unordered_map<geometry::GeometryId, GeometryDetails>
      geometry_details_;
  mutable geometry::GeometryVersion geometry_version_;

  // Local copy of the current geometry query results, updated by
  // CalcGeometryContactData and used by the patch constraint.
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs_;
  std::vector<geometry::ContactSurface<T>> surfaces_;

  // Storage for intermediate computations, often overwritten.
  mutable Scratch scratch_{plant_};
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfBuilder);
