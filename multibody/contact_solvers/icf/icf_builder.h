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
#include "drake/multibody/contact_solvers/icf/icf.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {

// Represent a linear feedback law of the form
//   τ = −K⋅v + b,
// where K is non-negative and diagonal.
template <typename T>
struct LinearFeedbackGains {
  VectorX<T> K;
  VectorX<T> b;

  void resize(int nv) {
    K.resize(nv);
    b.resize(nv);
  }
};

template <typename T>
class IcfBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfBuilder);

  explicit IcfBuilder(const MultibodyPlant<T>& plant);

  IcfBuilder(const MultibodyPlant<T>& plant,
             const systems::Context<T>& context);

  /**
   * Update the given IcfModel to represent the ICF problem
   *
   *     min ℓ(v; q₀, v₀, δt)
   *
   * @param context The MbP context storing the current state (q₀, v₀).
   * @param time_step The requested time step δt.
   * @param actuation_feedback Optional linearization data (Kᵤ, bᵤ) for
   * actuation forces τ = clamp(-Kᵤ⋅v + b, e).
   * @param external_feedback Optional linearization data (Kₑ, bₑ) for external
   * forces τ = -Kₑ⋅v + bₑ.
   * @param model The IcfModel to update.
   */
  void UpdateModel(const systems::Context<T>& context, const T& time_step,
                   std::optional<LinearFeedbackGains<T>> actuation_feedback,
                   std::optional<LinearFeedbackGains<T>> external_feedback,
                   IcfModel<T>* model);

  // Ignore actuation and external force constraints.
  void UpdateModel(const systems::Context<T>& context, const T& time_step,
                   IcfModel<T>* model) {
    UpdateModel(context, time_step, std::nullopt, std::nullopt, model);
  }

  // Only update the time step δt. All other model data remains unchanged.
  void UpdateModel(const T& time_step, IcfModel<T>* model) const;

 private:
  // Compute geometry data and store it internally for later use
  void CalcGeometryContactData(const systems::Context<T>& context);

  // Allocate space for both point contact and hydroelastic contact constraints
  // @pre Geometry contact data has already been computed
  void AllocatePatchConstraints(IcfModel<T>* model) const;

  // Set point contact constraints in the model
  void SetPatchConstraintsForPointContact(const systems::Context<T>& context,
                                          IcfModel<T>* model) const;

  // Set hydroelastic contact constraints in the model
  void SetPatchConstraintsForHydroelasticContact(
      const systems::Context<T>& context, IcfModel<T>* model) const;

  // Coupler constraints
  void AllocateCouplerConstraints(IcfModel<T>* model) const;
  void SetCouplerConstraints(const systems::Context<T>& context,
                             IcfModel<T>* model) const;

  // Joint limit constraints
  void AllocateLimitConstraints(IcfModel<T>* model) const;
  void SetLimitConstraints(const systems::Context<T>& context,
                           IcfModel<T>* model) const;

  // Allocate space for gain constraints. We assume that external force
  // constraints come first, followed by actuator constraints.
  void AllocateGainConstraints(IcfModel<T>* model, bool actuation,
                               bool external_forces) const;

  // External force constraints τ = −Kₑ⋅v + bₑ, where Kₑ is diagonal and >= 0.
  void SetExternalGainConstraints(const VectorX<T>& Ke, const VectorX<T>& be,
                                  IcfModel<T>* model) const;

  // Actuation constraints τ = clamp(−Kᵤ⋅v + bᵤ, e), where Kᵤ is diagonal
  // and >= 0. Note that effort limits e are enforced here.
  void SetActuationGainConstraints(const VectorX<T>& Ku, const VectorX<T>& bu,
                                   bool has_external_forces,
                                   IcfModel<T>* model) const;

  // The multibody plant used to build the model.
  const MultibodyPlant<T>& plant() const { return *plant_; }
  const MultibodyPlant<T>* plant_{nullptr};

  // Map a tree index to a clique index. Cliques are trees with nv > 0.
  // If a tree has nv == 0, it maps to -1.
  int tree_to_clique(int tree_index) const {
    DRAKE_ASSERT(tree_index >= 0 && tree_index < ssize(tree_to_clique_));
    return tree_to_clique_[tree_index];
  }

  // Model properties that do not change unless the system changes.
  std::map<geometry::GeometryId, CoulombFriction<double>> friction_;
  std::map<geometry::GeometryId, T> stiffness_;    // point contact.
  std::map<geometry::GeometryId, T> dissipation_;  // H&C dissipation.
  std::vector<int> tree_to_clique_;      // cliques are trees with nv > 0.
  std::vector<int> clique_sizes_;        // nv for each clique.
  std::vector<int> body_jacobian_cols_;  // cols of J_WB for each body.
  std::vector<int> body_to_clique_;      // clique index for each body.
  std::vector<int> body_is_floating_;    // 1 if body is floating, 0 otherwise.
  std::vector<T> body_mass_;             // mass of each body.
  VectorX<T> effort_limits_;             // actuator limits for each velocity.
  std::vector<int> clique_nu_;           // number of actuators per clique.

  std::vector<int> limited_clique_sizes_;        // nv in each limited clique.
  std::vector<int> clique_to_limit_constraint_;  // clique idx <--> limit idx
  std::vector<int> limit_constraint_to_clique_;

  // Internal storage for geometry query results.
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs_;
  std::vector<geometry::ContactSurface<T>> surfaces_;

  // Scratch workspace data to build the model.
  struct Scratch {
    MatrixX<T> M;        // Dense mass matrix computed by MbP.
    Matrix6X<T> J_V_WB;  // Dense spatial velocity Jacobian.
    VectorX<T> tmp_v1;   // Scratch of size num_velocities.
    std::unique_ptr<MultibodyForces<T>> forces;
  };
  mutable Scratch scratch_;
};

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::IcfBuilder);
