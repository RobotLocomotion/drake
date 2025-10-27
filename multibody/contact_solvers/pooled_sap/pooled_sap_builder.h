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

  /**
   * Update the given PooledSapModel to represent the SAP problem
   *
   *     min ℓ(v; q₀, v₀, δt)
   *
   * @param context The MbP context storing the current state (q₀, v₀).
   * @param time_step The requested time step δt.
   * @param reuse_geometry_data Flag for reusing geometry data aready set in the
   *                            model. This is useful for avoiding geometry
   *                            queries when only δt has changed, not (q₀, v₀).
   * @param model The PooledSapModel to update.
   *
   * TODO(vincekurtz): instead of the reuse_geometry_data flag, consider an
   * alternative ResetTimestep() methods that only updates δt, keeping all of
   * the other terms unchanged (and therefore not requiring geometry queries).
   */
  void UpdateModel(const systems::Context<T>& context, const T& time_step,
                   bool reuse_geometry_data, PooledSapModel<T>* model);

  /**
   * Update the given model to include external forces
   *
   *   τ = −Kₑ⋅v + bₑ
   *
   * where Kₑ is a positive semi-definite gain matrix and bₑ a bias term. Matrix
   * Kₑ is diagonal.
   *
   * More specifically, the linear dynamics matrix A and residual term r are
   * updated according to:
   *
   *   A += δt⋅Kₑ
   *   r += δt⋅bₑ
   *
   * Kₑ and bₑ are provided for the entire model. Kₑ can have zero entries.
   *
   * @pre Ke has positive (or zero) entries.
   * @pre Both Ke and be are vectors of size model->num_velocities().
   */
  void AddExternalGains(const VectorX<T>& Ke, const VectorX<T>& be,
                        PooledSapModel<T>* model) const;

  /**
   * Add constraints to model actuation with effort limits:
   *
   *  τ = clamp(−Kᵤ⋅v + bᵤ, e)
   *
   * where Kᵤ is a positive semi-definite gain matrix, bᵤ a bias term and e the
   * effort limit. Matrix Kᵤ is diagonal.

   * Kᵤ and bᵤ are provided for the entire model and Kᵤ can have zero entries.
   * The effort limits are obtained from the plant() model provided at
   * construction of `this` builder.
   * Constraints are added on a per-clique basis, and no actuation constraints
   * are added if a clique has no actuators.
   */
  void AddActuationGains(const VectorX<T>& Ku, const VectorX<T>& bu,
                         PooledSapModel<T>* model) const;

 private:
  // Compute geometry data and store it internally for later use
  void CalcGeometryContactData(const systems::Context<T>& context);

  // Allocate space for both point contact and hydroelastic contact constraints
  // @pre Geometry contact data has already been computed
  void AllocatePatchConstraints(PooledSapModel<T>* model) const;

  // Add point contact constraints to the model
  void AddPatchConstraintsForPointContact(const systems::Context<T>& context,
                                          PooledSapModel<T>* model) const;

  // Add hydroelastic contact constraints to the model
  void AddPatchConstraintsForHydroelasticContact(
      const systems::Context<T>& context, PooledSapModel<T>* model) const;

  // Add coupler constraints to the model
  void AddCouplerConstraints(const systems::Context<T>& context,
                             PooledSapModel<T>* model) const;

  // Add joint limit constraints to the model
  void AddLimitConstraints(const systems::Context<T>& context,
                           PooledSapModel<T>* model) const;

  // The multibody plant used to build the model.
  const MultibodyPlant<T>& plant() const { return *plant_; }
  const MultibodyPlant<T>* plant_{nullptr};

  // Model properties that do not change unless the system changes.
  std::map<geometry::GeometryId, CoulombFriction<double>> friction_;
  std::map<geometry::GeometryId, T> stiffness_;    // point contact.
  std::map<geometry::GeometryId, T> dissipation_;  // H&C dissipation.

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

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapBuilder);
