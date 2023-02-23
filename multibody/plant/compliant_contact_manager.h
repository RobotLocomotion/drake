#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/plant/contact_pair_kinematics.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/deformable_driver.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
// Forward declaration.
struct SapSolverParameters;
}  // namespace internal
}  // namespace contact_solvers
namespace internal {

// Forward declaration.
template <typename>
class SapDriver;
template <typename>
class TamsiDriver;

// To compute accelerations due to external forces (in particular non-contact
// forces), we pack forces, ABA cache and accelerations into a single struct
// to confine memory allocations into a single cache entry.
template <typename T>
struct AccelerationsDueToExternalForcesCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AccelerationsDueToExternalForcesCache)
  explicit AccelerationsDueToExternalForcesCache(
      const MultibodyTreeTopology& topology);
  MultibodyForces<T> forces;  // The external forces causing accelerations.
  ArticulatedBodyInertiaCache<T> abic;   // Articulated body inertia cache.
  std::vector<SpatialForce<T>> Zb_Bo_W;  // Articulated body biases cache.
  multibody::internal::ArticulatedBodyForceCache<T> aba_forces;  // ABA cache.
  multibody::internal::AccelerationKinematicsCache<T> ac;  // Accelerations.
};

// This class implements the interface given by DiscreteUpdateManager so that
// contact computations can be consumed by MultibodyPlant.
//
// In particular, this manager sets up a contact problem where each rigid body
// in the MultibodyPlant model is compliant without introducing state. Supported
// models include point contact with a linear model of compliance, see
// GetPointContactStiffness() and the hydroelastic contact model, see @ref
// mbp_hydroelastic_materials_properties in MultibodyPlant's Doxygen
// documentation. Dynamics of deformable bodies (if any exists) are calculated
// in DeformableDriver. Deformable body contacts are modeled as near-rigid point
// contacts where compliance is added as a means of stabilization without
// introducing additional states (i.e. the penetration distance x and its time
// derivative ẋ are not states). Dissipation is modeled using a linear model.
// For point contact, the normal contact force (in Newtons) is modeled as:
//   fₙ = k⋅(x + τ⋅ẋ)₊
// where k is the point contact stiffness, see GetPointContactStiffness(), τ is
// the dissipation timescale, and ()₊ corresponds to the "positive part"
// operator.
// Similarly, for hydroelastic contact the normal traction p (in Pascals) is:
//   p = (p₀+τ⋅dp₀/dn⋅ẋ)₊
// where p₀ is the object-centric virtual pressure field introduced by the
// hydroelastic model.
//
// TODO(amcastro-tri): Retire code from MultibodyPlant as this contact manager
// replaces all the contact related capabilities, per #16106.
//
// @warning Scalar support on T = symbolic::Expression is only limited,
// conditional to the solver in use:
//   - For TAMSI. Discrete updates are only supported when there is no contact
//     geometry. Otherwise an exception is thrown.
//   - For SAP. Discrete updates are not supported.
//
// Even when limited support for discrete updates is provided for T =
// symbolic::Expression, a MultibodyPlant can be scalar converted to symbolic in
// order to perform other supported queries, such as kinematics, or
// introspection.
//
// @tparam_default_scalar
template <typename T>
class CompliantContactManager final
    : public internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactManager)

  using internal::DiscreteUpdateManager<T>::plant;

  CompliantContactManager();

  ~CompliantContactManager() final;

  // Sets the parameters to be used by the SAP solver.
  // @pre plant().get_discrete_contact_solver() == DiscreteContactSolver::kSap.
  // @throws if called when instantiated on T = symbolic::Expression.
  void set_sap_solver_parameters(
      const contact_solvers::internal::SapSolverParameters& parameters);

  // @returns `true`.
  bool is_cloneable_to_double() const final;

  // @returns `true`.
  bool is_cloneable_to_autodiff() const final;

  // @returns `true`.
  bool is_cloneable_to_symbolic() const final;

 private:
  // TODO(amcastro-tri): Instead of friendship consider another set of class(es)
  // with tighter functionality. For instance, a class that takes care of
  // getting proximity properties and creating DiscreteContactPairs.
  friend class SapDriver<T>;
  friend class TamsiDriver<T>;

  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    systems::CacheIndex contact_kinematics;
    systems::CacheIndex discrete_contact_pairs;
    systems::CacheIndex hydroelastic_contact_info;
    systems::CacheIndex non_contact_forces_accelerations;
  };

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class CompliantContactManager;

  // Provide private access for unit testing only.
  friend class CompliantContactManagerTester;

  const MultibodyTreeTopology& tree_topology() const {
    return internal::GetInternalTree(this->plant()).get_topology();
  }

  std::unique_ptr<DiscreteUpdateManager<double>> CloneToDouble() const final;
  std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>> CloneToAutoDiffXd()
      const final;
  std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>> CloneToSymbolic()
      const final;

  // Extracts non state dependent model information from MultibodyPlant. See
  // DiscreteUpdateManager for details.
  void ExtractModelInfo() final;

  // Associates the given `DeformableModel` with `this` manager. The discrete
  // states of the deformable bodies registered in the given `model` will be
  // advanced by this manager. This manager holds onto the given pointer and
  // therefore the model must outlive the manager.
  // @throws std::exception if a deformable model has already been registered.
  // @pre model != nullptr.
  void ExtractConcreteModel(const DeformableModel<T>* model);

  // For testing purposes only, we provide a default no-op implementation on
  // arbitrary models of unknown concrete model type. Otherwise, for the closed
  // list of models forward declared in physical_model.h, we must provide a
  // function that extracts the particular variant of the physical model.
  void ExtractConcreteModel(std::monostate) {}

  void DoDeclareCacheEntries() final;

  // TODO(amcastro-tri): implement these APIs according to #16955.
  // @throws For SAP if T = symbolic::Expression.
  // @throws For TAMSI if T = symbolic::Expression only if the model contains
  // contact geometry.
  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final;
  void DoCalcDiscreteValues(const systems::Context<T>&,
                            systems::DiscreteValues<T>*) const final;
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final;
  void DoCalcContactResults(
      const systems::Context<T>&,
      ContactResults<T>* contact_results) const final;

  // This method computes sparse kinematics information for each contact pair at
  // the given configuration stored in `context`.
  std::vector<ContactPairKinematics<T>> CalcContactKinematics(
      const systems::Context<T>& context) const;

  // Eval version of CalcContactKinematics().
  const std::vector<ContactPairKinematics<T>>& EvalContactKinematics(
      const systems::Context<T>& context) const;

  // Given the configuration stored in `context`, this method appends discrete
  // pairs corresponding to point contact into `pairs`.
  // @pre pairs != nullptr.
  void AppendDiscreteContactPairsForPointContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this method appends discrete
  // pairs corresponding to hydroelastic contact into `pairs`.
  // @pre pairs != nullptr.
  void AppendDiscreteContactPairsForHydroelasticContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this method computes all
  // discrete contact pairs, including point, hydroelastic, and deformable
  // contact, into `pairs`. Contact pairs including deformable bodies are
  // guaranteed to come after point and hydroelastic contact pairs. Throws an
  // exception if `pairs` is nullptr.
  void CalcDiscreteContactPairs(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Eval version of CalcDiscreteContactPairs().
  const std::vector<internal::DiscreteContactPair<T>>& EvalDiscreteContactPairs(
      const systems::Context<T>& context) const;

  // Computes per-face contact information for the hydroelastic model (slip
  // velocity, traction, etc). On return contact_info->size() will equal the
  // number of faces discretizing the contact surface.
  void CalcHydroelasticContactInfo(
      const systems::Context<T>& context,
      std::vector<HydroelasticContactInfo<T>>* contact_info) const;

  // Eval version of CalcHydroelasticContactInfo() .
  const std::vector<HydroelasticContactInfo<T>>& EvalHydroelasticContactInfo(
      const systems::Context<T>& context) const;

  // Computes all continuous forces in the MultibodyPlant model. Joint limits
  // are not included as continuous compliant forces but rather as constraints
  // in the solver, and therefore must be excluded.
  // Values in `forces` will be overwritten.
  // @pre forces != nullptr and is consistent with plant().
  void CalcNonContactForcesExcludingJointLimits(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Calc non-contact forces and the accelerations they induce.
  void CalcAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context,
      AccelerationsDueToExternalForcesCache<T>* no_contact_accelerations_cache)
      const;

  // Eval version of CalcAccelerationsDueToNonContactForcesCache().
  const multibody::internal::AccelerationKinematicsCache<T>&
  EvalAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context) const;

  // Helper method to fill in contact_results with point contact information
  // for the given state stored in `context`.
  // @param[in,out] contact_results is appended to
  void AppendContactResultsForPointContact(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

  // Helper method to fill in `contact_results` with hydroelastic contact
  // information for the given state stored in `context`.
  // @param[in,out] contact_results is appended to
  void AppendContactResultsForHydroelasticContact(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

  CacheIndexes cache_indexes_;
  // Vector of joint damping coefficients, of size plant().num_velocities().
  // This information is extracted during the call to ExtractModelInfo().
  VectorX<T> joint_damping_;

  // deformable_driver_ computes the information on all deformable bodies needed
  // to advance the discrete states.
  std::unique_ptr<DeformableDriver<double>> deformable_driver_;

  // Specific contact solver drivers are created at ExtractModelInfo() time,
  // when the manager retrieves modeling information from MultibodyPlant.
  // Only one of these drivers will be non-nullptr.
  std::unique_ptr<SapDriver<T>> sap_driver_;
  std::unique_ptr<TamsiDriver<T>> tamsi_driver_;
};

// N.B. These geometry queries are not supported when T = symbolic::Expression
// and therefore their implementation throws.
template <>
void CompliantContactManager<symbolic::Expression>::CalcDiscreteContactPairs(
    const drake::systems::Context<symbolic::Expression>&,
    std::vector<DiscreteContactPair<symbolic::Expression>>*) const;
template <>
void CompliantContactManager<symbolic::Expression>::
    AppendDiscreteContactPairsForHydroelasticContact(
        const drake::systems::Context<symbolic::Expression>&,
        std::vector<DiscreteContactPair<symbolic::Expression>>*) const;

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
