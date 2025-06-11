#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/contact_solvers/sap/sap_fixed_constraint.h"
#include "drake/multibody/contact_solvers/schur_complement.h"
#include "drake/multibody/fem/fem_solver.h"
#include "drake/multibody/plant/deformable_contact_info.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/discrete_contact_data.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/geometry_contact_data.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* Helper class for DeformableDriver that acts both as a multiplexer and a
 demultiplexer -- it combines multiple Eigen vectors into a single stacked
 vector and it also splits an Eigen vector into multiple vectors.
 @tparam_default_scalar */
template <typename T>
class Multiplexer {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Multiplexer);

  /* Create an invalid Multiplexer. It cannot be used to (de)multiplex any
   vectors. */
  Multiplexer() = default;

  /* Constructs a Multiplexer that combines and splits vectors of the given
   sizes.
   @pre `sizes` is not empty and each entry is non-negative. */
  explicit Multiplexer(std::vector<int> sizes);

  /* The number of vectors to be multiplexed. */
  int num_vectors() const { return sizes_.size(); }

  /* Combines the given vectors into a single vector.
   @throws std::exception if the sizes of `inputs` aren't compatible with the
   sizes provided at construction. */
  VectorX<T> Multiplex(std::vector<VectorX<T>>&& inputs) const;

  /* Splits the given vector into multiple vectors and returns the one with
   the given `index`.
   @throws std::exception if the size of `input` is not the sum of sizes
   provided at construction.
   @throws std::exception if index is not in [0, num_vectors).
   @returns a vector block of the indexed vector. */
  Eigen::Ref<const VectorX<T>> Demultiplex(
      const Eigen::Ref<const VectorX<T>>& input, int index) const;

  /* Mutable version of `Demultiplex()` that takes a pointer to a stacked
   vector. */
  Eigen::Ref<VectorX<T>> Demultiplex(EigenPtr<VectorX<T>> input,
                                     int index) const;

 private:
  std::vector<int> sizes_;
  std::vector<int> offsets_;
  /* The sum over `sizes_`. */
  int num_entries_{0};
};

template <typename T>
class DiscreteUpdateManager;

/* DeformableDriver is responsible for computing dynamics information about
 all deformable bodies. It works in tandem with a DeformableModel and a
 DiscreteUpdateManager that are provided at construction time. The deformable
 model informs the driver of modeling choices of the deformable bodies
 such as its Finite Element Model. The discrete update manager consumes the
 results of the computation performed by the driver and also provides
 information about the result of the world that the deformable bodies are
 interacting with. In particular, the manager provides access to MultibodyPlant.

 For any vertex in a deformable body, we say that it is "participating in
 contact and constraints" (or "participating" in short) if it is incident to a
 tetrahedron containing one or more contact points or explicitly specified as
 under constraint. We say a degree of freedom (dof) is "participating" if it
 belongs to a participating vertex. DeformableDriver reports participating
 quantities in increasing order of deformable body indexes. That is, the
 participating quantities of body 0 come first, followed by participating
 quantities of body 1 and so on. Within a single body, the participating
 vertices/dofs are ordered according to their associated vertex indexes.

 @tparam_double_only */
template <typename T>
class DeformableDriver : public ScalarConvertibleComponent<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableDriver);

  /* Constructs a deformable driver that solves for the dynamics of the given
   `deformable_model`. The newly constructed driver is used in the given
   `manager` to perform discrete updates. The given `deformable_model` and
   `manager` must outlive this driver.
   @pre deformable_model != nullptr.
   @pre manager != nullptr. */
  DeformableDriver(const DeformableModel<T>* deformable_model,
                   const DiscreteUpdateManager<T>* manager);

  ~DeformableDriver() override;

  int num_deformable_bodies() const { return deformable_model_->num_bodies(); }

  // TODO(xuchenhan-tri): Implement CloneToDouble() and allow cloning to double.
  bool is_cloneable_to_double() const final { return false; }
  bool is_cloneable_to_autodiff() const final { return false; }
  bool is_cloneable_to_symbolic() const final { return false; }

  /* Declares cache entries used by this DeformableDriver through the given
   manager.
   @pre `manager` is not nullptr and points to the same DiscreteUpdateManager
   provided at construction. */
  void DeclareCacheEntries(DiscreteUpdateManager<T>* manager);

  /* Evaluates the velocities of all participating dofs. See class documentation
   for how the velocities are ordered. */
  const VectorX<T>& EvalParticipatingVelocities(
      const systems::Context<T>& context) const;

  /* Evaluates the free motion velocities of all participating dofs. See class
   documentation for how the velocities are ordered. */
  const VectorX<T>& EvalParticipatingFreeMotionVelocities(
      const systems::Context<T>& context) const;

  /* Appends the linear dynamics matrices for participating dofs of each
   deformable body registered in this model to `A` in increasing order of
   deformable body indexes. The matrix corresponding to a body without any
   participating dof is empty.
   @note a disabled deformable body has no participating dofs and hence the
   corresponding matrix is empty. See DeformableModel::Disable().
   @pre A != nullptr. */
  void AppendLinearDynamicsMatrix(const systems::Context<T>& context,
                                  std::vector<MatrixX<T>>* A) const;

  /* Appends discrete contact pairs where at least one of the bodies in contact
   is deformable.
   @note a disabled deformable body does not participate in contact. See
   DeformableModel::Disable().
   @pre result != nullptr. */
  void AppendDiscreteContactPairs(
      const systems::Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const;

  /* Appends the constraint kinematics information for each deformable rigid
   fixed constraint.
   @note If a deformable body is disabled, then it by definition does not have
   any constraint kinematics and nothing is appended.
   @pre result != nullptr. */
  void AppendDeformableRigidFixedConstraintKinematics(
      const systems::Context<T>& context,
      std::vector<contact_solvers::internal::FixedConstraintKinematics<T>>*
          result) const;

  /* Computes the contact information for all deformable bodies.
   @pre contact_info != nullptr. */
  void CalcDeformableContactInfo(
      const geometry::internal::DeformableContact<T>& deformable_contact,
      const DiscreteContactData<DiscreteContactPair<T>>& contact_pairs,
      const contact_solvers::internal::ContactSolverResults<T>& solver_results,
      std::vector<DeformableContactInfo<T>>* contact_info) const;

  /* Evaluates FemState at the next time step for each deformable body and
   copies the them into the corresponding DiscreteValues.
   @pre next_states != nullptr. */
  void CalcDiscreteStates(const systems::Context<T>& context,
                          systems::DiscreteValues<T>* next_states) const;

  /* Evaluates the multiplexer for participating velocities for all bodies.
   @pre result != nullptr. */
  const Multiplexer<T>& EvalParticipatingVelocityMultiplexer(
      const systems::Context<T>& context) const;

  /* Evaluates the constraint participation information of the deformable body
   with the given `index`. See geometry::internal::ContactParticipation. */
  const geometry::internal::ContactParticipation& EvalConstraintParticipation(
      const systems::Context<T>& context, DeformableBodyIndex index) const;

  /* Computes the contact information for all registered deformable bodies.
   This is used by MbP to populate the GeometryContactData summary along with
   all of the other types of geometry contacts (point, surface, etc).
   @pre result != nullptr. */
  void CalcDeformableContact(
      const geometry::QueryObject<T>& query_object,
      geometry::internal::DeformableContact<T>* result) const;

 private:
  friend class DeformableDriverTest;
  friend class DeformableDriverContactTest;
  friend class DeformableDriverContactKinematicsTest;
  friend class DeformableIntegrationTest;

  /* Struct used to conglomerate the indexes of cache entries declared by
   the manager. */
  struct CacheIndexes {
    /* Per body cache entries indexed by DeformableBodyIndex. */
    std::vector<systems::CacheIndex> fem_solvers;
    std::vector<systems::CacheIndex> next_fem_states;
    std::vector<systems::CacheIndex> constraint_participations;
    std::unordered_map<geometry::GeometryId, systems::CacheIndex>
        vertex_permutations;
    systems::CacheIndex participating_velocity_mux;
    systems::CacheIndex participating_velocities;
    systems::CacheIndex participating_free_motion_velocities;
  };

  /* Struct to hold intermediate data from one of the two geometries in contact
   when computing DiscreteContactPair. */
  struct ContactData {
    /* The world frame position of the relative-to point for reporting the
     contact results. See DiscreteContactPair::p_ApC_W and
     DiscreteContactPair::p_BqC_W. `p_WG` is coincident with P and Q (and as
     they are all measured and expressed in the world frame, they will all
     have the same values). */
    Vector3<T> p_WG;
    /* Contact Jacobians for the kinematic tree corresponding to the object
     participating in the contact. `jacobian[i]` stores the contact Jacobian for
     the i-th contact point. This is empty if the geometry is rigid and welded
     to World. */
    std::vector<typename DiscreteContactPair<T>::JacobianTreeBlock> jacobian;
    /* Velocity (in the world frame) of the point Gc affixed to the geometry
     that is coincident with the contact point C. `v_WGc[i]` stores the
     world-frame velocity of the i-th contact point. This is empty if the
     geometry is rigid and welded to World. */
    std::vector<Vector3<T>> v_WGc;
    /* Name of the geometry in contact. */
    std::string name;
  };

  /* Computes the contact data for a deformable geometry G participating in
   contact.
   @param[in] context          Context of the MultibodyPlant owning this driver.
   @param[in] contact_surface  The contact surface between two geometries with
                               one of the geometries being geometry G.
   @param[in] is_A             True if geometry G is labeled as geometry A in
                               the given `contact_surface`. See class
                               documentation for
                               geometry::internal::DeformableContactSurface for
                               details. */
  ContactData ComputeContactDataForDeformable(
      const systems::Context<T>& context,
      const geometry::internal::DeformableContactSurface<T>& contact_surface,
      bool is_A) const;

  /* Computes the contact data for a rigid geometry G participating in contact.
   @param[in] context          Context of the MultibodyPlant owning this driver.
   @param[in] contact_surface  The contact surface between two geometries with
                               one of the geometries being geometry G.
   @note Unlike ComputeContactDataForDeformable where we need to determine
   whether geometry G is labeled as geometry A or B in DeformableContactSurface,
   by convention, a rigid geometry is always labeled as geometry B in
   DeformableContactSurface if it participates in deformable contact. */
  ContactData ComputeContactDataForRigid(
      const systems::Context<T>& context,
      const geometry::internal::DeformableContactSurface<T>& contact_surface)
      const;

  /* Copies the state of the deformable body with `id` in the given `context`
   to the `fem_state`.
   @pre fem_state != nullptr and has size compatible with the state of the
        deformable body with the given `index`.
   @pre `index` is valid and less than the number of deformable bodies. */
  void CalcFemState(const systems::Context<T>& context,
                    DeformableBodyIndex index,
                    fem::FemState<T>* fem_state) const;

  /* Eval version of CalcFemState(). */
  const fem::FemState<T>& EvalFemState(const systems::Context<T>& context,
                                       DeformableBodyIndex index) const;

  /* Given the state of the deformable body with `index` in the given `context`,
   computes its "free motion" state (the state the body would have at the next
   time step in the absence of contact or constraints) and the dependent
   Schur complement of the tangent matrix of the FEM model.
   @pre state_and_data != nullptr and is compatible with the FemModel associated
   with the deformable body with the given `index`. */
  void CalcFreeMotionFemSolver(const systems::Context<T>& context,
                               DeformableBodyIndex index,
                               fem::internal::FemSolver<T>* fem_solver) const;

  /* Eval version of CalcFreeMotionFemState(). */
  const fem::internal::FemSolver<T>& EvalFreeMotionFemSolver(
      const systems::Context<T>& context, DeformableBodyIndex index) const;

  const fem::FemState<T>& EvalFreeMotionFemState(
      const systems::Context<T>& context, DeformableBodyIndex index) const;

  const contact_solvers::internal::SchurComplement&
  EvalFreeMotionTangentMatrixSchurComplement(const systems::Context<T>& context,
                                             DeformableBodyIndex index) const;

  /* Given the state of the deformable body with `index` in the given `context`,
   computes the state of the deformable body at the next time step.
   @note The state of the deformable body will the same as the "free motion"
         state in the absence of contact or constraints. Otherwise, the discrete
         solver results for participating dofs are evaluated, and the Schur
         complement of the tangent matrix is used to update the
         non-participating dofs.
   @pre next_fem_state != nullptr and is compatible with the state of
        the deformable body with the given `index`. */
  void CalcNextFemState(const systems::Context<T>& context,
                        DeformableBodyIndex index,
                        fem::FemState<T>* next_fem_state) const;

  /* Eval version of CalcNextFemState(). */
  const fem::FemState<T>& EvalNextFemState(const systems::Context<T>& context,
                                           DeformableBodyIndex index) const;

  /* Eval version of CalcDeformableContact(), though notably routed through
   MultibodyPlant so that the plant can own the GeometryContactData cache. */
  const geometry::internal::DeformableContact<T>& EvalDeformableContact(
      const systems::Context<T>& context) const;

  /* Calc version of EvalConstraintParticipation.
   @pre constraint_participation != nullptr. */
  void CalcConstraintParticipation(
      const systems::Context<T>& context, DeformableBodyIndex index,
      geometry::internal::ContactParticipation* constraint_participation) const;

  /* Evaluates the partial permutation that maps dof indices of the
   deformable geometry with the given `id` to their corresponding values in the
   constraint problem. */
  const contact_solvers::internal::PartialPermutation& EvalDofPermutation(
      const systems::Context<T>& context, DeformableBodyIndex index) const;

  /* Computes the partial permutation that maps vertex/dof indices of the
   deformable geometry with the given `id` to their corresponding values in the
   constraint problem.
   @pre result != nullptr. */
  void CalcPermutation(
      const systems::Context<T>& context, geometry::GeometryId id,
      contact_solvers::internal::VertexPartialPermutation* result) const;

  /* Evaluates the partial permutation that maps vertex indices of the
   deformable geometry with the given `id` to their corresponding values in the
   constraint problem. */
  const contact_solvers::internal::PartialPermutation& EvalVertexPermutation(
      const systems::Context<T>& context, geometry::GeometryId id) const;

  /* Calc version of EvalParticipatingVelocityMultiplexer(). */
  void CalcParticipatingVelocityMultiplexer(const systems::Context<T>& context,
                                            Multiplexer<T>* result) const;

  /* Calc version of EvalParticipatingVelocities().
   @pre result != nullptr. */
  void CalcParticipatingVelocities(const systems::Context<T>& context,
                                   VectorX<T>* result) const;

  /* Calc version of EvalParticipatingFreeMotionVelocities().
   @pre result != nullptr. */
  void CalcParticipatingFreeMotionVelocities(const systems::Context<T>& context,
                                             VectorX<T>* result) const;

  CacheIndexes cache_indexes_;
  /* Modeling information about all deformable bodies. */
  const DeformableModel<T>* const deformable_model_;
  const DiscreteUpdateManager<T>* const manager_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
