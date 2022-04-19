#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
struct ContactPairKinematics {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactPairKinematics);

  // Struct to store the block contribution from a given tree to the contact
  // Jacobian for a contact pair.
  struct JacobianTreeBlock {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JacobianTreeBlock);

    JacobianTreeBlock(TreeIndex tree_in, Matrix3X<T> J_in)
        : tree(tree_in), J(std::move(J_in)) {}

    // Index of the tree for this block.
    TreeIndex tree;

    // J.cols() must equal the number of generalized velocities for
    // the corresponding tree.
    Matrix3X<T> J;
  };

  ContactPairKinematics(T phi_in, std::vector<JacobianTreeBlock> jacobian_in,
                        math::RotationMatrix<T> R_WC_in)
      : phi(std::move(phi_in)),
        jacobian(std::move(jacobian_in)),
        R_WC(std::move(R_WC_in)) {}

  // Signed distance for the given pair. Defined negative for overlapping
  // bodies.
  T phi{};

  // TODO(amcastro-tri): consider using absl::InlinedVector since here we know
  // this has a size of at most 2.
  // Jacobian for a discrete contact pair stored as individual blocks for each
  // of the trees participating in the contact. Only one or two trees can
  // participate in a given contact.
  std::vector<JacobianTreeBlock> jacobian;

  // Rotation matrix to re-express between contact frame C and world frame W.
  math::RotationMatrix<T> R_WC;
};

// CompliantContactManager computes the contact Jacobian J_AcBc_C for the
// relative velocity at a contact point Co between two geometries A and B,
// expressed in a contact frame C with Cz coincident with the contact normal.
// This structure is used to cache the kinematics associated with the contact
// pairs for a given configuration.
template <typename T>
struct ContactJacobianCache {
  // Vector to store the kinematics for each of the discrete contact pairs.
  std::vector<ContactPairKinematics<T>> contact_kinematics;
};

// To compute accelerations due to external forces (in particular non-contact
// forces), we pack forces, ABA cache and accelerations into a single struct
// to confine memory allocations into a single cache entry.
template <typename T>
struct AccelerationsDueToExternalForcesCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AccelerationsDueToExternalForcesCache)
  explicit AccelerationsDueToExternalForcesCache(
      const MultibodyTreeTopology& topology);
  MultibodyForces<T> forces;  // The external forces causing accelerations.
  multibody::internal::ArticulatedBodyForceCache<T> aba_forces;  // ABA cache.
  multibody::internal::AccelerationKinematicsCache<T> ac;  // Accelerations.
};

template <typename T>
struct ContactProblemCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactProblemCache);
  explicit ContactProblemCache(double time_step) {
    sap_problem =
        std::make_unique<contact_solvers::internal::SapContactProblem<T>>(
            time_step);
  }
  copyable_unique_ptr<contact_solvers::internal::SapContactProblem<T>>
      sap_problem;
  std::vector<math::RotationMatrix<T>> R_WC;
};

// This class implements the interface given by DiscreteUpdateManager so that
// contact computations can be consumed by MultibodyPlant.
//
// In particular, this manager sets up a contact problem where each of the
// bodies in the MultibodyPlant model is compliant without introducing state.
// Supported models include point contact with a linear model of compliance, see
// GetPointContactStiffness() and the hydroelastic contact model, see @ref
// mbp_hydroelastic_materials_properties in MultibodyPlant's Doxygen
// documentation.
// Dissipation is modeled using a linear model. For point contact, given the
// penetration distance x and its time derivative ẋ, the normal contact force
// (in Newtons) is modeled as:
//   fₙ = k⋅(x + τ⋅ẋ)₊
// where k is the point contact stiffness, see GetPointContactStiffness(), τ is
// the dissipation time scale, and ()₊ corresponds to the "positive part"
// operator.
// Similarly, for hydroelastic contact the normal traction p (in Pascals) is:
//   p = (p₀+τ⋅dp₀/dn⋅ẋ)₊
// where p₀ is the object-centric virtual pressure field introduced by the
// hydroelastic model.
//
// TODO(amcastro-tri): Retire code from MultibodyPlant as this contact manager
// replaces all the contact related capabilities, per #16106.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class CompliantContactManager final
    : public internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactManager)

  CompliantContactManager() = default;

  ~CompliantContactManager() final;

  // Sets the parameters to be used by the SAP solver.
  void set_sap_solver_parameters(
      const contact_solvers::internal::SapSolverParameters& parameters) {
    sap_parameters_ = parameters;
  }

 private:
  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    systems::CacheIndex contact_problem;
    systems::CacheIndex discrete_contact_pairs;
    systems::CacheIndex non_contact_forces_accelerations;
    systems::CacheIndex non_contact_forces_evaluation_in_progress;
  };

  using internal::DiscreteUpdateManager<T>::plant;

  // Provide private access for unit testing only.
  friend class CompliantContactManagerTest;

  const MultibodyTreeTopology& tree_topology() const {
    return internal::GetInternalTree(this->plant()).get_topology();
  }

  // TODO(amcastro-tri): Either implement in future PR or resolve with 16955.
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final {
    throw std::runtime_error(
        "CompliantContactManager::DoCalcAccelerationKinematicsCache() must be "
        "implemented.");
  }

  void DeclareCacheEntries() final;
  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final;
  void DoCalcDiscreteValues(const systems::Context<T>&,
                            systems::DiscreteValues<T>*) const final;

  // Returns the point contact stiffness stored in group
  // geometry::internal::kMaterialGroup with property
  // geometry::internal::kPointStiffness for the specified geometry.
  // If the stiffness property is absent, it returns MultibodyPlant's default
  // stiffness.
  // GeometryId `id` must exist in the model or an exception is thrown.
  T GetPointContactStiffness(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Returns the dissipation time constant stored in group
  // geometry::internal::kMaterialGroup with property
  // "dissipation_time_constant". If not present, it returns
  // plant().time_step().
  T GetDissipationTimeConstant(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Helper to acquire per-geometry Coulomb friction coefficients from
  // SceneGraph. Discrete models cannot make a distinction between static and
  // dynamic coefficients of friction. Therefore this method returns the
  // coefficient of dynamic friction stored by SceneGraph while the coefficient
  // of static friction is ignored.
  // @pre id is a valid GeometryId in the inspector.
  double GetCoulombFriction(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Utility to combine stiffnesses k1 and k2 according to the rule:
  //   k  = k₁⋅k₂/(k₁+k₂)
  // In other words, the combined compliance (the inverse of stiffness) is the
  // sum of the individual compliances.
  static T CombineStiffnesses(const T& k1, const T& k2);

  // Utility to combine linear dissipation time constants. Consider two
  // spring-dampers with stiffnesses k₁ and k₂, and dissipation time scales τ₁
  // and τ₂, respectively. When these spring-dampers are connected in series,
  // they result in an equivalent spring-damper with stiffness k  =
  // k₁⋅k₂/(k₁+k₂) and dissipation τ = τ₁ + τ₂.
  // This method returns tau1 + tau2.
  static T CombineDissipationTimeConstant(const T& tau1, const T& tau2);

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
  // discrete contact pairs, including point and hydroelastic contact, into
  // `pairs.`
  // Throws an exception if `pairs` is nullptr.
  void CalcDiscreteContactPairs(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Eval version of CalcDiscreteContactPairs().
  const std::vector<internal::DiscreteContactPair<T>>& EvalDiscreteContactPairs(
      const systems::Context<T>& context) const;

  // This method computes the kinematics information for each contact pair at
  // the given configuration stored in `context`.
  std::vector<ContactPairKinematics<T>> CalcContactKinematics(
      const systems::Context<T>& context) const;

  // Given the previous state x0 stored in `context`, this method computes the
  // "free motion" velocities, denoted v*.
  void CalcFreeMotionVelocities(const systems::Context<T>& context,
                                VectorX<T>* v_star) const;

  // Computes the linearized momentum equation matrix A to build the SAP
  // contact problem. Refer to SapContactProblem's class documentation for
  // details.
  void CalcLinearDynamicsMatrix(const systems::Context<T>& context,
                                std::vector<MatrixX<T>>* A) const;

  // Calc non-contact forces and the accelerations they induce.
  void CalcAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context,
      AccelerationsDueToExternalForcesCache<T>* no_contact_accelerations_cache)
      const;

  // Eval version of CalcAccelerationsDueToNonContactForcesCache().
  const multibody::internal::AccelerationKinematicsCache<T>&
  EvalAccelerationsDueToNonContactForcesCache(
      const systems::Context<T>& context) const;

  // Computes the necessary data to describe the SAP contact problem. Additional
  // information such as the orientation of each contact frame in the world is
  // also computed here so that it can be used at a later stage to compute
  // contact results.
  // All contact constraints are added before any other constraint types. This
  // manager assumes this ordering of the constraints in order to extract
  // contact impulses for reporting contact results.
  void CalcContactProblemCache(const systems::Context<T>& context,
                               ContactProblemCache<T>* cache) const;

  // Eval version of CalcContactProblemCache().
  const ContactProblemCache<T>& EvalContactProblemCache(
      const systems::Context<T>& context) const;

  // Add contact constraints for the configuration stored in `context` into
  // `problem`. This method returns the orientation of the contact frame in the
  // world frame for each contact constraint added to `problem`. That is, the
  // i-th entry in the return vector corresponds to the orientation R_WC contact
  // frame in the world frame for the i-th contact constraint added to
  // `problem`.
  std::vector<math::RotationMatrix<T>> AddContactConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // This method takes SAP results for a given `problem` and loads forces due to
  // contact only into `contact_results`. `contact_results` is properly resized
  // on output.
  // @pre contact_results is not nullptr.
  // @pre All `num_contacts` contact constraints in `problem` were added before
  // any other SAP constraint. This requirement is imposed by this manager which
  // adds constraints (with AddContactConstraints()) to the contact problem
  // before any other constraints are added. See the implementation of
  // CalcContactProblemCache(), who is responsible for adding constraints in
  // this particular order.
  void PackContactSolverResults(
      const contact_solvers::internal::SapContactProblem<T>& problem,
      int num_contacts,
      const contact_solvers::internal::SapSolverResults<T>& sap_results,
      contact_solvers::internal::ContactSolverResults<T>* contact_results)
      const;

  CacheIndexes cache_indexes_;
  contact_solvers::internal::SapSolverParameters sap_parameters_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::CompliantContactManager);
