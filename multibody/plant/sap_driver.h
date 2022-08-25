#pragma once

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

// Forward declaration.
template <typename>
class MultibodyPlant;

namespace internal {

// Forward declaration.
template <typename>
class CompliantContactManager;

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

// Performs the computations need by CompliantContactManager using the SAP
// solver.
template <typename T>
class SapDriver {
 public:
  SapDriver(CompliantContactManager<T>* manager);

  void set_sap_solver_parameters(
      const contact_solvers::internal::SapSolverParameters& parameters);

  const CompliantContactManager<T>& manager() const { return *manager_; }

  const MultibodyPlant<T>& plant() const { return manager().plant(); }    

  void DeclareCacheEntries(CompliantContactManager<T>* manager);

  void CalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const;  

 private:
  // Provide private access for unit testing only.
  friend class SapDriverTest;

  const MultibodyTreeTopology& tree_topology() const {
    return GetInternalTree(this->plant()).get_topology();
  }

  // Computes the linearized momentum equation matrix A to build the SAP
  // contact problem. Refer to SapContactProblem's class documentation for
  // details.
  void CalcLinearDynamicsMatrix(const systems::Context<T>& context,
                                std::vector<MatrixX<T>>* A) const;

  // Given the previous state x0 stored in `context`, this method computes the
  // "free motion" velocities, denoted v*.
  void CalcFreeMotionVelocities(const systems::Context<T>& context,
                                VectorX<T>* v_star) const;

  // This method computes the kinematics information for each contact pair at
  // the given configuration stored in `context`.
  std::vector<ContactPairKinematics<T>> CalcContactKinematics(
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

  // Add limit constraints for the configuration stored in `context` into
  // `problem`. Limit constraints are only added when the state q₀ for a
  // particular joint is "close" to the joint's limits (qₗ,qᵤ). To decide when
  // the state q₀ is close to the joint's limits, this method estimates a window
  // (wₗ,wᵤ) for the expected value of the configuration q at the next time
  // step. Lower constraints are considered whenever qₗ > wₗ and upper
  // constraints are considered whenever qᵤ < wᵤ. This window (wₗ,wᵤ) is
  // estimated based on the current velocity v₀ and the free motion velocities
  // v*, provided with `v_star`.
  // Since the implementation uses the current velocity v₀ to estimate whether
  // the constraint should be enabled, it is at least as good as a typical
  // continuous collision detection method. It could mispredict under conditions
  // of strong acceleration (it is assuming constant velocity across a step).
  // Still, at typical robotics step sizes and rates it would be surprising to
  // see that happen, and if it did the limit would come on in the next step.
  // TODO(amcastro-tri): Consider using the acceleration at t₀ to get a second
  // order prediction for the configuration at the next time step.
  // @pre problem must not be nullptr.
  void AddLimitConstraints(
      const systems::Context<T>& context, const VectorX<T>& v_star,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds holonomic constraints to model couplers specified in the
  // MultibodyPlant.
  void AddCouplerConstraints(
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

  // Computes the necessary data to describe the SAP contact problem. Additional
  // information such as the orientation of each contact frame in the world is
  // also computed here so that it can be used at a later stage to compute
  // contact results.
  // All contact constraints are added before any other constraint types. This
  // manager assumes this ordering of the constraints in order to extract
  // contact impulses for reporting contact results.
  void CalcContactProblemCache(const systems::Context<T>& context,
                               ContactProblemCache<T>* cache) const;

  // Eval version of CalcContactProblemCache()
  const ContactProblemCache<T>& EvalContactProblemCache(
      const systems::Context<T>& context) const;

  // The driver only has mutable access at construction time, when it can
  // declare additional state, cache entries, ports, etc. After construction,
  // the driver only has const access to the manager.
  const CompliantContactManager<T>* manager_{nullptr};
  systems::CacheIndex contact_problem_;
  // Vector of joint damping coefficients, of size plant().num_velocities().
  // This information is extracted during the call to ExtractModelInfo().
  VectorX<T> joint_damping_;
  // Parameters for SAP.
  contact_solvers::internal::SapSolverParameters sap_parameters_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake