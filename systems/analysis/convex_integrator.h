#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/plant/discrete_contact_data.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using multibody::MultibodyForces;
using multibody::MultibodyPlant;
using multibody::internal::DiscreteContactData;
using multibody::internal::DiscreteContactPair;
using multibody::internal::GetInternalTree;
using multibody::internal::MultibodyTreeTopology;
using multibody::internal::MultibodyTree;
using multibody::JacobianWrtVariable;
using math::RigidTransform;
using geometry::GeometryId;
using multibody::BodyIndex;

/**
 * An experimental implicit integrator that solves a convex SAP problem to
 * advance the state, rather than relying on non-convex Newton-Raphson.
 */
template <class T>
class ConvexIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexIntegrator);

  ~ConvexIntegrator() override = default;

  /**
   * Constructs the experimental convex integrator.
   *
   * @param system the overall system diagram to simulate. Must be a Diagram
   *               with a MultibodyPlant as a subsystem (and MbP must be the
   *               only subsystem with interesting dynamics.)
   * @param context context for the overall system.
   *
   * N.B. Although this is an implicit integration scheme, we inherit from
   * IntegratorBase rather than ImplicitIntegrator because the way we compute
   * the Jacobian (Hessian for us) is completely different, and MultibodyPlant
   * specific.
   */
  explicit ConvexIntegrator(const System<T>& system,
                            Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    // Check that the system we're simulating is a diagram with a plant in it
    const Diagram<T>* diagram = dynamic_cast<const Diagram<T>*>(&system);
    DRAKE_DEMAND(diagram != nullptr);

    // Extract the plant that we're dealing with
    plant_ = dynamic_cast<const MultibodyPlant<T>*>(
        &diagram->GetSubsystemByName("plant"));
    DRAKE_DEMAND(plant_ != nullptr);
  }

  // TODO(vincekurtz): add error estimation
  bool supports_error_estimation() const override { return false; }

  // TODO(vincekurtz): add error estimation
  int get_error_estimate_order() const override { return 0; }

  // Get a reference to the plant used for SAP computations
  const MultibodyPlant<T>& plant() const { return *plant_; }

 private:
  // Allocate the workspace
  void DoInitialize() final;

  // The main integration step, sets x_{t+h} in this->context.
  bool DoStep(const T& h) override;

  // Compute v*, the velocities that would occur without contact constraints.
  void CalcFreeMotionVelocities(const Context<T>& context, const T& h,
                                VectorX<T>* v_star);

  // Compute the linearized momentum matrix A = M + h D for the SAP problem.
  void CalcLinearDynamicsMatrix(const Context<T>& context, const T& h,
                                std::vector<MatrixX<T>>* A);

  // Adds contact constraints to the SAP problem.
  void AddContactConstraints(const Context<T>& context,
                             SapContactProblem<T>* problem) const;

  // Compute signed distances and jacobians. While we store this in a
  // DiscreteContactData struct (basically copying DiscreteUpdateManager), this
  // is computed using the plant's continuous state.
  void CalcContactPairs(
      const Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const;

  // Copied from DiscreteContactManager, but using the continuous state
  void AppendDiscreteContactPairsForPointContact(
      const Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const;

  // Copied from DiscreteContactManager, but using the continuous state
  void AppendDiscreteContactPairsForHydroelasticContact(
      const Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const;

  // Tree topology used for defining the sparsity pattern in A.
  const MultibodyTreeTopology& tree_topology() const {
    return GetInternalTree(plant()).get_topology();
  }

  // Various methods exposed from the plant
  const MultibodyTree<T>& internal_tree() const {
    return plant().internal_tree();
  }

  double default_contact_stiffness() const {
    return plant().penalty_method_contact_parameters_.geometry_stiffness;
  }

  double default_contact_dissipation() const {
    return plant().penalty_method_contact_parameters_.dissipation;
  }

  BodyIndex FindBodyByGeometryId(GeometryId geometry_id) const {
    return plant().FindBodyByGeometryId(geometry_id);
  }

  // Plant model, since convex integration is specific to MbP
  const MultibodyPlant<T>* plant_;

  // Scratch space for intermediate calculations
  struct Workspace {
    // Used in DoStep
    VectorX<T> q;                     // generalized positions to set
    VectorX<T> v_star;                // velocities of the unconstrained system
    std::vector<MatrixX<T>> A;        // Linear dynamics matrix
    SapSolverResults<T> sap_results;  // Container for v_{t+h}

    // Used in CalcFreeMotionVelocities
    MatrixX<T> M;  // mass matrix
    VectorX<T> k;  // coriolis and gravity terms from inverse dynamics
    VectorX<T> a;  // accelerations
    std::unique_ptr<MultibodyForces<T>> f_ext;  // external forces (gravity)

    // Used in AddContactConstraint
    DiscreteContactData<DiscreteContactPair<T>> contact_data;
  } workspace_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
