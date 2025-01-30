#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

using multibody::MultibodyForces;
using multibody::MultibodyPlant;

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

  // Plant model, since convex integration is specific to MbP
  const MultibodyPlant<T>* plant_;

  // Scratch space for intermediate calculations
  struct Workspace {
    VectorX<T> q;  // Generalized positions to set
    MatrixX<T> M;  // Mass matrix
    VectorX<T> k;  // coriolis and gravity terms from inverse dynamics
    VectorX<T> a;  // accelerations
    std::unique_ptr<MultibodyForces<T>> f_ext;  // External forces (gravity)
    VectorX<T> v_star;  // velocities of the unconstrained system
  } workspace_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
