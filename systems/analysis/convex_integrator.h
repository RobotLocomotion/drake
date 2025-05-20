#pragma once

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

using multibody::MultibodyPlant;

/**
 * An experimental implicit integrator that solves a convex SAP problem to
 * advance the state, rather than relying on non-convex Newton-Raphson.
 *
 * N.B. Although this is an implicit integration scheme, we inherit from
 * IntegratorBase rather than ImplicitIntegrator because the way we compute
 * the Jacobian (Hessian) is completely different, and MultibodyPlant
 * specific.
 */
template <class T>
class ConvexIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexIntegrator);

  ~ConvexIntegrator() override = default;

  /**
   * Constructs the convex integrator.
   *
   * @param system the overall system diagram to simulate. Must include a
   *               MultibodyPlant and associated SceneGraph.
   * @param context context for the overall system.
   *
   * @note This constructor matches the signature used by other integrators, but
   *       does not specify the MultibodyPlant used to set up the optimization
   *       problem. set_plant() must be called before using the integrator.
   */
  explicit ConvexIntegrator(const System<T>& system,
                            Context<T>* context = nullptr);

  /**
   * Constructs the convex integrator, specifying the MultibodyPlant used to
   * formulate the convex optimization problem.
   */
  ConvexIntegrator(const System<T>& system, MultibodyPlant<T>* plant,
                   Context<T>* context = nullptr);

  /**
   * Specifies the MultibodyPlant used to set up the optimization problem.
   */
  void set_plant(MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    plant_ = plant;
  }

  /**
   * Get a reference to the MultibodyPlant used to formulate the convex
   * optimization problem.
   */
  const MultibodyPlant<T>& plant() const {
    DRAKE_ASSERT(plant_ != nullptr);
    return *plant_;
  }

  // TODO(vincekurtz): add support for error estimation.
  bool supports_error_estimation() const final { return false; }
  int get_error_estimate_order() const final { return 0; }

 private:
  // Perform final checks and allocations before beginning integration.
  void DoInitialize() final;

  // Perform the main integration step, setting x_{t+h} and the error estimate.
  bool DoStep(const T& h) override;

  // The multibody plant used as the basis of the convex optimization problem.
  MultibodyPlant<T>* plant_{nullptr};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
