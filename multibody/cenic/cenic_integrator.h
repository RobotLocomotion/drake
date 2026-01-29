#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace multibody {

/** Convex Error-controlled Numerical Integration for Contact (CENIC) is a
specialized error-controlled implicit integrator for contact-rich robotics
simulations [Kurtz and Castro, 2025].

@warning This class is currently just a stub that throws when used.

References:

  [Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
  Error-controlled Numerical Integration for Contact.
  https://arxiv.org/abs/2511.08771.

@tparam_nonsymbolic_scalar */
template <class T>
class CenicIntegrator final : public systems::IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CenicIntegrator);

  /** Constructs the integrator.
  @param system The overall system diagram to simulate. Must include a
                MultibodyPlant and associated SceneGraph, with the plant
                found as a direct child of the `system` diagram using the
                subsystem name `"plant"`. This system is aliased by this
                object so must remain alive longer than the integrator.
  @param context context for the overall system.  */
  explicit CenicIntegrator(const systems::System<T>& system,
                           systems::Context<T>* context = nullptr);

  ~CenicIntegrator() final;

  /** Gets a reference to the MultibodyPlant used to formulate the convex
  optimization problem. */
  const MultibodyPlant<T>& plant() const { return plant_; }

  bool supports_error_estimation() const final;

  int get_error_estimate_order() const final;

 private:
  void DoInitialize() final;
  bool DoStep(const T& h) final;

  // The multibody plant used as the basis of the convex optimization problem.
  const MultibodyPlant<T>& plant_;
  const systems::SubsystemIndex plant_subsystem_index_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
