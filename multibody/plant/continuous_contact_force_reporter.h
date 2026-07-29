#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* Abstract interface by which a continuous-mode MultibodyPlant reports contact
and reaction forces consistent with a discrete/convex contact model that some
external integrator is using to advance the plant, instead of the plant's own
compliant point/hydroelastic continuous contact model.

The concrete implementation (e.g. ContinuousIcfForceManager for the ICF model
used by CenicIntegrator) lives in a target downstream of both //multibody/plant
and the contact-model library, and is injected into the plant via
MultibodyPlant::SetContinuousContactForceReporter(). This indirection exists
because the ICF contact-solver library depends on the full MultibodyPlant, so
the plant cannot depend on it directly; see continuous_icf_force_manager.h.

When a reporter is set on a continuous plant, MultibodyPlant routes the
continuous branch of its reaction_forces output port through it. When none is
set, the plant uses its compliant contact model as before. The reporter is
queried only as a pure function of the plant Context.

@tparam_nonsymbolic_scalar */
template <typename T>
class ContinuousContactForceReporter {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContinuousContactForceReporter);

  ContinuousContactForceReporter() = default;
  virtual ~ContinuousContactForceReporter() = default;

  /* Computes the multibody forces applied by the contact model at the state
  stored in `context`: all non-contact forces (gravity, force elements, joint
  damping, actuation) plus the model's contact/constraint forces, such that
  forward dynamics of `forces` reproduces the acceleration the integrator would
  take. `forces` is resized as needed. */
  virtual void CalcAppliedForces(const systems::Context<T>& context,
                                 MultibodyForces<T>* forces) const = 0;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::ContinuousContactForceReporter);
