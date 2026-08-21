#pragma once

#include <memory>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace multibody {

/** Constructs a CenicIntegrator.
@param system The overall Diagram to simulate. Must include a MultibodyPlant and
  associated SceneGraph, with the plant found as a direct child of the `system`
  diagram using the subsystem name `"plant"`. This system is aliased by this
  object so must remain alive longer than the integrator.
@param context Context for `system`.
@tparam_nonsymbolic_scalar */
template <class T>
std::unique_ptr<systems::IntegratorBase<T>> MakeCenicIntegrator(
    const systems::System<T>& system, systems::Context<T>* context = nullptr);

}  // namespace multibody
}  // namespace drake
