#pragma once

#include "drake/common/name_value.h"
#include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"

namespace drake {
namespace multibody {

/** (Advanced) Parameters to configure a CenicIntegrator.

These are the CENIC-specific settings that can be given to a Simulator via
drake::systems::SimulatorConfig::integrator_params. In YAML, they are selected
with the `!CenicIntegratorParameters` tag. For example:

@code{.yaml}
integration_scheme: cenic
integrator_params: !CenicIntegratorParameters
  explicit_external_inputs: true
  icf_solver:
    min_tolerance: 1e-6
@endcode

These are the settings that belong to CENIC itself, as opposed to the ICF solver
it uses (which are in `icf_solver`). */
struct CenicIntegratorParameters {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(explicit_external_inputs));
    a->Visit(DRAKE_NVP(icf_solver));
  }

  /** Whether to treat the plant's external inputs (actuation and external
  forces) explicitly, i.e., hold them fixed at their values at the start of the
  step. When false (the default), they are linearized with respect to velocity
  using finite differences so that they can be treated implicitly, which
  requires re-evaluating all upstream systems several times per step. Setting
  this to true skips that work, which is useful when the inputs come from
  expensive external systems, at the cost of reduced stability for stiff
  feedback controllers. This option can be used with or without error control.
  See also @ref cenic_explicit_external_inputs "Explicit external inputs". */
  bool explicit_external_inputs{false};

  /** Parameters for the ICF convex solver used by CENIC. */
  contact_solvers::icf::IcfSolverParameters icf_solver;
};

}  // namespace multibody
}  // namespace drake
