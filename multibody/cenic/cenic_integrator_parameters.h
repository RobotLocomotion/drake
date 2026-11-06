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
  icf_solver:
    min_tolerance: 1e-6
    explicit_external_inputs: true
@endcode

This struct exists as a place to add settings that belong to CENIC itself,
rather than to the ICF solver it uses. */
struct CenicIntegratorParameters {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(icf_solver));
  }

  /** Parameters for the ICF convex solver used by CENIC. */
  contact_solvers::icf::IcfSolverParameters icf_solver;
};

}  // namespace multibody
}  // namespace drake
