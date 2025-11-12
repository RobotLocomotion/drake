#pragma once

/* @file
This is the file to #include to use ICF. It includes subsidiary headers for
nested class definitions to keep file sizes reasonable.

ICF solves the convex Irrotational Contact Fields problem

    minᵥ ℓ(v; q₀, v₀, δt)

to produce next-step velocities v, as described in [Castro et al., 2023]. The
CENIC integrator builds on ICF to perform error-controlled integration.

This implementation of ICF focuses on simplicity and performance:
   - Contiguous memory layouts for better cache locality.
   - Memory allocations are performed once at simulation startup, typically with
     overestimation. Reallocation is permitted but should occur at most once.
   - Constraints are grouped into patches, enabling re-use of per-patch
     computations and reduced FLOP count.
   - Contact constraints are expressed directly in world coordinates,
     eliminating expensive intermediate frame computations.

Key components and how they talk to each other:

   - IcfSolver: Coming soon (#23747), this will solve the convex problem itself.
   - IcfBuilder: Constructs the optimization problem. The builder is the only
     component that knows about MultibodyPlant.
   - IcfModel: Represents the convex ICF problem min ℓ(v; q₀, v₀, δt) for a
     given state (q₀, v₀) and time step δt. The model is constant during the
     optimization process, e.g., for different values of decision variables v.
   - IcfData: Holds all variable data that changes during the optimization
     process, e.g., v and derived quantities.

Other components:

   - Constraint pools (CouplerConstraintsPool, GainConstraintsPool,
     LimitConstraintsPool, PatchConstraintsPool) are part of IcfModel, and hold
     constraints of various types.
   - Similarly, constraint data pools (CouplerConstraintsDataPool,
     GainConstraintsDataPool, LimitConstraintsDataPool,
     PatchConstraintsDataPool) are part of IcfData, and hold data that change
     with v for the corresponding constraints.
   - The underlying EigenPool datatype is used to store constraint quantities in
     contiguous memory blocks. EigenPool essentially provides something like
     std::vector<MatrixX<T>>, but with contiguous storage.

References:
  [Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. Irrotational
  Contact Fields. https://arxiv.org/abs/2312.03908
*/

#define DRAKE_ICF_INCLUDED

// Don't alpha-sort these internal includes; the order matters.
// clang-format off
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/icf_model_coupler_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_model_gain_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_model_limit_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_model_patch_constraints_pool.h"
// clang-format on

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfModel);

#undef DRAKE_ICF_INCLUDED
