#pragma once

/* @file
This is the file to #include to use ICF. It includes subsidiary headers for
nested class definitions to keep file sizes reasonable. */

#define DRAKE_ICF_INCLUDED

// Don't alpha-sort these internal includes; the order matters.
// clang-format off
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/coupler_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/gain_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/limit_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/patch_constraints_pool.h"
// clang-format on

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfModel);

#undef DRAKE_ICF_INCLUDED
