#pragma once

/* @file
This is the file to #include to use PooledSap. It includes
subsidiary headers for nested class definitions to keep file sizes
reasonable. */

#define DRAKE_POOLED_SAP_INCLUDED

// Don't alpha-sort these internal includes; the order matters.
// clang-format off
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_model.h"
#include "drake/multibody/contact_solvers/pooled_sap/patch_constraints_pool.h"
// clang-format on

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel);

#undef DRAKE_POOLED_SAP_INCLUDED
