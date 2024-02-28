#pragma once

/** @file
This is the file to #include to use SpanningForest. It includes
subsidiary headers for nested class definitions to keep file sizes
reasonable. */

// TODO(sherm1) Not used yet by MultibodyPlant; see PR #20225.

#define DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED

// Don't alpha-sort these internal includes; the order matters.
// clang-format off
#include "drake/multibody/topology/spanning_forest.h"
// TODO(sherm1) More to come.
// clang-format on

#undef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
