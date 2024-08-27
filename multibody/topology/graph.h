#pragma once

/* @file
This is the file to #include to use LinkJointGraph. It includes
subsidiary headers for nested class definitions to keep file sizes
reasonable. */

#define DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED

// Don't alpha-sort these internal includes; the order matters.
// clang-format off
#include "drake/multibody/topology/link_joint_graph.h"
#include "drake/multibody/topology/link_joint_graph_link.h"
#include "drake/multibody/topology/link_joint_graph_loop_constraint.h"
#include "drake/multibody/topology/link_joint_graph_joint.h"
#include "drake/multibody/topology/link_joint_graph_inlines.h"
// clang-format on

#undef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
