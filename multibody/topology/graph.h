#pragma once

/** @file This is the file to #include to use LinkJointGraph. It includes
subsidiary headers for nested class definitions to keep file sizes
reasonable. */

/* Terminology note: we use "Link" here to mean what MultibodyPlant calls a
"Body". In general, welded-together Links may be combined into a single body
and when there are loops a Link may be split into multiple bodies. So it is
very confusing to use the same term for the user's input and the internal
model we build. For clarity here, we need to clearly distinguish the
input graph from the spanning forest model we're going to build. To do that we
will use Link/Joint for the nodes and parent/child directed edges of the user's
input structure, and Body/Mobilizer (a.k.a. MobilizedBody) for the nodes and
inboard/outboard directed edges of the generated forest. */
// TODO(sherm1) Consider switching to "Link" in MultibodyPlant for clarity
//  and consistency.

#define DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED

// Don't alpha-sort these internal includes; the order matters.
// clang-format off
#include "drake/multibody/topology/link_joint_graph.h"
#include "drake/multibody/topology/link_joint_graph_link.h"
#include "drake/multibody/topology/link_joint_graph_joint.h"
#include "drake/multibody/topology/link_joint_graph_constraint.h"
#include "drake/multibody/topology/link_joint_graph_inlines.h"
// clang-format on

#undef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
