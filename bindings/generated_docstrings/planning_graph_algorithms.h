#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/planning/graph_algorithms/graph_algorithms_internal.h"
// #include "drake/planning/graph_algorithms/max_clique_solver_base.h"
// #include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"
// #include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"
// #include "drake/planning/graph_algorithms/min_clique_cover_solver_base.h"
// #include "drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h"

// Symbol: pydrake_doc_planning_graph_algorithms
constexpr struct /* pydrake_doc_planning_graph_algorithms */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::graph_algorithms
      struct /* graph_algorithms */ {
        // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverBase
        struct /* MaxCliqueSolverBase */ {
          // Source: drake/planning/graph_algorithms/max_clique_solver_base.h
          const char* doc =
R"""(The problem of finding the maximum clique in a graph is known to be
NP-complete. This base class provides a unified interface for various
implementations of a solver for this problem which may be solved
rigorously or via heuristics.)""";
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverBase::MaxCliqueSolverBase
          struct /* ctor */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_base.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverBase::SolveMaxClique
          struct /* SolveMaxClique */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_base.h
            const char* doc =
R"""(Given the adjacency matrix of an undirected graph, find the maximum
clique within the graph. A clique is a collection of vertices in a
graph such that each pair of vertices is connected by an edge (i.e. a
fully connected subgraph). This problem is known to be NP-complete,
and so the concrete implementation of the solver determines whether
the return of this function is the true maximum clique in the graph
(which may take very long to compute), or only an approximate solution
found via heuristics.

This method throws if the adjacency matrix is not symmetric and may
throw depending on the concrete implementation of the solver.

Parameter ``adjacency_matrix``:
    a symmetric binary matrix encoding the edge relationship.

Returns:
    A binary vector with the same indexing as the adjacency matrix,
    with true indicating membership in the clique.)""";
          } SolveMaxClique;
        } MaxCliqueSolverBase;
        // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaGreedy
        struct /* MaxCliqueSolverViaGreedy */ {
          // Source: drake/planning/graph_algorithms/max_clique_solver_via_greedy.h
          const char* doc =
R"""(Approximately solves the maximum clique problem via a greedy
heuristic. Vertices are greedily added to the clique based on their
degree of connectivity. The algorithm initializes the clique with an
empty set and makes every vertex a candidate, then the degree of every
vertex is computed and the candidate vertex with the highest degree is
added to the clique. Afterwards, new candidate list is updated and the
previous two steps are repeated until no candidates are left.)""";
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaGreedy::MaxCliqueSolverViaGreedy
          struct /* ctor */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_via_greedy.h
            const char* doc = R"""()""";
          } ctor;
        } MaxCliqueSolverViaGreedy;
        // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaMip
        struct /* MaxCliqueSolverViaMip */ {
          // Source: drake/planning/graph_algorithms/max_clique_solver_via_mip.h
          const char* doc =
R"""(Solves the maximum clique problem to global optimality by solving the
mixed-integer program

Maximize ∑ᵢ xᵢ subject to xᵢ + xⱼ ≤ 1 if (i,j) is not in the edge xᵢ ∈
{0,1}.

Note: This solver requires the availability of a Mixed-Integer Linear
Programming solver (e.g. Gurobi and/or Mosek). We recommend enabling
those solvers if possible
(https://drake.mit.edu/bazel.html#proprietary_solvers).

Raises:
    RuntimeError if no Mixed-Integer Linear Programming solver is
    available.

Raises:
    RuntimeError if the initial guess has the wrong size for the
    provided adjacency matrix.)""";
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaMip::GetInitialGuess
          struct /* GetInitialGuess */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_via_mip.h
            const char* doc = R"""()""";
          } GetInitialGuess;
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaMip::GetSolverOptions
          struct /* GetSolverOptions */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_via_mip.h
            const char* doc = R"""()""";
          } GetSolverOptions;
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaMip::MaxCliqueSolverViaMip
          struct /* ctor */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_via_mip.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaMip::SetInitialGuess
          struct /* SetInitialGuess */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_via_mip.h
            const char* doc = R"""()""";
          } SetInitialGuess;
          // Symbol: drake::planning::graph_algorithms::MaxCliqueSolverViaMip::SetSolverOptions
          struct /* SetSolverOptions */ {
            // Source: drake/planning/graph_algorithms/max_clique_solver_via_mip.h
            const char* doc = R"""()""";
          } SetSolverOptions;
        } MaxCliqueSolverViaMip;
        // Symbol: drake::planning::graph_algorithms::MinCliqueCoverSolverBase
        struct /* MinCliqueCoverSolverBase */ {
          // Source: drake/planning/graph_algorithms/min_clique_cover_solver_base.h
          const char* doc = R"""()""";
          // Symbol: drake::planning::graph_algorithms::MinCliqueCoverSolverBase::MinCliqueCoverSolverBase
          struct /* ctor */ {
            // Source: drake/planning/graph_algorithms/min_clique_cover_solver_base.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::planning::graph_algorithms::MinCliqueCoverSolverBase::SolveMinCliqueCover
          struct /* SolveMinCliqueCover */ {
            // Source: drake/planning/graph_algorithms/min_clique_cover_solver_base.h
            const char* doc =
R"""(Given the adjacency matrix of an undirected graph, finds a
(potentially approximate) minimum clique cover of the graph. A clique
is a collection of vertices in a graph such that each pair of vertices
is connected by an edge (i.e. a fully connected subgraph). A clique
cover is a collection of cliques where each vertex in the graph is in
at least one of the cliques. This problem is known to be NP-complete,
and so the concrete implementation of the solver determines whether
the return of this function is the true minimum clique cover of the
graph (which may take very long to compute), or only an approximate
solution found via heuristics.

This method throws if the adjacency matrix is not symmetric and may
throw depending on the concrete implementation of the solver.

Parameter ``adjacency_matrix``:
    a symmetric binary matrix encoding the edge relationship.

Parameter ``partition``:
    If true, then every vertex is allowed to be covered exactly once.
    Otherwise, the same vertex may be covered multiple times.

Returns:
    A binary vector with the same indexing as the adjacency matrix,
    with true indicating membership in the clique.

Returns:
    The ith entry of the returned vector is a vector containing the
    indices of the ith clique.

Note that this method is intentionally not-const because some solvers
may use randomized algorithms.)""";
          } SolveMinCliqueCover;
        } MinCliqueCoverSolverBase;
        // Symbol: drake::planning::graph_algorithms::MinCliqueCoverSolverViaGreedy
        struct /* MinCliqueCoverSolverViaGreedy */ {
          // Source: drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h
          const char* doc =
R"""(Approximately solves the min clique cover problem via a greedy
heuristic.

At each step, the largest clique found by the provided max clique
solver is added to the clique cover. These vertices are then removed
from graph and max clique is solved again. This loop continues until a
clique is found which is smaller than the min clique size.

Note:
    if min clique size > 1, then this class will not strictly compute
    a clique cover since not all vertices will be covered.)""";
          // Symbol: drake::planning::graph_algorithms::MinCliqueCoverSolverViaGreedy::MinCliqueCoverSolverViaGreedy
          struct /* ctor */ {
            // Source: drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h
            const char* doc =
R"""(Constructs using the given max clique solver.)""";
          } ctor;
          // Symbol: drake::planning::graph_algorithms::MinCliqueCoverSolverViaGreedy::get_min_clique_size
          struct /* get_min_clique_size */ {
            // Source: drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h
            const char* doc = R"""()""";
          } get_min_clique_size;
          // Symbol: drake::planning::graph_algorithms::MinCliqueCoverSolverViaGreedy::set_min_clique_size
          struct /* set_min_clique_size */ {
            // Source: drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h
            const char* doc =
R"""(Set the minimum clique size. Throws if this is less than 1.

Parameter ``min_clique_size``:)""";
          } set_min_clique_size;
        } MinCliqueCoverSolverViaGreedy;
      } graph_algorithms;
    } planning;
  } drake;
} pydrake_doc_planning_graph_algorithms;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
