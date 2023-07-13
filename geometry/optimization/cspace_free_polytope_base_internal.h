#pragma once
/**
 * The users should not include this header file. It is for internal usage.
 */

#include <optional>
#include <vector>

#include "drake/geometry/optimization/cspace_free_polytope_base.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/solvers/choose_best_solver.h"

namespace drake {
namespace geometry {
namespace optimization {
/*
 Given a C-space polytope, find the separation certificates for the geometries
 in `plane_geometries`.
 @param plane_geometries All the planes for every pair of geometries that might
 get in collision (not taking ignored_collision_pairs into consideration).
 @param cspace_polytope A description of the C-space polytope region.
 */
template <typename CspacePolytopeType, typename CertificateProgramType,
          typename SeparationCertificateResultType>
void FindSeparationCertificateGivenCspacePolytope(
    const CspaceFreePolytopeBase& cspace_free_polytope,
    const std::vector<PlaneSeparatesGeometries>& plane_geometries,
    const CspaceFreePolytopeBase::IgnoredCollisionPairs&
        ignored_collision_pairs,
    const CspacePolytopeType& cspace_polytope,
    const FindSeparationCertificateOptions& options,
    const std::function<CertificateProgramType(const PlaneSeparatesGeometries&,
                                               const CspacePolytopeType&)>&
        construct_plane_search_program_fun,
    std::vector<std::optional<SeparationCertificateResultType>>*
        separation_results) {
  DRAKE_DEMAND(plane_geometries.size() ==
               cspace_free_polytope.separating_planes().size());
  // Stores the indices in separating_planes() that don't appear in
  // ignored_collision_pairs.
  std::vector<int> active_plane_indices;
  active_plane_indices.reserve(cspace_free_polytope.separating_planes().size());
  for (int i = 0;
       i < static_cast<int>(cspace_free_polytope.separating_planes().size());
       ++i) {
    if (ignored_collision_pairs.count(SortedPair<geometry::GeometryId>(
            cspace_free_polytope.separating_planes()[i]
                .positive_side_geometry->id(),
            cspace_free_polytope.separating_planes()[i]
                .negative_side_geometry->id())) == 0) {
      active_plane_indices.push_back(i);
    }
  }
  *separation_results =
      std::vector<std::optional<SeparationCertificateResultType>>(
          active_plane_indices.size(), std::nullopt);
  // This lambda function formulates and solves a small SOS program for each
  // pair of geometries.
  auto solve_small_sos = [&cspace_free_polytope, &plane_geometries,
                          &cspace_polytope, &construct_plane_search_program_fun,
                          &active_plane_indices, &options,
                          &separation_results](int plane_count) {
    const int plane_index = active_plane_indices[plane_count];
    auto certificate_program = construct_plane_search_program_fun(
        plane_geometries[plane_index], cspace_polytope);
    solvers::MathematicalProgramResult result;
    solvers::MakeSolver(options.solver_id)
        ->Solve(*certificate_program.prog, std::nullopt, options.solver_options,
                &result);
    if (result.is_success()) {
      (*separation_results)[plane_count].emplace(
          certificate_program.certificate.GetSolution(
              plane_index,
              cspace_free_polytope.separating_planes()[plane_index].a,
              cspace_free_polytope.separating_planes()[plane_index].b,
              cspace_free_polytope.separating_planes()[plane_index]
                  .decision_variables,
              result));
      return true;
    } else {
      (*separation_results)[plane_count].reset();
      return false;
    }
  };

  cspace_free_polytope.SolveCertificationForEachPlaneInParallel(
      active_plane_indices, solve_small_sos, options.num_threads,
      options.verbose, options.terminate_at_failure);
}
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
