#pragma once
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/c_iris_collision_geometry.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/geometry/optimization/cspace_separating_plane.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

// Note: the user should not include this header in their code. This header is
// created for internal use only.

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

void GenerateRationals(
    const std::vector<std::unique_ptr<
        CSpaceSeparatingPlane<symbolic::Variable>>>& separating_planes,
    const Vector3<symbolic::Variable> y_slack,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    std::vector<PlaneSeparatesGeometries>* plane_geometries);

// Returns the number of y_slack variables in `rational`.
// Not all y_slack necessarily appear in `rational`.
int GetNumYInRational(const symbolic::RationalFunction& rational,
                      const Vector3<symbolic::Variable>& y_slack);

/**
 * Given a plant and associated scene graph, returns all
 * the collision geometries.
 */
[[nodiscard]] std::map<multibody::BodyIndex,
                       std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph);

/**
 * Given a plant and associated scene graph compute all the possible collision
 * pairs between the bodies.
 * @param[in, out] collision_pairs A map from pairs of body indices to pairs of
 * collision geometries which can collide on each of the body indices.
 * @return The total number of collision pairs in the scene graph.
 */
[[nodiscard]] int GenerateCollisionPairs(
    const multibody::MultibodyPlant<double>& plant,
    const geometry::SceneGraph<double>& scene_graph,
    const std::map<multibody::BodyIndex,
                   std::vector<std::unique_ptr<CIrisCollisionGeometry>>>&
        link_geometries,
    std::map<SortedPair<multibody::BodyIndex>,
             std::vector<std::pair<const CIrisCollisionGeometry*,
                                   const CIrisCollisionGeometry*>>>*
        collision_pairs);

/**
   Solves a SeparationCertificateProgram with the given options
   @return result If we find the separation certificate, then `result` contains
   the separation plane and the Lagrangian polynomials; otherwise result is
   empty.
   */
[[nodiscard]] SeparationCertificateResultBase
SolveSeparationCertificateProgramBase(
    const SeparationCertificateProgramBase& certificate_program,
    const FindSeparationCertificateOptions& options,
    const CSpaceSeparatingPlane<symbolic::Variable>& separating_plane);

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
