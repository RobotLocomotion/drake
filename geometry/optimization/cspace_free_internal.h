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

/*
 Generate all the conditions (certain rationals being non-negative, and
 certain vectors with length <= 1) such that the robot configuration is
 collision free.
 @param[in] separating_planes A vector to non-null pointers to
 CSpaceSeparatingPlane objects containing the plane parameters as well as
 information about which bodies to separate.
 @param[in] y_slack The auxiliary variables required to enforce that capsules,
 spheres, and cylinders are on the correct side. See
 c_iris_collision_geometry.h/cc for details.
 @param[in] q_star The point about which the rational forward kinematics are
 taken. See rational_forward_kinematics.h/cc for details.
 @param[in] plane_geometries A non-null pointer to an empty vector.
 @param[out] plane_geometries Contains the separation information.
 */
void GenerateRationals(
    const std::vector<std::unique_ptr<
        CSpaceSeparatingPlane<symbolic::Variable>>>& separating_planes,
    const Vector3<symbolic::Variable>& y_slack,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const multibody::RationalForwardKinematics& rational_forward_kin,
    std::vector<PlaneSeparatesGeometries>* plane_geometries);

/*
 Returns the number of y_slack variables in `rational`.
 Not all y_slack necessarily appear in `rational`.
 */
[[nodiscard]] int GetNumYInRational(const symbolic::RationalFunction& rational,
                                    const Vector3<symbolic::Variable>& y_slack);

/*
 Given a plant and associated scene graph, returns all
 the collision geometries.
 @return A map ret, such that ret[body_index] returns all the
 CIrisCollisionGeometries attached to the body as body_index.
 */
[[nodiscard]] std::map<multibody::BodyIndex,
                       std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph);

/*
 Given a plant and associated scene graph compute all the possible collision
 pairs between the bodies.
 @param[in] link_geometries A map from body indices to a vector of non-null
 pointers to CIrisCollisionGeometry.
 @param[in, out] collision_pairs A non-null pointer to a map from pairs of
 body indices to pairs of collision geometries which can collide on each of
 the body indices.
 @return The total number of collision pairs in the scene graph.
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

/*
 Solves a SeparationCertificateProgram with the given options
 @param[in, out] result Will always contain the MathematicalProgramResult and
 plane_index associated to solving certificate_program. If a separation
 certificate is found (i.e. result->result.is_success) then result will also
 contain the plane parameters of the separating plane.
 */
void SolveSeparationCertificateProgramBase(
    const SeparationCertificateProgramBase& certificate_program,
    const FindSeparationCertificateOptions& options,
    const CSpaceSeparatingPlane<symbolic::Variable>& separating_plane,
    SeparationCertificateResultBase* result);

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
