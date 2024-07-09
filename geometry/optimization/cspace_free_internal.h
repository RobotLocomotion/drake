#pragma once

#include <array>
#include <map>
#include <memory>
#include <optional>
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
 Overloads GenerateRationals.
 Use separating_planes as the map of the plane_index to the separating plane.
 */
void GenerateRationals(
    const std::map<int, const CSpaceSeparatingPlane<symbolic::Variable>*>&
        separating_planes,
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

// Returns the total size of the lower triangular variables in the Gram
// matrices. Each Gram matrix should match with the monomial basis in
// `monomial_basis_array`. Depending on whether we include y in the
// indeterminates (see Options::with_cross_y for more details) and the size of
// y, the number of Gram matrices will change.
// @param monomial_basis The candidate monomial_basis for all gram matricies.
// @param with_cross_y See Options::with_cross_y
// @param num_y The size of y indterminates.
int GetGramVarSize(
    const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array,
    bool with_cross_y, int num_y);

// Given the monomial_basis_array, compute the sos polynomial.
// monomial_basis_array contains [m(s), y₀*m(s), y₁*m(s), y₂*m(s)].
//
// If num_y == 0, then the sos polynomial is just
// m(s)ᵀ * X * m(s)
// where X is a Gram matrix, `grams` is a length-1 vector containing X.
//
// If num_y != 0 and with_cross_y = true, then the sos polynomial is
// ⌈    m(s)⌉ᵀ * Y * ⌈    m(s)⌉
// | y₀*m(s)|        | y₀*m(s)|
// |   ...  |        |   ...  |
// ⌊ yₙ*m(s)⌋        ⌊ yₙ*m(s)⌋
// where n = num_y-1. Y is a Gram matrix, `grams` is a length-1 vector
// containing Y.
//
// if num_y != 0 and with_cross_y = false, then the sos polynomial is
// ∑ᵢ ⌈    m(s)⌉ᵀ * Zᵢ * ⌈    m(s)⌉
//    ⌊ yᵢ*m(s)⌋         ⌊ yᵢ*m(s)⌋
// where Zᵢ is a Gram matrix, i = 0, ..., num_y-1.  `grams` is a vector of
// length `num_y`, and grams[i] = Zᵢ
struct GramAndMonomialBasis {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GramAndMonomialBasis);

  GramAndMonomialBasis(
      const std::array<VectorX<symbolic::Monomial>, 4>& monomial_basis_array,
      bool with_cross_y, int num_y);

  // Adds the constraint that the polynomial represented by this Gram and
  // monomial basis is sos.
  void AddSos(solvers::MathematicalProgram* prog,
              const Eigen::Ref<const VectorX<symbolic::Variable>>& gram_lower,
              symbolic::Polynomial* poly);

  int gram_var_size;
  std::vector<MatrixX<symbolic::Variable>> grams;
  std::vector<VectorX<symbolic::Monomial>> monomial_basis;
};

// Solves an optimization problem. If the optimization problem has a cost, then
// after we find the optimal solution for that cost (where the optimal solution
// would be on the boundary of the feasible set), we back-off a little bit and
// only find a strictly feasible solution in the strict interior of the
// feasible set. This helps the next iteration of the bilinear alternation.
// @note that `prog` will be mutated after this function call if it has a cost.
solvers::MathematicalProgramResult SolveWithBackoff(
    solvers::MathematicalProgram* prog, std::optional<double> backoff_scale,
    const std::optional<solvers::SolverOptions>& solver_options,
    const solvers::SolverId& solver_id);

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
