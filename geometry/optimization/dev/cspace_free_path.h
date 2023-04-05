#pragma once

#include <list>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry/optimization/cspace_free_polytope.h"

#include "drake/common/eigen_types.h"
#include "drake/geometry/optimization/c_iris_separating_plane.h"
#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 Contains the information to enforce a pair of geometries are separated by a
 plane. The conditions are that certain rational functions should be always
 positive.
 */
struct PlaneSeparatesGeometriesOnPath {
  PlaneSeparatesGeometriesOnPath(
      const PlaneSeparatesGeometries& plane_geometries,
      const symbolic::Variable& mu,
      const std::unordered_map<symbolic::Variable, symbolic::Polynomial>&
          path_with_y_subs,
      const symbolic::Variables& indeterminates,
      symbolic::Polynomial::SubstituteAndExpandCacheData* cached_substitutions);

  // We use lists instead of vectors since
  // ParametrizedPolynomialPositiveOnUnitInterval is NO_COPY_NO_MOVE_NO_ASSIGN
  std::list<ParametrizedPolynomialPositiveOnUnitInterval>
      positive_side_conditions;
  std::list<ParametrizedPolynomialPositiveOnUnitInterval>
      negative_side_conditions;
  int plane_index;
};

class CspaceFreePath : public CspaceFreePolytope {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePath);
  /**
   @param plant The plant for which we compute the C-space free trajectory. It
   must outlive this CspaceFreePath object.
   @param scene_graph The scene graph that has been connected with `plant`. It
   must outlive this CspaceFreePath object.
   @param plane_order The order of the polynomials in the plane to separate a
   pair of collision geometries.
   @param q_star Refer to RationalForwardKinematics for its meaning.
   @param maximum_path_degree The maximum degree of the polynomial paths
   this CspaceFreePath will certify.
   */
  CspaceFreePath(const multibody::MultibodyPlant<double>* plant,
                 const geometry::SceneGraph<double>* scene_graph,
                 SeparatingPlaneOrder plane_order,
                 const Eigen::Ref<const Eigen::VectorXd>& q_star,
                 unsigned int maximum_path_degree,
                 const Options& options = Options{});

  ~CspaceFreePath() {}

  [[nodiscard]] const symbolic::Variable& mu() const { return mu_; }

//  /**
//   * Constructs the program that certifies that a given path is safe for the
//   * geometry pair.
//   * @param geometry_pair The geometry pair who's safety is to be certified.
//   * @param state_to_env_map A map containing the evaluation of the decision
//   * variables of the values in path_. Every key in path_ must be a key in state_to_env_map
//   *
//   * I.e. if k is a key in path_ then
//   * path_.at(k).Evaluate(state_to_env_map.at(k)) must not throw an error. Every key in path_ must be a key in
//   * @return
//   */
//  [[nodiscard]] const SeparationCertificateProgram
//  MakeIsPathSafeForGeometryProgram(
//      const SortedPair<geometry::GeometryId>& geometry_pair,
//      const std::unordered_map<symbolic::Variable, symbolic::Environment>&
//          state_to_env_map) const;
//
//  /**
//   Solves a SeparationCertificateProgram with the given options
//   @return result If we find the separation certificate, then `result` contains
//   the separation plane and the Lagrangian polynomials; otherwise result is
//   empty.
//   */
//  [[nodiscard]] std::optional<SeparationCertificateResult>
//  SolveIsPathSafeForGeometryProgram(
//      const SeparationCertificateProgram& certificate_program,
//      const FindSeparationCertificateGivenPolytopeOptions& options) const;
//
//  [[nodiscard]] bool IsPathSafe(
//      const std::unordered_map<symbolic::Variable, symbolic::Environment>&
//          state_to_env_map,
//      const IgnoredCollisionPairs& ignored_collision_pairs,
//      const FindSeparationCertificateGivenPolytopeOptions& options,
//      std::unordered_map<SortedPair<geometry::GeometryId>,
//                         SeparationCertificateResult>* certificates);

 protected:
  /**
   Generate all the conditions (certain rationals being non-negative, and
   certain vectors with length <= 1) such that the robot configuration is
   collision free as a function of the path variable.
   */
  void GeneratePathRationals();

 private:
  // Forward declaration the tester class. This tester class will expose the
  // private members of CspaceFreePath for unit test.
  friend class CspaceFreePathTester;

  // The path parametrization variable going between 0 and 1.
  const symbolic::Variable mu_;

  // A map storing the substitutions from the s_set_ variables to the path
  // parametrization.
  const std::unordered_map<symbolic::Variable, symbolic::Polynomial> path_;

  // We have the invariant plane_geometries_on_path_[i].plane_index == i. We use
  // a list instead of a vector as PlaneSeparatesGeometriesOnPath contains
  // objects which cannot be copied or moved.
  std::vector<PlaneSeparatesGeometriesOnPath> plane_geometries_on_path_;

  /**
   Constructs the program which searches for the plane separating a pair of
   geometries, for all configuration in the set {s | C * s <= d, s_lower <= s
   <= s_upper}.
   @param[in] plane_geometries Contain the conditions that need to be
   non-negative on the region C * s <= d and s_lower <= s <= s_upper.
   @param[in] d_minus_Cs d - C*s.
   @param[in] s_minus_s_lower s - s_lower.
   @param[in] s_upper_minus_s s_upper - s.
   @param[in] C_redundant_indices In the polyhedron C*s <= d, s_lower <= s <=
   s_upper, some rows of C*s<=d might be redundant. We store the indices of the
   redundant rows in C_redundant_indices.
   @param[in] s_lower_redundant_indices. Store the indices of the redundant rows
   in s >= s_lower.
   @param[in] s_upper_redundant_indices. Store the indices of the redundant rows
   in s <= s_upper.
   */
  [[nodiscard]] SeparationCertificateProgram ConstructPlaneSearchProgramForPair(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices) const;

  // Friend declaration for use in constructor to avoid large initialization
  // lambda.
  friend std::unordered_map<symbolic::Variable, symbolic::Polynomial>
  initialize_path_map(CspaceFreePath* cspace_free_path,
                      unsigned int maximum_path_degree);
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
