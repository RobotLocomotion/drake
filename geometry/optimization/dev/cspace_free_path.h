#pragma once

#include <list>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/optimization/c_iris_separating_plane.h"
#include "drake/geometry/optimization/cspace_free_polytope.h"
#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"
#include "drake/common/polynomial.h"


#include <Eigen/Core>


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

  [[nodiscard]] int max_degree() const { return max_degree_; }

  [[nodiscard]] SeparationCertificateProgram
  MakeIsGeometrySeparableOnPathProgram(
      const SortedPair<geometry::GeometryId>& geometry_pair,
      const VectorX<Polynomiald>& path) const;

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

  // The maximum degree path this object can certify
  const int max_degree_;

  // A map storing the substitutions from the s_set_ variables to the path
  // parametrization.
  const std::unordered_map<symbolic::Variable, symbolic::Polynomial> path_;


  // We have the invariant plane_geometries_on_path_[i].plane_index == i. We use
  // a list instead of a vector as PlaneSeparatesGeometriesOnPath contains
  // objects which cannot be copied or moved.
  std::vector<PlaneSeparatesGeometriesOnPath> plane_geometries_on_path_;

  /**
   Constructs the MathematicalProgram which searches for a separation
   certificate for a pair of geometries along the path.
   @param[in] plane_geometries_on_path Contain the parametric conditions that
   need to be non-negative on the path.
   @param[in] path maps each configuration space variable to a univariate
   polynomial of degree less than max_degree_.
   */
  [[nodiscard]] SeparationCertificateProgram ConstructPlaneSearchProgramOnPath(
      const PlaneSeparatesGeometriesOnPath& plane_geometries_on_path,
      const std::unordered_map<symbolic::Variable,
                               Polynomial<double>>& path) const;

  // Friend declaration for use in constructor to avoid large initialization
  // lambda.
  friend std::unordered_map<symbolic::Variable, symbolic::Polynomial>
  initialize_path_map(CspaceFreePath* cspace_free_path,
                      unsigned int maximum_path_degree);
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
