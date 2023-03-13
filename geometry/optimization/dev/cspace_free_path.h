#pragma once

#include <map>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry/optimization/cspace_free_polytope.h"

#include "drake/common/eigen_types.h"
#include "drake/geometry/optimization/c_iris_separating_plane.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 * Certifying a path as collision free will require certifying that many matrix
 * SOS conditions yᵀP(μ;s)y ≥ 0 for μ ∈ [0,1] where the s is a multivariate
 * parameter which must be evaluated before the optimization program is solved.
 * This class contains the information for adding the constraint that a
 * polynomial be positive on the unit interval. The polynomial is parametrized
 * as some of its indeterminates needing to be evaluated before being added to
 * the program.
 *
 * @param poly The parametrized polynomial we will enforce positivity of.
 * @param interval_variable The variable μ associated to the unit interval.
 * @param parameters The parameters which must be evaluated before enforcing the
 * positivity of poly.
 * @param auxillary_variables If poly is the polynomial associated to a
 * univariate matrix SOS program, these are the auxillary variables used to
 * convert the matrix SOS to a single polynomial.
 */
class ParametrizedPolynomialPositiveOnUnitInterval {
 public:
  ParametrizedPolynomialPositiveOnUnitInterval(
      const symbolic::Polynomial& poly,
      const symbolic::Variable& interval_variable,
      const symbolic::Variables& parameters,
      const std::optional<const symbolic::Variables>& auxillary_variables =
          std::nullopt);

  // Add the constraint that this parametrized polynomial is positive on the
  // unit interval. The Environment env must contain an evaluation for all the
  // parameters in parameters_.
  void AddPositivityConstraintToProgram(solvers::MathematicalProgram* prog,
                                        symbolic::Environment env);

 private:
  // TODO(Alexandre.Amice) make members const.

  // A polynomial q(μ,y) = ∑ f(μ)yᵢyⱼ (where μ is univariate and y is
  // multivariate) is positive on the interval μ ∈ [0,1] if and only if there
  // exists biforms λ(μ,y) = ∑ ϕᵢ²(μ,y) and ν(μ,y) = ∑ ψᵢ²(μ,y) such that
  // q(μ,y) = λ(μ,y) + ν(μ,y)*μ*(1-μ) if deg(q, μ) = 2d or q(μ,y) = λ(μ,y)*μ +
  // ν(μ,y)*(1-μ) if deg(q, μ) = 2d + 1. Moreover, in both cases deg(ϕᵢ, μ) ≤ d,
  // deg(ϕᵢ, y) = 1, and and deg(ψᵢ, y) = 1. In the first case
  // deg(ψᵢ, μ) ≤ d - 1, and in the second deg(ψᵢ, μ) ≤ d. If deg(poly,μ) > 0 we
  // construct the polynomial p_(μ,y) = poly(μ,y)-q(μ,y) which we will later
  // constrain to be equal to 0.
  symbolic::Polynomial p_;

  // A program which stores the psd variables and constraints associated to λ
  // and ν. See the description of p_.
  solvers::MathematicalProgram psatz_variables_and_psd_constraints_;

  // The parameters in the polynomial p_ which must be evaluated before the
  // positivity constraint is added.
  const symbolic::Variables parameters_;
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

  // Friend declaration for use in constructor to avoid large initialization
  // lambda.
  friend std::unordered_map<symbolic::Variable, symbolic::Polynomial>
  initialize_path_map(CspaceFreePath* cspace_free_path,
                      unsigned int maximum_path_degree);
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
