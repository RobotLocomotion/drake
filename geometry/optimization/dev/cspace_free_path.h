#pragma once

#include <map>
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
 * Contains the information for adding the constraint that a polynomial be
 * positive on the unit interval. The polynomial is parametrized as some of
 * its indeterminates need to be evaluated before being added to the program.
 *
 * @param poly The parametrized polynomial we will enforce positivity of.
 * @param interval_variable The variable associated to the unit interval.
 * @param parameters The parameters which must be evaluated before enforcing the
 * positivity of poly.
 * @param auxillary_variables If poly is the polynomial associated to a
 * univariate matrix SOS program, these are the auxillary variables used to
 * convert the matrix SOS to a single polynomial.
 */
struct ParametrizdPolynomialPositiveOnUnitInterval {
 public:
  ParametrizdPolynomialPositiveOnUnitInterval(
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
  // A univariate polynomial q(μ) is nonnegative on [0, 1] if and
  // only if q(μ) = λ(μ) + ν(μ)*μ*(1-μ) if deg(q) = 2d with deg(λ) ≤ 2d and
  // deg(ν) ≤ 2d - 2 or q(μ) = λ(μ)*μ + ν(μ)*(1-μ) if deg(q) = 2d + 1 with
  // deg(λ) ≤ 2d and deg(ν) ≤ 2d and λ, ν are SOS. We construct the polynomial
  // p_(μ) = poly-q(μ) which we will later constrain to be equal
  // to 0.
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

  // the path parametrization variable going between 0 and 1
  const symbolic::Variable mu_;

  // a map storing the substitutions from the s_set_ variables to the path
  // parametrization.
  const std::unordered_map<symbolic::Variable, symbolic::Polynomial> path_;

  // friend declaration for use in constructor to avoid large initialization
  // lambda.
  friend std::unordered_map<symbolic::Variable, symbolic::Polynomial>
  initialize_path_map(CspaceFreePath* cspace_free_path,
                      unsigned int maximum_path_degree);
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
