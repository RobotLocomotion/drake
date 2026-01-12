#pragma once

#include <list>
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/geometry/optimization/c_iris_collision_geometry.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/geometry/optimization/cspace_separating_plane.h"
#include "drake/geometry/optimization/dev/polynomial_positive_on_path.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 Contains the information to enforce a pair of geometries are separated by a
 plane. The conditions are that certain rational functions should be always
 positive.
 @param plane_geometries Contains the information required to separate two
 geometries over a range of configurations in the tangential configuration space
 variables s. These condition are rational function in the variables s and
 (sometimes) in auxillary variables y.
 @param mu The variable used to represent time in the path.
 @param path_with_y_subs Maps the configuration space variables s to Polynomials
 of mu and maps the auxillary variables y to symbolic::Polynomial(y).
 @param indeterminates Must contain mu and the auxillary variables y needed to
 implement the separating plane condition.
 @param cached_substitutions SubstituteAndExpandCacheData which can be used to
 speed up the substitutions of the path.
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

/**
 This class certifies whether polynomial trajectories of maximum degree d in the
 tangential-configuration space are collision free. By tangential-configuration
 space, we mean the revolute joint angle θ is replaced by t = tan(θ/2).
 */
class CspaceFreePath {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePath);
  /**
   @param plant The plant for which we compute the TC-space free trajectory. It
   must outlive this CspaceFreePath object.
   @param scene_graph The scene graph that has been connected with `plant`. It
   must outlive this CspaceFreePath object.
   @param q_star Refer to RationalForwardKinematics for its meaning.
   @param maximum_path_degree Paths will be univariate polynomials in a single
   indeterminate μ ∈ [0,1]. This is the maximum degree of the polynomial paths
   in  μ this CspaceFreePath will certify.
   @param plane_degree The certificate will be in terms of a polynomial,
   parametric hyperplane in the single indeterminate μ ∈ [0,1]. This is the
   degree of the planes used.
   */
  CspaceFreePath(const multibody::MultibodyPlant<double>* plant,
                 const geometry::SceneGraph<double>* scene_graph,
                 const Eigen::Ref<const Eigen::VectorXd>& q_star,
                 int maximum_path_degree, int plane_degree);

  ~CspaceFreePath() {}

  /**
   A certificate of safety that the pair of geometries are separated by a plane
   along the whole trajectory.
   */
  struct SeparationCertificateResult final : SeparationCertificateResultBase {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateResult);

    SeparationCertificateResult() {}
    ~SeparationCertificateResult() override;
  };

  /**
   A SeparationCertificateProgram which allows stores the path that this
   program certifies.
   @param[in] path maps each configuration space variable to a univariate
   polynomial of degree less than max_degree_ which corresponds to the path
   taken by that configuration space variable. The path must be given
   i.e. no polynomials in the path may contain decision variables.
   */
  struct SeparationCertificateProgram final : SeparationCertificateProgramBase {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateProgram);
    SeparationCertificateProgram(
        const std::unordered_map<symbolic::Variable, symbolic::Polynomial>
            m_path,
        int m_plane_index)
        : SeparationCertificateProgramBase(), path{std::move(m_path)} {
      plane_index = m_plane_index;
      for (const auto& item : path) {
        DRAKE_DEMAND(item.second.decision_variables().empty());
      }
    }

    ~SeparationCertificateProgram() override;

    std::unordered_map<symbolic::Variable, symbolic::Polynomial> path;
  };

  [[nodiscard]] const symbolic::Variable& mu() const { return mu_; }

  [[nodiscard]] const Vector3<symbolic::Variable>& y_slack() const {
    return y_slack_;
  }

  [[nodiscard]] int max_degree() const { return max_degree_; }

  [[nodiscard]] int plane_degree() const { return plane_degree_; }

  /**
   separating_planes()[map_geometries_to_separating_planes.at(geometry1_id,
   geometry2_id)] is the separating plane that separates geometry1 and
   geometry 2.
   */
  [[nodiscard]] const std::unordered_map<SortedPair<geometry::GeometryId>, int>&
  map_geometries_to_separating_planes() const {
    return map_geometries_to_separating_planes_;
  }

  [[nodiscard]] const std::vector<CSpaceSeparatingPlane<symbolic::Variable>>&
  separating_planes() const {
    return separating_planes_;
  }

  /**
   Constructs the SeparationCertificateProgram which searches for a
   separation certificate for a pair of geometries along a path in
   tangent-configuration space.
   @param[in] plane_geometries_on_path The geometry pair that we wish to
   certify is collision free along the path.
   @param[in] path A vector the same size as the plant's generalized
   positions with each entry a univariate polynomial. The path is the value
   of these polynomials as the variable varies between [0,1].
   */
  [[nodiscard]] SeparationCertificateProgram
  MakeIsGeometrySeparableOnPathProgram(
      const SortedPair<geometry::GeometryId>& geometry_pair,
      const VectorX<Polynomiald>& path) const;

  /**
   Solves a SeparationCertificateProgram with the given options
   @return result If we find the separation certificate, then `result`
   contains the separation plane and the Lagrangian polynomials; otherwise
   result is empty.
   */
  [[nodiscard]] SeparationCertificateResult SolveSeparationCertificateProgram(
      const SeparationCertificateProgram& certificate_program,
      const FindSeparationCertificateOptions& options) const;

 private:
  /**
   Generate all the conditions (certain rationals being non-negative, and
   certain vectors with length <= 1) such that the robot configuration is
   collision free as a function of the path variable.
  */
  void GeneratePathRationals(
      const std::vector<PlaneSeparatesGeometries>& plane_geometries);

  int GetSeparatingPlaneIndex(
      const SortedPair<geometry::GeometryId>& pair) const;

  multibody::RationalForwardKinematics rational_forward_kin_;
  const geometry::SceneGraph<double>& scene_graph_;

  Eigen::VectorXd q_star_;

  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
      link_geometries_;

  // The degree of the separating planes.
  const int plane_degree_;

  // The path parametrization variable going between 0 and 1.
  const symbolic::Variable mu_;

  // The maximum degree path this object can certify
  const int max_degree_;

  // A map storing the substitutions from the s_set_ variables to the path
  // parametrization.
  const std::unordered_map<symbolic::Variable, symbolic::Polynomial> path_;

  // We have the invariant plane_geometries_on_path_[i].plane_index == i.
  std::vector<PlaneSeparatesGeometriesOnPath> plane_geometries_on_path_;

  std::vector<CSpaceSeparatingPlane<symbolic::Variable>> separating_planes_;
  std::unordered_map<SortedPair<geometry::GeometryId>, int>
      map_geometries_to_separating_planes_;

  // Sometimes we need to impose that a certain matrix of polynomials are always
  // psd (for example with sphere or capsule collision geometries). We will use
  // this slack variable to help us impose the matrix-sos constraint.
  Vector3<symbolic::Variable> y_slack_;

  /**
   Constructs the SeparationCertificateProgram which searches for a
   separation certificate for a pair of geometries along the path.
   @param[in] plane_geometries_on_path Contain the parametric conditions
   that need to be non-negative on the path.
   @param[in] path maps each tangential-configuration space variable to a
   univariate polynomial of degree less than max_degree_. The path must be
   given i.e. no polynomials in the path may contain decision variables.
   */
  [[nodiscard]] SeparationCertificateProgram ConstructPlaneSearchProgramOnPath(
      const PlaneSeparatesGeometriesOnPath& plane_geometries_on_path,
      const std::unordered_map<symbolic::Variable, symbolic::Polynomial>& path)
      const;

  // Friend declaration for use in constructor to avoid large initialization
  // lambda.
  friend std::unordered_map<symbolic::Variable, symbolic::Polynomial>
  initialize_path_map(
      CspaceFreePath* cspace_free_path, int maximum_path_degree,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& s_variables);

  // Forward declaration the tester class. This tester class will expose the
  // private members of CspaceFreePath for unit test.
  friend class CspaceFreePathTester;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
