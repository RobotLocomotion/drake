#pragma once

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/geometry/optimization/c_iris_collision_geometry.h"
#include "drake/geometry/optimization/c_iris_separating_plane.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace geometry {
namespace optimization {
/**
 Contains the information to enforce a pair of geometries are separated by a
 plane. The conditions are that certain rational functions should be always
 positive.
 */
struct PlaneSeparatesGeometries {
  PlaneSeparatesGeometries(
      std::vector<symbolic::RationalFunction> m_positive_side_rationals,
      std::vector<symbolic::RationalFunction> m_negative_side_rationals,
      int m_plane_index)
      : positive_side_rationals{std::move(m_positive_side_rationals)},
        negative_side_rationals{std::move(m_negative_side_rationals)},
        plane_index{m_plane_index} {}
  const std::vector<symbolic::RationalFunction> positive_side_rationals;
  const std::vector<symbolic::RationalFunction> negative_side_rationals;
  int plane_index;
};

/**
 This class tries to find large convex region in the configuration space, such
 that this whole convex set is collision free.
 For more details, refer to the paper
 "Certified Polyhedral Decompositions of Collision-Free COnfiguration Space"
 by Hongkai Dai*, Alexandre Amice*, Peter Werner, Annan Zhang and Russ Tedrake.
 A conference version of this paper is published at
 "Finding and Optimizing Certified, Colision-Free Regions in Configuration Space
 for Robot Manipulators" by Alexandre Amice, Hongkai Dai, Peter Werner, Annan
 Zhang and Russ Tedrake, 2022.
 */
class CspaceFreePolytope {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePolytope)

  using IgnoredCollisionPairs =
      std::unordered_set<SortedPair<geometry::GeometryId>>;

  ~CspaceFreePolytope() {}

  /** Optional argument for constructing CspaceFreePolytope */
  struct Options {
    Options() {}

    // For non-polytopic collision geometries, we will impose a matrix-sos
    // constraint X(s) being psd, with a slack indeterminates y, such that the
    // polynomial
    // p(s, y) = ⌈ 1 ⌉ᵀ * X(s) * ⌈ 1 ⌉
    //           ⌊ y ⌋           ⌊ y ⌋
    // is positive. This p(s, y) polynomial doesn't contain the cross term of y
    // (namely it doesn't have y(i)*y(j), i≠j). When we select the monomial
    // basis for this polynomial, we can also exclude the cross term of y in the
    // monomial basis.
    //
    // To illustrate the idea, let's consider the following toy example: if we
    // want to certify that
    // a(0) + a(1)*y₀ + a(2)*y₁ + a(3)*y₀² + a(4)*y₁² is positive
    // (this polynomial doesn't have the cross term y₀*y₁), we can write it as
    // ⌈ 1⌉ᵀ * A₀ * ⌈ 1⌉ + ⌈ 1⌉ᵀ * A₁ * ⌈ 1⌉
    // ⌊y₀⌋         ⌊y₀⌋   ⌊y₁⌋         ⌊y₁⌋
    // with two small psd matrices A₀, A₁
    // Instead of
    // ⌈ 1⌉ᵀ * A * ⌈ 1⌉
    // |y₀|        |y₀|
    // ⌊y₁⌋        ⌊y₁⌋
    // with one large psd matrix A. The first parameterization won't have the
    // cross term y₀*y₁ by construction, while the second parameterization
    // requires imposing extra constraints on certain off-diagonal terms in A
    // so that the cross term vanishes.
    //
    // If we set with_cross_y = false, then we will use the monomial basis that
    // doesn't generate cross terms of y, leading to smaller size sos problems.
    // If we set with_cross_y = true, then we will use the monomial basis that
    // will generate cross terms of y, causing larger size sos problems, but
    // possibly able to certify a larger C-space polytope.
    bool with_cross_y{false};
  };

  /**
   @param plant The plant for which we compute the C-space free polytopes. It
   must outlive this CspaceFreePolytope object.
   @param scene_graph The scene graph that has been connected with `plant`. It
   must outlive this CspaceFreePolytope object.
   @param plane_order The order of the polynomials in the plane to separate a
   pair of collision geometries.
   @param q_star Refer to RationalForwardKinematics for its meaning.
   */
  CspaceFreePolytope(const multibody::MultibodyPlant<double>* plant,
                     const geometry::SceneGraph<double>* scene_graph,
                     SeparatingPlaneOrder plane_order,
                     const Eigen::Ref<const Eigen::VectorXd>& q_star,
                     const Options& options = Options{});

  [[nodiscard]] const multibody::RationalForwardKinematics&
  rational_forward_kin() const {
    return rational_forward_kin_;
  }

  /**
   separating_planes()[map_geometries_to_separating_planes.at(geometry1_id,
   geometry2_id)] is the separating plane that separates geometry1 and
   geometry 2.
   */
  [[nodiscard]] const std::unordered_map<SortedPair<geometry::GeometryId>, int>&
  map_geometries_to_separating_planes() const {
    return map_geometries_to_separating_planes_;
  }

  [[nodiscard]] const std::vector<CIrisSeparatingPlane<symbolic::Variable>>&
  separating_planes() const {
    return separating_planes_;
  }

  [[nodiscard]] const Vector3<symbolic::Variable>& y_slack() const {
    return y_slack_;
  }

  /**
   When searching for the separating plane, we want to certify that the
   numerator of a rational is non-negative in the C-space region C*s<=d,
   s_lower<= s <=s_upper. Hence for each of the rational we will introduce
   Lagrangian multipliers for the polytopic constraint d-C*s >= 0, s - s_lower
   >= 0, s_upper - s >= 0.
   */
  struct SeparatingPlaneLagrangians {
    SeparatingPlaneLagrangians(int C_rows, int s_size)
        : polytope(C_rows), s_lower(s_size), s_upper(s_size) {}
    // The Lagrangians for d - C*s >= 0.
    VectorX<symbolic::Polynomial> polytope;
    // The Lagrangians for s - s_lower >= 0.
    VectorX<symbolic::Polynomial> s_lower;
    // The Lagrangians for s_upper - s >= 0.
    VectorX<symbolic::Polynomial> s_upper;

    [[nodiscard]] SeparatingPlaneLagrangians GetSolution(
        const solvers::MathematicalProgramResult& result) const;
  };

  /**
   We certify that a pair of geometries is collision free in the C-space region
   {s | Cs<=d, s_lower<=s<=s_upper}, by finding the separating plane and the
   Lagrangian multipliers. This struct contains the certificate, that the
   separating plane {x | aᵀx+b=0 } separates the two geometries in
   separating_planes()[plane_index] in the C-space polytope.
   */
  struct SeparationCertificateResult {
    int plane_index;
    std::vector<SeparatingPlaneLagrangians> positive_side_rational_lagrangians;
    std::vector<SeparatingPlaneLagrangians> negative_side_rational_lagrangians;
    // The separating plane is { x | aᵀx+b=0 }
    Vector3<symbolic::Polynomial> a;
    symbolic::Polynomial b;
    // The value of the plane.decision_variables at solution. This field is used
    // for debugging.
    Eigen::VectorXd plane_decision_var_vals;
  };

  /**
   This struct stores the necessary information to search for the separating
   plane for the polytopic C-space region C*s <= d, s_lower <= s <= s_upper.
   We need to impose that N rationals are non-negative in this C-space polytope.
   The denominator of each rational is always positive hence we need to impose
   the N numerators are non-negative in this C-space region.
   We impose the condition
   numerator_i(s) - λ(s)ᵀ * (d - C*s) - λ_lower(s)ᵀ * (s - s_lower)
         -λ_upper(s)ᵀ * (s_upper - s) is sos
   λ(s) are sos, λ_lower(s) are sos, λ_upper(s) are sos.
   */
  struct SeparationCertificate {
    SeparationCertificate() {}

    [[nodiscard]] SeparationCertificateResult GetSolution(
        int plane_index, const Vector3<symbolic::Polynomial>& a,
        const symbolic::Polynomial& b,
        const VectorX<symbolic::Variable>& plane_decision_vars,
        const solvers::MathematicalProgramResult& result) const;

    // positive_side_rational_lagrangians[i] is the Lagrangian multipliers for
    // PlaneSeparatesGeometries::positive_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> positive_side_rational_lagrangians;
    // negative_side_rational_lagrangians[i] is the Lagrangian multipliers for
    // PlaneSeparatesGeometries::negative_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> negative_side_rational_lagrangians;
  };

  struct SeparationCertificateProgram {
    SeparationCertificateProgram()
        : prog{new solvers::MathematicalProgram()}, certificate{} {}
    /// The program that stores all the constraints to search for the separating
    /// plane and Lagrangian multipliers as certificate.
    std::unique_ptr<solvers::MathematicalProgram> prog;
    SeparationCertificate certificate;
    int plane_index;
  };

  struct FindSeparationCertificateGivenPolytopeOptions {
    // We can find the certificate for each pair of geometries in parallel.
    // num_threads specifies how many threads we run in parallel. If num_threads
    // <=0, then we use all available threads on the computer.
    int num_threads{-1};

    // If verbose set to true, then we will print some information to the
    // terminal.
    bool verbose{false};

    // The solver invoked for the sos program.
    solvers::SolverId solver_id{solvers::MosekSolver::id()};

    // If the SOS in one thread fails, then don't launch any more threads.
    bool terminate_at_failure{true};

    // The solver options used for the SOS program.
    std::optional<solvers::SolverOptions> solver_options{std::nullopt};

    // If a row in C*s<=d is redundant (this row is implied by other rows in
    // C*s<=d, s_lower<=s<=s_upper), then we don't search for the Lagrangian
    // multiplier for this row.
    bool ignore_redundant_C{false};
  };

  /** Finds the certificates that the C-space polytope {s | C*s<=d, s_lower <= s
   * <= s_upper} is collision free.
   *
   * @param C The C-space polytope is {s | C*s<=d, s_lower<=s<=s_upper}
   * @param d The C-space polytope is {s | C*s<=d, s_lower<=s<=s_upper}
   * @param ignored_collision_pairs We will ignore the pair of geometries in
   * `ignored_collision_pairs`.
   * @param[out] certificates Contains the certificate we successfully found for
   * each pair of geometries. Notice that depending on `options`, the program
   * could search for the certificate for each geometry pair in parallel, and
   * will terminate the search once it fails to find the certificate for any
   * pair.
   * @retval success If true, then we have certified that the C-space polytope
   * {s | C*s<=d, s_lower<=s<=s_upper} is collision free. Otherwise
   * success=false.
   */
  [[nodiscard]] bool FindSeparationCertificateGivenPolytope(
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::VectorXd>& d,
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const FindSeparationCertificateGivenPolytopeOptions& options,
      std::unordered_map<SortedPair<geometry::GeometryId>,
                         SeparationCertificateResult>* certificates) const;

  /**
   Adds the constraint that each column of s_inner_pts is in the polytope {s |
   C*s<=d}.
   */
  void AddCspacePolytopeContainment(solvers::MathematicalProgram* prog,
                                    const MatrixX<symbolic::Variable>& C,
                                    const VectorX<symbolic::Variable>& d,
                                    const Eigen::MatrixXd& s_inner_pts) const;

  /**
   The cost used when fixing the Lagrangian multiplier and search for C and d in
   the C-space polytope {s | C*s <=d, s_lower<=s<=s_upper}. We denote δᵢ as the
   margin between the i'th face C.row(i)<=d(i) to the inscribed ellipsoid.
   */
  enum EllipsoidMarginCost {
    kSum,            ///< Maximize ∑ᵢδᵢ
    kGeometricMean,  ///< Maximize the geometric mean power(∏ᵢ (δᵢ + ε), 1/n)
                     ///< where n is C.rows(),
                     ///< ε=FindPolytopeGivenLagrangianOptions.ellipsoid_margin_epsilon.
                     ///< # NOLINT
  };

  struct FindPolytopeGivenLagrangianOptions {
    std::optional<double> backoff_scale{std::nullopt};

    // We will maximize the cost ∏ᵢ (δᵢ + ε) where δᵢ is the margin from each
    // face of the polytope {s | Cs<=d} to the inscribed ellipsoid, ε is
    // ellipsoid_margin_epsilon, a small positive constant to make sure δᵢ + ε
    // being strictly positive.
    double ellipsoid_margin_epsilon{1E-5};

    solvers::SolverId solver_id{solvers::MosekSolver::id()};

    std::optional<solvers::SolverOptions> solver_options{std::nullopt};

    // We can constrain the C-space polytope {s | C*s<=d, s_lower<=s<=s_upper}
    // to contain some sampled s. Each column of s_inner_pts is a sample of s.
    std::optional<Eigen::MatrixXd> s_inner_pts;

    // If set to true, then we will also search for the Lagrangian multipliers
    // for the constraint s_lower <= s <= s_upper; otherwise we fix the
    // Lagrangian multiplier to the solution found when we fix the C-space
    // polytope {s | C*s<=d, s_lower<=s<=s_upper}.
    bool search_s_bounds_lagrangians{true};

    EllipsoidMarginCost ellipsoid_margin_cost{
        EllipsoidMarginCost::kGeometricMean};
  };

  struct SearchResult {
    Eigen::MatrixXd C;
    Eigen::VectorXd d;
    // This is the certified C-space polytope {s | C * s <= d, s_lower <= s <=
    // s_upper}.
    HPolyhedron certified_polytope;
    // a[i].dot(x) + b[i]=0 is the separation plane for separating_planes()[i].
    std::unordered_map<int, Vector3<symbolic::Polynomial>> a;
    std::unordered_map<int, symbolic::Polynomial> b;
    // The number of iterations at termination.
    int num_iter;

    // Clear this->a and this->b and reset their values.
    void SetSeparatingPlanes(
        const std::vector<std::optional<SeparationCertificateResult>>&
            certificates_result);

    // Update this->a and this->b with the values in certificates_result.
    void UpdateSeparatingPlanes(
        const std::vector<std::optional<SeparationCertificateResult>>&
            certificates_results);
  };

  struct BilinearAlternationOptions {
    int max_iter{10};
    double convergence_tol{1E-3};
    FindPolytopeGivenLagrangianOptions find_polytope_options;
    FindSeparationCertificateGivenPolytopeOptions find_lagrangian_options;
    /** After finding the maximal inscribed ellipsoid in C-space polytope {s |
     * C*s<=d, s_lower<=s<=s_upper}, we scale this ellipsoid by
     * ellipsoid_scaling, and require the new C-space polytope to contain this
     * scaled ellipsoid. ellipsoid_scaling=1 corresponds to no scaling.
     */
    double ellipsoid_scaling{0.99};
  };

  /** Search for a collision-free C-space polytope.
   {s | C*s<=d, s_lower<=s<=s_upper} through bilinear alternation.
   The goal is to maximize the volume the C-space polytope. Since we can't
   compute the polytope volume in the closed form, we use the volume of the
   maximal inscribed ellipsoid as a surrogate function of the polytope volume.
   @param ignored_collision_pairs The pairs of geometries that we ignore when
   searching for separation certificates.
   @param C_init The initial value of C.
   @param d_init The initial value of d.
   @param options The options for the bilinear alternation.
   @retval results Stores the certification result in each iteration of the
   bilinear alternation.
   */
  [[nodiscard]] std::vector<SearchResult> SearchWithBilinearAlternation(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const Eigen::Ref<const Eigen::MatrixXd>& C_init,
      const Eigen::Ref<const Eigen::VectorXd>& d_init,
      const BilinearAlternationOptions& options) const;

  struct BinarySearchOptions {
    double scale_max{1};
    double scale_min{0.01};
    int max_iter{10};
    double convergence_tol{1E-3};
    FindSeparationCertificateGivenPolytopeOptions find_lagrangian_options;
  };

  /** Binary search on d such that the C-space polytope {s | C*s<=d,
   s_lower<=s<=s_upper} is collision free.
   We scale the polytope {s | C*s<=d_init} about its center `s_center` and
   search the scaling factor.
   @pre s_center is in the polytope {s | C*s<=d_init, s_lower<=s<=s_upper}
   */
  [[nodiscard]] std::optional<SearchResult> BinarySearch(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::VectorXd>& d_init,
      const Eigen::Ref<const Eigen::VectorXd>& s_center,
      const BinarySearchOptions& options) const;

  /**
   Constructs a program to search for the C-space polytope {s | C*s<=d,
   s_lower<=s<=s_upper} such that this polytope is collision free.
   This program treats C and d as decision variables, and searches for the
   separating planes between each pair of geometries.
   Note that this program doesn't contain any cost yet.
   @param certificates The return of
   FindSeparationCertificateGivenPolytope().
   @param search_s_bounds_lagrangians Set to true if we search for the
   Lagrangian multiplier for the bounds s_lower <=s<=s_upper.
   @param[out] C The C-space polytope is parameterized as {s | C*s<=d,
   s_lower<=s<=s_upper}.
   @param[out] d The C-space polytope is parameterized as {s | C*s<=d,
   s_lower<=s<=s_upper}.
   @param[out] new_certificates The new certificates to certify the new C-space
   polytope {s | C*s<=d, s_lower<=s<=s_upper} is collision free. If
   new_certificates=nullptr, then we don't update it. This is used for testing.
   */
  [[nodiscard]] std::unique_ptr<solvers::MathematicalProgram>
  InitializePolytopeSearchProgram(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const std::unordered_map<SortedPair<geometry::GeometryId>,
                               SeparationCertificateResult>& certificates,
      bool search_s_bounds_lagrangians, MatrixX<symbolic::Variable>* C,
      VectorX<symbolic::Variable>* d,
      std::unordered_map<int, SeparationCertificate>* new_certificates =
          nullptr) const;

  /**
   Constructs the MathematicalProgram which searches for a separation
   certificate for a pair of geometries for a C-space polytope.Search for the
   separation certificate for a pair of geometries for a C-space polytope
   {s | C*s<=d, s_lower<=s<=s_upper}.
   */
  [[nodiscard]] SeparationCertificateProgram MakeIsGeometrySeparableProgram(
      const SortedPair<geometry::GeometryId>& geometry_pair,
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::VectorXd>& d) const;

  /**
   Solves a SeparationCertificateProgram with the given options
   @return result If we find the separation certificate, then `result` contains
   the separation plane and the Lagrangian polynomials; otherwise result is
   empty.
   */
  [[nodiscard]] std::optional<SeparationCertificateResult>
  SolveSeparationCertificateProgram(
      const SeparationCertificateProgram& certificate_program,
      const FindSeparationCertificateGivenPolytopeOptions& options) const;

 private:
  // Forward declaration the tester class. This tester class will expose the
  // private members of CspaceFreePolytope for unit test.
  friend class CspaceFreePolytopeTester;
  // Find the redundant inequalities in C*s <= d, s_lower <= s <= s_upper
  void FindRedundantInequalities(
      const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
      const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
      double tighten, std::unordered_set<int>* C_redundant_indices,
      std::unordered_set<int>* s_lower_redundant_indices,
      std::unordered_set<int>* s_upper_redundant_indices) const;

  // Computes s-s_lower and s_upper - s as polynomials of s.
  void CalcSBoundsPolynomial();

  // Computes d - C*s as a vector of polynomials on indeterminate s.
  template <typename T>
  [[nodiscard]] VectorX<symbolic::Polynomial> CalcDminusCs(
      const Eigen::Ref<const MatrixX<T>>& C,
      const Eigen::Ref<const VectorX<T>>& d) const;

  /**
   Generate all the conditions (certain rationals being non-negative, and
   certain vectors with length <= 1) such that the robot configuration is
   collision free.
   */
  void GenerateRationals();

  /**
   Computes the monomial basis for each pair of bodies.

   There can be multiple collision geometries on the same body, and their SOS
   problem will all share the same monomial basis. Hence we can first compute
   the monomial basis for each body, and reuse the result for all the collision
   geometries on the same body pair.
   */
  void CalcMonomialBasis();

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
  [[nodiscard]] SeparationCertificateProgram ConstructPlaneSearchProgram(
      const PlaneSeparatesGeometries& plane_geometries,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::unordered_set<int>& C_redundant_indices,
      const std::unordered_set<int>& s_lower_redundant_indices,
      const std::unordered_set<int>& s_upper_redundant_indices) const;

  /**
   For each pair of geometries, find the certificate that the pair is collision
   free in the C-space region {s | C*s<=d, s_lower<=s<=s_upper}.

   @retval certificates certificates[i] is the separation certificate for
   a pair of geometries. If we cannot certify or haven't certified
   the separation for this pair, then certificates[i] contains std::nullopt.
   Note that when we run this function in parallel and
   options.terminate_at_failure=true, we will terminate all the remaining
   certification programs that have been launched, so certificates[i] =
   std::nullopt could be either because that we have attempted to find the
   certificate for this pair of geometry but failed, or it could be that we fail
   to find the certificate for another pair and haven't attempted to find the
   certificate for this pair.
   */
  [[nodiscard]] std::vector<std::optional<SeparationCertificateResult>>
  FindSeparationCertificateGivenPolytope(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::VectorXd>& d,
      const FindSeparationCertificateGivenPolytopeOptions& options) const;

  /** When we fix the Lagrangian multipliers and search for the C-space polytope
  {s | C*s<=d, s_lower<=s<=s_upper}, we count the total size of all Gram
  matrices in the SOS program.
  */
  [[nodiscard]] int GetGramVarSizeForPolytopeSearchProgram(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      bool search_s_bounds_lagrangians) const;

  /**
   Constructs a program to search for the C-space polytope {s | C*s<=d,
   s_lower<=s<=s_upper} such that this polytope is collision free.
   This program takes C and d as decision variables, and searches for the
   separating planes between each pair of geometries.
   Note that this program doesn't contain any cost yet.
   @param certificates_vec The return of
   FindSeparationCertificateGivenPolytope().
   @param search_s_bounds_lagrangians Set to true if we search for the
   Lagrangian multiplier for the bounds s_lower <=s<=s_upper.
   @param gram_total_size The return of
   GetGramVarSizeForPolytopeSearchProgram().
   @param[out] new_certificates The new certificates to certify the new C-space
   polytope {s | C*s<=d, s_lower<=s<=s_upper} is collision free. If
   new_certificates=nullptr, then we don't update it. This is used for testing.
   */
  [[nodiscard]] std::unique_ptr<solvers::MathematicalProgram>
  InitializePolytopeSearchProgram(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::vector<std::optional<SeparationCertificateResult>>&
          certificates_vec,
      bool search_s_bounds_lagrangians, int gram_total_size,
      std::unordered_map<int, SeparationCertificate>* new_certificates =
          nullptr) const;

  /** Adds the constraint that the ellipsoid {Q*u+s₀ | uᵀu≤1} is inside the
     polytope {s | C*s <= d} with margin δ. Namely for the i'th face cᵢᵀs≤dᵢ, we
     have |cᵢᵀQ|₂ ≤ dᵢ − cᵢᵀs₀ − δᵢ and |cᵢ|₂≤1
     Note that this function does NOT add the constraint δ>=0
   */
  void AddEllipsoidContainmentConstraint(
      solvers::MathematicalProgram* prog, const Eigen::MatrixXd& Q,
      const Eigen::VectorXd& s0, const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Variable>& ellipsoid_margins) const;

  struct FindPolytopeGivenLagrangianResult {
    Eigen::MatrixXd C;
    Eigen::MatrixXd d;
    // a[i].dot(x) + b[i] = 0 is the separation plane for separating_planes_[i].
    std::unordered_map<int, Vector3<symbolic::Polynomial>> a;
    std::unordered_map<int, symbolic::Polynomial> b;
    Eigen::VectorXd ellipsoid_margins;
  };

  /**
   @param[out] certificates_result If certificates_result=nullptr, then we don't
   update its value; otherwise we set it to map the plane index to the
   separation certificates result for the plane.
   */
  [[nodiscard]] std::optional<FindPolytopeGivenLagrangianResult>
  FindPolytopeGivenLagrangian(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      const MatrixX<symbolic::Variable>& C,
      const VectorX<symbolic::Variable>& d,
      const VectorX<symbolic::Polynomial>& d_minus_Cs,
      const std::vector<std::optional<SeparationCertificateResult>>&
          certificates_vec,
      const Eigen::MatrixXd& Q, const Eigen::VectorXd& s0,
      const VectorX<symbolic::Variable>& ellipsoid_margins, int gram_total_size,
      const FindPolytopeGivenLagrangianOptions& options,
      std::unordered_map<int, SeparationCertificateResult>* certificates_result)
      const;

  /** Gets the H-polyhedron {s | C*s<=d, s_lower<=s<=s_upper}. */
  HPolyhedron GetPolyhedronWithJointLimits(const Eigen::MatrixXd& C,
                                           const Eigen::VectorXd& d) const;

  multibody::RationalForwardKinematics rational_forward_kin_;
  const geometry::SceneGraph<double>& scene_graph_;
  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
      link_geometries_;

  SeparatingPlaneOrder plane_order_;
  std::vector<CIrisSeparatingPlane<symbolic::Variable>> separating_planes_;
  std::unordered_map<SortedPair<geometry::GeometryId>, int>
      map_geometries_to_separating_planes_;

  // Sometimes we need to impose that a certain matrix of polynomials are always
  // psd (for example with sphere or capsule collision geometries). We will use
  // this slack variable to help us impose the matrix-sos constraint.
  Vector3<symbolic::Variable> y_slack_;

  symbolic::Variables s_set_;

  Eigen::VectorXd q_star_;
  Eigen::VectorXd s_lower_;
  Eigen::VectorXd s_upper_;
  VectorX<symbolic::Polynomial> s_minus_s_lower_;
  VectorX<symbolic::Polynomial> s_upper_minus_s_;
  // We have the invariant plane_geometries_[i].plane_index == i.
  std::vector<PlaneSeparatesGeometries> plane_geometries_;

  // Maps a pair of body (body1, body2) to an array of monomial basis
  // `monomial_basis_array`. monomial_basis_array[0] contains all the monomials
  // of form ∏ᵢ pow(sᵢ, dᵢ), dᵢ=0 or 1, sᵢ correspond to the revolute/prismatic
  // joint on the kinematic chain between body1 and body2.
  // monomial_basis_array[i+1] = y_slack_[i] * monomial_basis_array[0]
  std::unordered_map<SortedPair<multibody::BodyIndex>,
                     std::array<VectorX<symbolic::Monomial>, 4>>
      map_body_to_monomial_basis_array_;

  // See Options::with_cross_y for its meaning.
  bool with_cross_y_;
};

/**
 * Given a diagram (which contains the plant and the scene_graph), returns all
 * the collision geometries.
 */
[[nodiscard]] std::map<multibody::BodyIndex,
                       std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph);
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
