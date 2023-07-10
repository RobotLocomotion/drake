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

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/optimization/c_iris_collision_geometry.h"
#include "drake/geometry/optimization/c_iris_separating_plane.h"
#include "drake/geometry/optimization/cspace_free_polytope_base.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 This class tries to find large convex polytopes in the tangential-configuration
 space, such that all configurations in the convex polytopes is collision free.
 By tangential-configuration space, we mean the revolute joint angle θ is
 replaced by t = tan(θ/2).
 For more details, refer to the paper

 Certified Polyhedral Decomposition of Collisoin-Free Configuration Space
 by Hongkai Dai*, Alexandre Amice*, Peter Werner, Annan Zhang and Russ Tedrake.

 A conference version is published at

 Finding and Optimizing Certified, Colision-Free Regions in Configuration Space
 for Robot Manipulators
 by Alexandre Amice*, Hongkai Dai*, Peter Werner, Annan Zhang and Russ Tedrake.
 */
class CspaceFreePolytope : public CspaceFreePolytopeBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePolytope)

  using CspaceFreePolytopeBase::IgnoredCollisionPairs;

  ~CspaceFreePolytope() override = default;

  using CspaceFreePolytopeBase::Options;

  /**
   @param plant The plant for which we compute the C-space free polytopes. It
   must outlive this CspaceFreePolytope object.
   @param scene_graph The scene graph that has been connected with `plant`. It
   must outlive this CspaceFreePolytope object.
   @param plane_order The order of the polynomials in the plane to separate a
   pair of collision geometries.
   @param q_star Refer to RationalForwardKinematics for its meaning.

   @note CspaceFreePolytope knows nothing about contexts. The plant and
   scene_graph must be fully configured before instantiating this class.
   */
  CspaceFreePolytope(const multibody::MultibodyPlant<double>* plant,
                     const geometry::SceneGraph<double>* scene_graph,
                     SeparatingPlaneOrder plane_order,
                     const Eigen::Ref<const Eigen::VectorXd>& q_star,
                     const Options& options = Options{});

  /**
   When searching for the separating plane, we want to certify that the
   numerator of a rational is non-negative in the C-space region C*s<=d,
   s_lower <= s <= s_upper. Hence for each of the rational we will introduce
   Lagrangian multipliers for the polytopic constraint d-C*s >= 0, s - s_lower
   >= 0, s_upper - s >= 0.
   */
  class SeparatingPlaneLagrangians {
   public:
    SeparatingPlaneLagrangians(int C_rows, int s_size)
        : polytope_(C_rows), s_lower_(s_size), s_upper_(s_size) {}

    /** Substitutes the decision variables in each Lagrangians with its value in
     * result, returns the substitution result.
     */
    [[nodiscard]] SeparatingPlaneLagrangians GetSolution(
        const solvers::MathematicalProgramResult& result) const;

    /// The Lagrangians for d - C*s >= 0.
    const VectorX<symbolic::Polynomial>& polytope() const { return polytope_; }

    /// The Lagrangians for d - C*s >= 0.
    VectorX<symbolic::Polynomial>& mutable_polytope() { return polytope_; }

    /// The Lagrangians for s - s_lower >= 0.
    const VectorX<symbolic::Polynomial>& s_lower() const { return s_lower_; }

    /// The Lagrangians for s - s_lower >= 0.
    VectorX<symbolic::Polynomial>& mutable_s_lower() { return s_lower_; }

    /// The Lagrangians for s_upper - s >= 0.
    const VectorX<symbolic::Polynomial>& s_upper() const { return s_upper_; }

    /// The Lagrangians for s_upper - s >= 0.
    VectorX<symbolic::Polynomial>& mutable_s_upper() { return s_upper_; }

   private:
    // The Lagrangians for d - C*s >= 0.
    VectorX<symbolic::Polynomial> polytope_;
    // The Lagrangians for s - s_lower >= 0.
    VectorX<symbolic::Polynomial> s_lower_;
    // The Lagrangians for s_upper - s >= 0.
    VectorX<symbolic::Polynomial> s_upper_;
  };

  /**
   We certify that a pair of geometries is collision free in the C-space region
   {s | Cs<=d, s_lower<=s<=s_upper} by finding the separating plane and the
   Lagrangian multipliers. This struct contains the certificate, that the
   separating plane {x | aᵀx+b=0 } separates the two geometries in
   separating_planes()[plane_index] in the C-space polytope.
   */
  struct SeparationCertificateResult final : SeparationCertificateResultBase {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateResult)
    SeparationCertificateResult() {}
    ~SeparationCertificateResult() override = default;

    const std::vector<SeparatingPlaneLagrangians>& lagrangians(
        PlaneSide plane_side) const {
      return plane_side == PlaneSide::kPositive
                 ? positive_side_rational_lagrangians
                 : negative_side_rational_lagrangians;
    }

    std::vector<SeparatingPlaneLagrangians> positive_side_rational_lagrangians;
    std::vector<SeparatingPlaneLagrangians> negative_side_rational_lagrangians;
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

    std::vector<SeparatingPlaneLagrangians>& mutable_lagrangians(
        PlaneSide plane_side) {
      return plane_side == PlaneSide::kPositive
                 ? positive_side_rational_lagrangians
                 : negative_side_rational_lagrangians;
    }

    // positive_side_rational_lagrangians[i] is the Lagrangian multipliers for
    // PlaneSeparatesGeometries::positive_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> positive_side_rational_lagrangians;
    // negative_side_rational_lagrangians[i] is the Lagrangian multipliers for
    // PlaneSeparatesGeometries::negative_side_rationals[i].
    std::vector<SeparatingPlaneLagrangians> negative_side_rational_lagrangians;
  };

  struct SeparationCertificateProgram final : SeparationCertificateProgramBase {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SeparationCertificateProgram)
    SeparationCertificateProgram() = default;
    virtual ~SeparationCertificateProgram() = default;

    SeparationCertificate certificate;
  };

  struct FindSeparationCertificateGivenPolytopeOptions final
      : FindSeparationCertificateOptions {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
        FindSeparationCertificateGivenPolytopeOptions)
    FindSeparationCertificateGivenPolytopeOptions()  = default;
    ~FindSeparationCertificateGivenPolytopeOptions() override = default;
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
    /** Maximize ∑ᵢδᵢ */
    kSum,
    /** Maximize the geometric mean power(∏ᵢ (δᵢ + ε), 1/n) where n is C.rows(),
    ε=FindPolytopeGivenLagrangianOptions.ellipsoid_margin_epsilon. */
    kGeometricMean,
  };

  /**
   Options for finding polytope with given Lagrangians.
   */
  struct FindPolytopeGivenLagrangianOptions {
    std::optional<double> backoff_scale{std::nullopt};

    /** We will maximize the cost ∏ᵢ (δᵢ + ε) where δᵢ is the margin from each
     face of the polytope {s | Cs<=d} to the inscribed ellipsoid, ε is
     ellipsoid_margin_epsilon, a small positive constant to make sure δᵢ + ε
     being strictly positive.
     */
    double ellipsoid_margin_epsilon{1E-5};

    /** ID for the solver */
    solvers::SolverId solver_id{solvers::MosekSolver::id()};

    /** options for solving the MathematicalProgram */
    std::optional<solvers::SolverOptions> solver_options{std::nullopt};

    /** We can constrain the C-space polytope {s | C*s<=d, s_lower<=s<=s_upper}
     to contain some sampled s. Each column of s_inner_pts is a sample of s.
     */
    std::optional<Eigen::MatrixXd> s_inner_pts;

    /** If set to true, then we will also search for the Lagrangian multipliers
     for the constraint s_lower <= s <= s_upper; otherwise we fix the
     Lagrangian multiplier to the solution found when we fix the C-space
     polytope {s | C*s<=d, s_lower<=s<=s_upper}.
     */
    bool search_s_bounds_lagrangians{true};

    /** Type of cost on the ellipsoid margin */
    EllipsoidMarginCost ellipsoid_margin_cost{
        EllipsoidMarginCost::kGeometricMean};
  };

  /** Result on searching the C-space polytope and separating planes. */
  class SearchResult {
   public:
    SearchResult() {}

    [[nodiscard]] const Eigen::MatrixXd& C() const { return C_; }

    [[nodiscard]] const Eigen::VectorXd& d() const { return d_; }

    [[nodiscard]] const HPolyhedron& certified_polytope() const {
      return certified_polytope_;
    }

    /** Each plane index is mapped to a vector of polynomials. */
    [[nodiscard]] const std::unordered_map<int, Vector3<symbolic::Polynomial>>&
    a() const {
      return a_;
    }

    /** Each plane index is mapped to a vector of polynomial, */
    [[nodiscard]] const std::unordered_map<int, symbolic::Polynomial>& b()
        const {
      return b_;
    }

    /** The number of iterations taken to search for the result. */
    [[nodiscard]] int num_iter() const { return num_iter_; }

   private:
    friend class CspaceFreePolytope;
    void SetPolytope(const Eigen::Ref<const Eigen::MatrixXd>& C,
                     const Eigen::Ref<const Eigen::VectorXd>& d,
                     const CspaceFreePolytope& cspace_free_polytope);

    void SetSeparatingPlanes(
        std::unordered_map<int, Vector3<symbolic::Polynomial>> a,
        std::unordered_map<int, symbolic::Polynomial> b);

    // Clear this->a and this->b and reset their values.
    // Each entry in certificates_result should have a value (cannot be
    // nullopt).
    void SetSeparatingPlanes(
        const std::vector<std::optional<SeparationCertificateResult>>&
            certificates_result);

    // Update this->a and this->b with the values in certificates_result.
    void UpdateSeparatingPlanes(
        const std::vector<std::optional<SeparationCertificateResult>>&
            certificates_results);

    Eigen::MatrixXd C_;
    Eigen::VectorXd d_;
    // This is the certified C-space polytope {s | C * s <= d, s_lower <= s <=
    // s_upper}.
    HPolyhedron certified_polytope_;
    // a[i].dot(x) + b[i]=0 is the separation plane for separating_planes()[i].
    std::unordered_map<int, Vector3<symbolic::Polynomial>> a_;
    std::unordered_map<int, symbolic::Polynomial> b_;
    // The number of iterations at termination.
    int num_iter_{};
  };

  /** Options for bilinear alternation. */
  struct BilinearAlternationOptions {
    /** The maximum number of bilinear alternation iterations. Must be
     non-negative.
     */
    int max_iter{10};
    /** When the change of the cost function between two consecutive iterations
     in bilinear alternation is no larger than this number, stop the bilinear
     alternation. Must be non-negative.
     */
    double convergence_tol{1E-3};
    FindPolytopeGivenLagrangianOptions find_polytope_options;
    FindSeparationCertificateGivenPolytopeOptions find_lagrangian_options;
    /** After finding the maximal inscribed ellipsoid in C-space polytope {s |
     C*s<=d, s_lower<=s<=s_upper}, we scale this ellipsoid by
     ellipsoid_scaling, and require the new C-space polytope to contain this
     scaled ellipsoid. ellipsoid_scaling=1 corresponds to no scaling.
     Must be strictly positive and no greater than 1.
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

  /** Options for binary search. */
  struct BinarySearchOptions {
    /** The maximal value of the scaling factor.
     Must be finite and no less than scale_min. */
    double scale_max{1};
    /** The minimal value of the scaling factor.
     Must be non-negative. */
    double scale_min{0.01};
    /** The maximal number of iterations in binary search.
     Must be non-negative. */
    int max_iter{10};
    /** When the gap between the upper bound and the lower bound of the scaling
     factor is below this `convergence_tol`, stops the binary search.
     Must be strictly positive.
     */
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
   certificate for a pair of geometries for a C-space polytope. Search for the
   separation certificate for a pair of geometries for a C-space polytope
   {s | C*s<=d, s_lower<=s<=s_upper}.
   @throws an error if this `geometry_pair` doesn't need separation certificate
   (for example, they are on the same body).
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

  [[nodiscard]] std::vector<PlaneSeparatesGeometries>&
  get_mutable_plane_geometries() {
    return plane_geometries_;
  }

  // Find the redundant inequalities in C*s <= d, s_lower <= s <= s_upper
  void FindRedundantInequalities(
      const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
      const Eigen::VectorXd& s_lower, const Eigen::VectorXd& s_upper,
      double tighten, std::unordered_set<int>* C_redundant_indices,
      std::unordered_set<int>* s_lower_redundant_indices,
      std::unordered_set<int>* s_upper_redundant_indices) const;

  // Computes d - C*s as a vector of polynomials on indeterminate s.
  template <typename T>
  [[nodiscard]] VectorX<symbolic::Polynomial> CalcDminusCs(
      const Eigen::Ref<const MatrixX<T>>& C,
      const Eigen::Ref<const VectorX<T>>& d) const;

  /*
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

  /*
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

  /* When we fix the Lagrangian multipliers and search for the C-space polytope
  {s | C*s<=d, s_lower<=s<=s_upper}, we count the total size of all Gram
  matrices in the SOS program.
  */
  [[nodiscard]] int GetGramVarSizeForPolytopeSearchProgram(
      const IgnoredCollisionPairs& ignored_collision_pairs,
      bool search_s_bounds_lagrangians) const;

  /*
   Overloads InitializePolytopeSearchProgram, but with C, d, d_minus_Cs,
   certificates_vec as the input arguments. This function will be called
   repeatedly (for example in bilinear alternation) with the same arguments C,
   d, d_minus_Cs; hence it is better to construct these arguments beforehand and
   call this overloaded function.
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

  /* Adds the constraint that the ellipsoid {Q*u+s₀ | uᵀu≤1} is inside the
     polytope {s | C*s <= d} with margin δ. Namely for the i'th face cᵢᵀs≤dᵢ, we
     have |cᵢᵀQ|₂ ≤ dᵢ − cᵢᵀs₀ − δᵢ and |cᵢ|₂≤1
     cᵢᵀ is the i'th row of C, dᵢ is the i'th entry of d, δᵢ is the i'th entry
     of ellipsoid_margins.
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

  /*
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

  /* Gets the H-polyhedron {s | C*s<=d, s_lower<=s<=s_upper}. */
  HPolyhedron GetPolyhedronWithJointLimits(const Eigen::MatrixXd& C,
                                           const Eigen::VectorXd& d) const;

  Eigen::VectorXd q_star_;
  Eigen::VectorXd s_lower_;
  Eigen::VectorXd s_upper_;
  VectorX<symbolic::Polynomial> s_minus_s_lower_;
  VectorX<symbolic::Polynomial> s_upper_minus_s_;
  // We have the invariant plane_geometries_[i].plane_index == i.
  std::vector<PlaneSeparatesGeometries> plane_geometries_;
};

DRAKE_DEPRECATED("2023-09-01", "This function was not intended for public use.")
std::map<multibody::BodyIndex,
         std::vector<std::unique_ptr<CIrisCollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
