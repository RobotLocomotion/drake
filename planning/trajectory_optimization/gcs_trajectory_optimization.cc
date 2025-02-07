#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"

#include <algorithm>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/pointer_cast.h"
#include "drake/common/scope_exit.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/common/trajectories/piecewise_trajectory.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/geodesic_convexity.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/geometry/optimization/point.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/matrix_util.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

using Subgraph = GcsTrajectoryOptimization::Subgraph;
using EdgesBetweenSubgraphs = GcsTrajectoryOptimization::EdgesBetweenSubgraphs;

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::VectorXd;
using geometry::optimization::CartesianProduct;
using geometry::optimization::CheckIfSatisfiesConvexityRadius;
using geometry::optimization::ComputePairwiseIntersections;
using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Intersection;
using geometry::optimization::PartitionConvexSet;
using geometry::optimization::Point;
using geometry::optimization::internal::ComputeOffsetContinuousRevoluteJoints;
using geometry::optimization::internal::GetMinimumAndMaximumValueAlongDimension;
using geometry::optimization::internal::ThrowsForInvalidContinuousJointsList;
using math::ExtractValue;
using math::InitializeAutoDiff;
using multibody::BallRpyJoint;
using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;
using multibody::PlanarJoint;
using multibody::RevoluteJoint;
using multibody::RpyFloatingJoint;
using multibody::UniversalJoint;
using solvers::Binding;
using solvers::ConcatenateVariableRefList;
using solvers::Constraint;
using solvers::Cost;
using solvers::L2NormCost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::MathematicalProgram;
using solvers::MatrixXDecisionVariable;
using solvers::QuadraticCost;
using solvers::Solve;
using solvers::VectorXDecisionVariable;
using symbolic::DecomposeLinearExpressions;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::MakeMatrixContinuousVariable;
using symbolic::MakeVectorContinuousVariable;
using symbolic::Variable;
using trajectories::BezierCurve;
using trajectories::CompositeTrajectory;
using trajectories::PiecewiseTrajectory;
using trajectories::Trajectory;
using Vertex = GraphOfConvexSets::Vertex;
using Edge = GraphOfConvexSets::Edge;
using VertexId = GraphOfConvexSets::VertexId;
using EdgeId = GraphOfConvexSets::EdgeId;
using Transcription = GraphOfConvexSets::Transcription;

const double kInf = std::numeric_limits<double>::infinity();

/* Implements a constraint of the form
  0 <= dᴺr(s) / dsᴺ - hᴺ * lb <= inf,
  0 <= - dᴺr(s) / dsᴺ + hᴺ * ub <= inf,
  where
  N := derivative order,
  h = x[num_control_points],
  dᴺr(s) / dsᴺ = M * x[0:num_control_points].

  This constraint is enforced along one dimension of the Bézier curve, hence
  must be called for each dimension separately.
*/
class NonlinearDerivativeConstraint : public solvers::Constraint {
 public:
  NonlinearDerivativeConstraint(const Eigen::SparseMatrix<double>& M, double lb,
                                double ub, int derivative_order)
      : Constraint(2 * M.rows(), M.cols() + 1,
                   Eigen::VectorXd::Zero(2 * M.rows()),
                   Eigen::VectorXd::Constant(
                       2 * M.rows(), std::numeric_limits<double>::infinity())),
        M_(M),
        lb_(lb),
        ub_(ub),
        derivative_order_(derivative_order),
        num_control_points_(M.cols()) {
    DRAKE_DEMAND(derivative_order > 1);
  }

  template <typename T, typename U>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<U>>& x,
                     VectorX<T>* y) const {
    // x is the stack [r_control.row(i); h].
    using std::pow;
    T pow_h = pow(x[num_control_points_], derivative_order_);

    // Precompute Matrix Product.
    VectorX<T> Mx = M_ * x.head(num_control_points_);

    y->head(M_.rows()) = Mx.array() - pow_h * lb_;
    y->tail(M_.rows()) = -Mx.array() + pow_h * ub_;
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const {
    DoEvalGeneric(x, y);
  }

 private:
  const Eigen::SparseMatrix<double> M_;
  const double lb_;
  const double ub_;
  const int derivative_order_;
  const int num_control_points_;
};

/* Implements a constraint of the form
  (dᴺrᵤ(s=1) / dsᴺ) * hᵥᴺ == (dᴺrᵥ(s=0) / dsᴺ) * hᵤᴺ
  where
  N := derivative order,
  hᵤ = x[num_u_control_points],
  hᵥ = x[num_u_control_points + num_v_control_points + 1],
  dᴺrᵤ(s) / dsᴺ = Mu * x[0:num_u_control_points].
  dᴺrᵥ(s) / dsᴺ = Mv * x[num_u_control_points + 1: -1].

  This constraint is enforced along one dimension of the Bézier curve, hence
  must be called for each dimension separately.
*/
class NonlinearContinuityConstraint : public solvers::Constraint {
 public:
  NonlinearContinuityConstraint(const Eigen::SparseMatrix<double>& Mu,
                                const Eigen::SparseMatrix<double>& Mv,
                                int continuity_order)
      : Constraint(1, Mu.cols() + 1 + Mv.cols() + 1, Eigen::VectorXd::Zero(1),
                   Eigen::VectorXd::Zero(1)),
        Mu_(Mu),
        Mv_(Mv),
        continuity_order_(continuity_order),
        num_control_points_u_(Mu.cols()),
        num_control_points_v_(Mv.cols()) {
    DRAKE_DEMAND(Mu.rows() == 1);
    DRAKE_DEMAND(Mv.rows() == 1);
    DRAKE_DEMAND(continuity_order >= 1);
  }

  template <typename T, typename U>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<U>>& x,
                     VectorX<T>* y) const {
    // x is the stack [r_control_u.row(i); h_u; r_control_v.row(i); h_v;].

    using std::pow;
    T pow_h_u = pow(x[num_control_points_u_], continuity_order_);
    T pow_h_v = pow(x[num_control_points_u_ + num_control_points_v_ + 1],
                    continuity_order_);

    // Precompute Matrix Products.
    T Mu_x = Mu_.row(0) * x.head(num_control_points_u_);
    T Mv_x = Mv_.row(0) *
             x.segment(num_control_points_u_ + 1, num_control_points_v_);

    (*y)[0] = Mu_x * pow_h_v - Mv_x * pow_h_u;
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const {
    DoEvalGeneric(x, y);
  }

 private:
  const Eigen::SparseMatrix<double> Mu_;
  const Eigen::SparseMatrix<double> Mv_;
  const int continuity_order_;
  const int num_control_points_u_;
  const int num_control_points_v_;
};

Subgraph::Subgraph(
    const ConvexSets& regions,
    const std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double h_min, double h_max, std::string name,
    const std::vector<VectorXd>* edge_offsets,
    GcsTrajectoryOptimization* traj_opt)
    : regions_(regions),
      order_(order),
      h_min_(h_min),
      name_(std::move(name)),
      traj_opt_(*traj_opt) {
  DRAKE_THROW_UNLESS(order >= 0);
  DRAKE_THROW_UNLESS(!regions_.empty());

  // This will hold the edge offsets if they weren't given as an argument.
  std::optional<std::vector<VectorXd>> maybe_edge_offsets;

  if (edge_offsets) {
    DRAKE_THROW_UNLESS(edge_offsets->size() == edges_between_regions.size());
  } else {
    maybe_edge_offsets = std::vector<VectorXd>{};
    std::vector<std::vector<std::pair<double, double>>> continuous_bboxes;
    continuous_bboxes.reserve(ssize(regions));
    for (const auto& region_ptr : regions) {
      continuous_bboxes.push_back(GetMinimumAndMaximumValueAlongDimension(
          *region_ptr, continuous_revolute_joints()));
    }
    maybe_edge_offsets->reserve(edges_between_regions.size());
    for (const auto& [i, j] : edges_between_regions) {
      maybe_edge_offsets->push_back(ComputeOffsetContinuousRevoluteJoints(
          num_positions(), continuous_revolute_joints(),
          continuous_bboxes.at(i), continuous_bboxes.at(j)));
    }
    edge_offsets = &(maybe_edge_offsets.value());
  }

  // Make sure all regions have the same ambient dimension.
  for (const std::unique_ptr<ConvexSet>& region : regions_) {
    DRAKE_THROW_UNLESS(region != nullptr);
    DRAKE_THROW_UNLESS(region->ambient_dimension() == num_positions());
  }

  // If there are any continuous revolute joints, make sure the convexity radius
  // is respected.
  if (continuous_revolute_joints().size() > 0) {
    ThrowsForInvalidConvexityRadius();
  }

  // Make time scaling set once to avoid many allocations when adding the
  // vertices to GCS.
  const HPolyhedron time_scaling_set =
      HPolyhedron::MakeBox(Vector1d(h_min), Vector1d(h_max));

  // Add Regions with time scaling set.
  for (int i = 0; i < ssize(regions_); ++i) {
    ConvexSets vertex_set;
    // Assign each control point to a separate set.
    const int num_points = order + 1;
    vertex_set.reserve(num_points + 1);
    vertex_set.insert(vertex_set.begin(), num_points,
                      ConvexSets::value_type{*regions_[i]});
    // Add time scaling set.
    vertex_set.emplace_back(time_scaling_set);

    vertices_.emplace_back(traj_opt_.gcs_.AddVertex(
        CartesianProduct(vertex_set), fmt::format("{}: Region{}", name_, i)));
    traj_opt->vertex_to_subgraph_[vertices_.back()] = this;
  }

  r_trajectory_ =
      BezierCurve<double>(0, 1, MatrixXd::Zero(num_positions(), order + 1));

  // GetControlPoints(u).col(order) - GetControlPoints(v).col(0) = 0, via Ax =
  // 0, A = [I, -I], x = [u_controls.col(order); v_controls.col(0)].
  Eigen::SparseMatrix<double> A(num_positions(), 2 * num_positions());
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(2 * num_positions());
  for (int i = 0; i < num_positions(); ++i) {
    tripletList.push_back(Eigen::Triplet<double>(i, i, 1.0));
    tripletList.push_back(Eigen::Triplet<double>(i, num_positions() + i, -1.0));
  }
  A.setFromTriplets(tripletList.begin(), tripletList.end());
  for (int idx = 0; idx < ssize(edges_between_regions); ++idx) {
    // Add edge.
    Vertex* u = vertices_.at(edges_between_regions[idx].first);
    Vertex* v = vertices_.at(edges_between_regions[idx].second);
    Edge* uv_edge = traj_opt_.AddEdge(u, v);

    edges_.emplace_back(uv_edge);

    // Add path continuity constraints. To handle edge offsets, we enforce the
    // constraint GetControlPoints(u).col(order) - GetControlPoints(v).col(0) =
    // -tau_uv.value(), via Ax = -edge_offsets->at(idx), A = [I, -I],
    // x = [u_controls.col(order); v_controls.col(0)].
    const auto path_continuity_constraint =
        std::make_shared<LinearEqualityConstraint>(A, -edge_offsets->at(idx));
    uv_edge->AddConstraint(Binding<Constraint>(
        path_continuity_constraint,
        {GetControlPoints(*u).col(order), GetControlPoints(*v).col(0)}));
  }

  // Construct placeholder variables.
  placeholder_vertex_duration_var_ = symbolic::Variable("t");
  placeholder_vertex_control_points_var_ =
      MakeMatrixContinuousVariable(num_positions(), order + 1, "x");
  placeholder_edge_durations_var_ =
      std::make_pair(symbolic::Variable("tu"), symbolic::Variable("tv"));
  placeholder_edge_control_points_var_ = std::make_pair(
      MakeMatrixContinuousVariable(num_positions(), order + 1, "xu"),
      MakeMatrixContinuousVariable(num_positions(), order + 1, "xv"));
}

Subgraph::~Subgraph() = default;

void Subgraph::ThrowsForInvalidConvexityRadius() const {
  for (int i = 0; i < ssize(regions_); ++i) {
    for (const int& j : continuous_revolute_joints()) {
      auto [min_value, max_value] =
          GetMinimumAndMaximumValueAlongDimension(*regions_[i], j);
      if (max_value - min_value >= M_PI) {
        throw std::runtime_error(fmt::format(
            "GcsTrajectoryOptimization: Region at index {} is wider than π "
            "along dimension {}, so it doesn't respect the convexity radius! "
            "To add this set, separate it into smaller pieces so that along "
            "dimensions corresponding to continuous revolute joints, its width "
            "is strictly smaller than π. This can be done manually, or with "
            "the helper function PartitionConvexSet.",
            i, j));
      }
    }
  }
}

std::vector<const GraphOfConvexSets::Vertex*> Subgraph::Vertices() const {
  std::vector<const GraphOfConvexSets::Vertex*> vertices;
  vertices.reserve(vertices_.size());
  for (const auto& v : vertices_) {
    vertices.push_back(v);
  }
  return vertices;
}

std::vector<const GraphOfConvexSets::Edge*> Subgraph::Edges() const {
  std::vector<const GraphOfConvexSets::Edge*> edges;
  edges.reserve(edges_.size());
  for (const auto& e : edges_) {
    edges.push_back(e);
  }
  return edges;
}

void Subgraph::AddTimeCost(double weight) {
  // The time cost is the sum of duration variables ∑ hᵢ
  auto time_cost =
      std::make_shared<LinearCost>(weight * Eigen::VectorXd::Ones(1), 0.0);

  for (Vertex* v : vertices_) {
    // The duration variable is the last element of the vertex.
    v->AddCost(Binding<LinearCost>(time_cost, v->x().tail(1)));
  }
}

void Subgraph::AddPathLengthCost(const MatrixXd& weight_matrix) {
  /*
    We will upper bound the trajectory length by the sum of the distances
    between the control points. ∑ |weight_matrix * (rᵢ₊₁ − rᵢ)|₂
  */
  DRAKE_THROW_UNLESS(weight_matrix.rows() == num_positions());
  DRAKE_THROW_UNLESS(weight_matrix.cols() == num_positions());

  if (order() == 0) {
    throw std::runtime_error(
        "Path length cost is not defined for a set of order 0.");
  }

  MatrixXd A(num_positions(), 2 * num_positions());
  A << weight_matrix, -weight_matrix;
  const auto path_length_cost =
      std::make_shared<L2NormCost>(A, VectorXd::Zero(num_positions()));

  for (Vertex* v : vertices_) {
    auto control_points = GetControlPoints(*v);
    for (int i = 0; i < control_points.cols() - 1; ++i) {
      v->AddCost(Binding<L2NormCost>(
          path_length_cost,
          {control_points.col(i + 1), control_points.col(i)}));
    }
  }
}

void Subgraph::AddPathEnergyCost(const MatrixXd& weight_matrix) {
  // Add a cost to the control points of the trajectory that is of the form ∑
  // (rᵢ₊₁ * weight_matrix * rᵢ)

  Eigen::MatrixXd b = Eigen::VectorXd::Zero(2 * num_positions());

  MatrixXd Q(2 * num_positions(), 2 * num_positions());

  Q.block(0, 0, num_positions(), num_positions()) << weight_matrix;
  Q.block(num_positions(), num_positions(), num_positions(), num_positions())
      << weight_matrix;
  Q.block(0, num_positions(), num_positions(), num_positions())
      << -weight_matrix;
  Q.block(num_positions(), 0, num_positions(), num_positions())
      << -weight_matrix;

  const auto path_squared_cost = std::make_shared<QuadraticCost>(Q, b);

  for (Vertex* v : vertices_) {
    auto control_points = GetControlPoints(*v);
    for (int i = 0; i < control_points.cols() - 1; ++i) {
      v->AddCost(Binding<QuadraticCost>(
          path_squared_cost,
          {control_points.col(i + 1), control_points.col(i)}));
    }
  }
}

void Subgraph::AddPathLengthCost(double weight) {
  const MatrixXd weight_matrix =
      weight * MatrixXd::Identity(num_positions(), num_positions());
  return Subgraph::AddPathLengthCost(weight_matrix);
}

void Subgraph::AddPathEnergyCost(double weight) {
  // Note that the scalar 2 is present to construct an appropriate default cost
  // of the form |(rᵢ₊₁ − rᵢ)|₂². The constructed quadratic cost is of the form
  // .5 * x'Qx, so Q must be 2 * I to counteract the 0.5 scalar in the quadratic
  // cost.
  const MatrixXd weight_matrix =
      weight * 2 * MatrixXd::Identity(num_positions(), num_positions());
  return Subgraph::AddPathEnergyCost(weight * weight_matrix);
}

void Subgraph::AddVelocityBounds(const Eigen::Ref<const VectorXd>& lb,
                                 const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());
  if (order() == 0) {
    throw std::runtime_error(
        "Velocity Bounds are not defined for a set of order 0.");
  }

  // We have q̇(t) = drds * dsdt = ṙ(s) / h, and h >= 0, so we
  // use h * lb <= ṙ(s) <= h * ub, formulated as:
  //     0 <= ṙ(s) - h * lb <= inf,
  // - inf <= ṙ(s) - h * ub <= 0.

  // This leverages the convex hull property of the B-splines: if all of the
  // control points satisfy these convex constraints and the curve is inside
  // the convex hull of these constraints, then the curve satisfies the
  // constraints for all t.

  // The relevant derivatives of the Bezier curve come in the form:
  // rdot_control.row(i).T = M.T * r_control.row(i).T, so we loop over the
  // positions, rather than over the control points.
  const VectorXd kVecInf = VectorXd::Constant(order_, kInf);
  const VectorXd kVecZero = VectorXd::Zero(order_);
  solvers::VectorXDecisionVariable vars(order_ + 2);
  SparseMatrix<double> H_lb(
      order_ /* number of control points for one row of ṙ(s)*/,
      order_ + 2 /* number of control points for one row of r(s) + 1*/);
  H_lb.leftCols(order_ + 1) =
      r_trajectory_.AsLinearInControlPoints(1).transpose();
  SparseMatrix<double> H_ub = H_lb;
  for (int i = 0; i < num_positions(); ++i) {
    // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
    H_lb.rightCols<1>() = VectorXd::Constant(order_, -lb[i]).sparseView();
    const auto lb_constraint =
        std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
    // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
    H_ub.rightCols<1>() = VectorXd::Constant(order_, -ub[i]).sparseView();
    const auto ub_constraint =
        std::make_shared<LinearConstraint>(H_ub, -kVecInf, kVecZero);
    for (Vertex* v : vertices_) {
      vars << GetControlPoints(*v).row(i).transpose(), GetTimeScaling(*v);
      v->AddConstraint(Binding<LinearConstraint>(lb_constraint, vars));
      v->AddConstraint(Binding<LinearConstraint>(ub_constraint, vars));
    }
  }
}

void Subgraph::AddNonlinearDerivativeBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, int derivative_order) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());

  if (derivative_order > order()) {
    throw std::runtime_error(
        "Derivative order must be less than or equal to the set order.");
  }

  if (derivative_order == 1) {
    throw std::runtime_error(
        "Use AddVelocityBounds instead of AddNonlinearDerivativeBounds with "
        "derivative_order=1; velocity bounds are linear.");
  }

  if (derivative_order < 1) {
    throw std::runtime_error("Derivative order must be greater than 1.");
  }

  // The nonlinear derivative dᴺq(t) / dtᴺ = dᴺr(s) / dsᴺ / hᴺ, with h >= 0,
  // can be written as hᴺ * lb <= dᴺr(s) / dsᴺ <= hᴺ * ub, and constrained as:
  // 0 <=   dᴺr(s) / dsᴺ - hᴺ * lb <= inf,
  // 0 <= - dᴺr(s) / dsᴺ + hᴺ * ub <= inf.

  // The nonlinear constraint will be enforced in the restriction and MIP
  // of GCS, while a convex surrogate in the relaxation transcription will
  // guide the rounding process.

  // Since hᴺ is the source of nonlinearities, we will replace it with:
  // h₀ᴺ⁻¹h
  // Then we will get the following linear constraint:
  // 0 <=   dᴺr(s) / dsᴺ - h₀ᴺ⁻¹h * lb <= inf
  // 0 <= - dᴺr(s) / dsᴺ + h₀ᴺ⁻¹h * ub <= inf.

  // For simplicity sake, we will set h₀ to one until we come up with a
  // reasonable heuristic, e.g. based on the maximum length of the convex set.
  const double h0 = 1.0;

  // This leverages the convex hull property of the B-splines: if all of the
  // control points satisfy these convex constraints and the curve is inside
  // the convex hull of these constraints, then the curve satisfies the
  // constraints for all t.

  // The relevant derivatives of the Bezier curve come in the form:
  // rdot_control.row(i).T = M.T * r_control.row(i).T, so we loop over the
  // positions, rather than over the control points.
  solvers::VectorXDecisionVariable vars(order_ + 2);
  SparseMatrix<double> M_transpose =
      r_trajectory_.AsLinearInControlPoints(derivative_order).transpose();

  // Lower bound: 0 <=   (dᴺr(s) / dsᴺ).row(i) - h₀ᴺ⁻¹h * lb[i] <= inf,
  // Upper bound: 0 <= - (dᴺr(s) / dsᴺ).row(i) + h₀ᴺ⁻¹h * ub[i] <= inf.

  int rdot_control_points = order_ + 1 - derivative_order;
  const VectorXd kVecInf = VectorXd::Constant(2 * rdot_control_points, kInf);
  const VectorXd kVecZero = VectorXd::Zero(2 * rdot_control_points);
  Eigen::MatrixXd H(
      2 * rdot_control_points,
      order_ + 2);  // number of control points for one row of r(s) + 1
  H.block(0, 0, M_transpose.rows(), M_transpose.cols()) = M_transpose;
  H.block(M_transpose.rows(), 0, M_transpose.rows(), M_transpose.cols()) =
      -M_transpose;

  for (int i = 0; i < num_positions(); ++i) {
    // Update the bounds for each position.
    H.block(0, order_ + 1, rdot_control_points, 1) = VectorXd::Constant(
        rdot_control_points, -std::pow(h0, derivative_order - 1) * lb[i]);
    H.block(rdot_control_points, order_ + 1, rdot_control_points, 1) =
        VectorXd::Constant(rdot_control_points,
                           std::pow(h0, derivative_order - 1) * ub[i]);

    const auto normalized_path_derivative_constraint =
        std::make_shared<LinearConstraint>(H.sparseView(), kVecZero, kVecInf);

    const auto nonlinear_derivative_constraint =
        std::make_shared<NonlinearDerivativeConstraint>(
            M_transpose, lb[i], ub[i], derivative_order);
    for (Vertex* v : vertices_) {
      vars << GetControlPoints(*v).row(i).transpose(), GetTimeScaling(*v);
      // Add convex surrogate.
      v->AddConstraint(Binding<LinearConstraint>(
                           normalized_path_derivative_constraint, vars),
                       {Transcription::kRelaxation});
      // Add nonlinear constraint.
      v->AddConstraint(Binding<NonlinearDerivativeConstraint>(
                           nonlinear_derivative_constraint, vars),
                       {Transcription::kMIP, Transcription::kRestriction});
    }
  }
}

void Subgraph::AddPathContinuityConstraints(int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }

  if (order_ < continuity_order) {
    throw std::runtime_error(
        "Cannot add continuity constraint of order greater than the set "
        "order.");
  }

  // The continuity on derivatives of r(s) will be enforced between the last
  // control point of the u set and the first control point of the v set in an
  // edge. urdot_control.col(order-continuity_order) - vrdot_control.col(0) = 0,
  // The latter can be achieved by getting the control point matrix M.
  // A = [M.col(order - continuity_order).T, -M.col(0).T],
  // x = [u_controls.row(i); v_controls.row(i)].
  // Ax = 0,

  SparseMatrix<double> Mu_transpose =
      r_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(order_ - continuity_order)
          .transpose();

  SparseMatrix<double> Mv_transpose =
      r_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(0)
          .transpose();

  // Concatenate Mu_transpose and Mv_transpose.
  // A = [Mu.T, - Mv.T]

  // The A matrix will have one row since sparsity allows us to enforce the
  // continuity in each dimension. The number of columns matches the number of
  // control points for one row of r_u(s) and r_v(s).
  SparseMatrix<double> A(1, 2 * (order_ + 1));
  A.leftCols(order_ + 1) = Mu_transpose;
  A.rightCols(order_ + 1) = -Mv_transpose;

  const auto continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(A, VectorXd::Zero(1));

  for (int i = 0; i < num_positions(); ++i) {
    for (Edge* edge : edges_) {
      // Add continuity constraints.
      edge->AddConstraint(Binding<LinearEqualityConstraint>(
          continuity_constraint, {GetControlPoints(edge->u()).row(i),
                                  GetControlPoints(edge->v()).row(i)}));
    }
  }
}

void Subgraph::AddContinuityConstraints(int continuity_order) {
  // TODO(cohnt): Rewrite to use the generic AddCost and AddConstraint
  // interfaces.

  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }

  if (continuity_order > order_) {
    throw std::runtime_error(
        "Cannot add continuity constraint of order greater than the set "
        "order.");
  }

  // The continuity on derivatives of q(t) will be enforced between the last
  // control point of the u set and the first control point of the v set in an
  // edge.

  // Since the derivative of q(t) appears nonlinear in h, the following
  // nonlinear constraint will be enforced on the MIP and the restriction:
  // (dᴺrᵤ(s=1) / dsᴺ) / hᵤᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥᴺ.

  // Which can be written as:
  // urdot_control.col(order-N) * hᵥᴺ - vrdot_control.col(0) * hᵤᴺ = 0.
  SparseMatrix<double> Mu_transpose =
      r_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(order_ - continuity_order)
          .transpose();

  SparseMatrix<double> Mv_transpose =
      r_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(0)
          .transpose();

  const auto nonlinear_continuity_constraint =
      std::make_shared<NonlinearContinuityConstraint>(
          Mu_transpose, Mv_transpose, continuity_order);
  solvers::VectorXDecisionVariable vars(2 * (order_ + 2));

  // Since hᵤᴺ, hᵥᴺ is the source of nonlinearities, we will replace it with
  // hᵤ₀ᴺ and hᵥ₀ᴺ, which are the characteristic times of the respective sets:
  // (dᴺrᵤ(s=1) / dsᴺ) / hᵤ₀ᴺ == (dᴺrᵥ(s=0) / dsᴺ) / hᵥ₀ᴺ.

  // Then we will get the following linear equality constraint as a surrogate:
  // urdot_control.col(order-N) * hᵥ₀ᴺ - vrdot_control.col(0) * hᵤ₀ᴺ = 0.

  // For simplicity sake, we will set hᵤ₀ and hᵥ₀ to one until we come up with a
  // reasonable heuristic, e.g. based on the maximum length of the convex set.

  const double hu0 = 1.0;
  const double hv0 = 1.0;

  // The latter can be achieved by getting the control point matrix M.
  // A = [Mu.col(order - continuity_order).T * hᵥ₀, -Mv.col(0).T * hᵤ₀],
  // x = [u_controls.row(i); v_controls.row(i)].
  // Ax = 0,

  // The A matrix will have one row since sparsity allows us to enforce the
  // continuity in each dimension. The number of columns matches the number of
  // control points for one row of r_u(s) and r_v(s).
  SparseMatrix<double> A(1, 2 * (order_ + 1));
  A.leftCols(order_ + 1) = Mu_transpose * hv0;
  A.rightCols(order_ + 1) = -Mv_transpose * hu0;

  const auto path_continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(A, VectorXd::Zero(1));

  for (int i = 0; i < num_positions(); ++i) {
    for (Edge* edge : edges_) {
      // Add convex surrogate.
      edge->AddConstraint(
          Binding<LinearEqualityConstraint>(
              path_continuity_constraint, {GetControlPoints(edge->u()).row(i),
                                           GetControlPoints(edge->v()).row(i)}),
          {Transcription::kRelaxation});

      // Add nonlinear constraint.
      vars << GetControlPoints(edge->u()).row(i).transpose(),
          GetTimeScaling(edge->u()),
          GetControlPoints(edge->v()).row(i).transpose(),
          GetTimeScaling(edge->v());
      edge->AddConstraint(Binding<NonlinearContinuityConstraint>(
                              nonlinear_continuity_constraint, vars),
                          {Transcription::kMIP, Transcription::kRestriction});
    }
  }
}

Eigen::Map<const MatrixX<symbolic::Variable>> Subgraph::GetControlPoints(
    const geometry::optimization::GraphOfConvexSets::Vertex& v) const {
  DRAKE_DEMAND(v.x().size() == num_positions() * (order_ + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      v.x().data(), num_positions(), order_ + 1);
}

symbolic::Variable Subgraph::GetTimeScaling(
    const geometry::optimization::GraphOfConvexSets::Vertex& v) const {
  DRAKE_DEMAND(v.x().size() == num_positions() * (order_ + 1) + 1);
  return v.x()(v.x().size() - 1);
}

namespace {

std::vector<Variable> FlattenVariables(
    const std::vector<Variable>& scalar_variables,
    const std::vector<VectorXDecisionVariable>& vector_variables,
    const std::vector<MatrixXDecisionVariable>& matrix_variables) {
  std::vector<Variable> all_variables;

  // Append scalar variables.
  all_variables.insert(all_variables.end(), scalar_variables.begin(),
                       scalar_variables.end());

  // Flatten vector variables.
  for (const auto& vector_variable : vector_variables) {
    for (int i = 0; i < vector_variable.size(); ++i) {
      all_variables.push_back(vector_variable(i));
    }
  }

  // Flatten matrix variables.
  for (const auto& matrix_variable : matrix_variables) {
    for (int i = 0; i < matrix_variable.rows(); ++i) {
      for (int j = 0; j < matrix_variable.cols(); ++j) {
        all_variables.push_back(matrix_variable(i, j));
      }
    }
  }

  return all_variables;
}

// Compatible with Expression and Formula.
template <typename T>
T SubstituteAllVariables(
    T e, std::vector<Variable> old_scalar_variables = {},
    std::vector<Variable> new_scalar_variables = {},
    std::vector<VectorXDecisionVariable> old_vector_variables = {},
    std::vector<VectorXDecisionVariable> new_vector_variables = {},
    std::vector<MatrixXDecisionVariable> old_matrix_variables = {},
    std::vector<MatrixXDecisionVariable> new_matrix_variables = {}) {
  DRAKE_DEMAND(old_scalar_variables.size() == new_scalar_variables.size());
  DRAKE_DEMAND(old_vector_variables.size() == new_vector_variables.size());
  DRAKE_DEMAND(old_matrix_variables.size() == new_matrix_variables.size());

  std::vector<Variable> old_variables = FlattenVariables(
      old_scalar_variables, old_vector_variables, old_matrix_variables);
  std::vector<Variable> new_variables = FlattenVariables(
      new_scalar_variables, new_vector_variables, new_matrix_variables);
  DRAKE_DEMAND(old_variables.size() == new_variables.size());

  symbolic::Variables e_vars = e.GetFreeVariables();
  for (int i = 0; i < ssize(old_variables); ++i) {
    if (e_vars.include(old_variables[i])) {
      e = e.Substitute(old_variables[i], new_variables[i]);
    }
  }
  return e;
}

// Compatible with Binding<Cost> and Binding<Constraint>.
template <typename T>
Binding<T> SubstituteAllVariables(
    const Binding<T>& binding, std::vector<Variable> old_scalar_variables = {},
    std::vector<Variable> new_scalar_variables = {},
    std::vector<VectorXDecisionVariable> old_vector_variables = {},
    std::vector<VectorXDecisionVariable> new_vector_variables = {},
    std::vector<MatrixXDecisionVariable> old_matrix_variables = {},
    std::vector<MatrixXDecisionVariable> new_matrix_variables = {}) {
  DRAKE_DEMAND(old_scalar_variables.size() == new_scalar_variables.size());
  DRAKE_DEMAND(old_vector_variables.size() == new_vector_variables.size());
  DRAKE_DEMAND(old_matrix_variables.size() == new_matrix_variables.size());

  std::vector<Variable> old_variables = FlattenVariables(
      old_scalar_variables, old_vector_variables, old_matrix_variables);
  std::vector<Variable> new_variables = FlattenVariables(
      new_scalar_variables, new_vector_variables, new_matrix_variables);
  DRAKE_DEMAND(old_variables.size() == new_variables.size());

  VectorXDecisionVariable new_binding_variables(binding.variables());
  for (int i = 0; i < new_binding_variables.size(); ++i) {
    // For the current placeholder variable, find its index in old_variables,
    // and grab the corresponding entry in new_variables.
    auto iterator = std::find_if(
        old_variables.begin(), old_variables.end(), [&](const Variable& var) {
          return var.equal_to(new_binding_variables[i]);
        });
    if (iterator == old_variables.end()) {
      // We throw an error if the user gave an unknown variable.
      throw std::runtime_error(
          fmt::format("Unknown variable with name {} provided.",
                      new_binding_variables[i].get_name()));
    }
    size_t index = std::distance(std::begin(old_variables), iterator);
    new_binding_variables[i] = new_variables[index];
  }

  return Binding<T>{binding.evaluator(), new_binding_variables};
}

// Compatible with Expression, Formula, Binding<Cost>, and Binding<Constraint>.
template <typename T>
void ThrowIfContainsVariables(const T& e, const std::vector<Variable>& vars,
                              const std::string& error_message) {
  symbolic::Variables e_vars;
  if constexpr (std::disjunction_v<std::is_same<T, Formula>,
                                   std::is_same<T, Expression>>) {
    e_vars = e.GetFreeVariables();
  } else {
    e_vars = symbolic::Variables(e.variables());
  }
  for (const auto& var : vars) {
    if (e_vars.include(var)) {
      throw(std::runtime_error(error_message));
    }
  }
}

}  // namespace

// Compatible with Expression, Formula, Binding<Cost>, and Binding<Constraint>.
template <typename T>
T Subgraph::SubstituteVertexPlaceholderVariables(T e,
                                                 const Vertex& vertex) const {
  // Check that a user hasn't used the edge placeholder variables.
  const std::string error_message =
      "Edge placeholder variables cannot be used to add vertex costs or "
      "constraints.";
  ThrowIfContainsVariables(
      e,
      FlattenVariables({placeholder_edge_durations_var_.first,
                        placeholder_edge_durations_var_.second},
                       {},
                       {placeholder_edge_control_points_var_.first,
                        placeholder_edge_control_points_var_.second}),
      error_message);

  return SubstituteAllVariables(
      e, {placeholder_vertex_duration_var_}, {GetTimeScaling(vertex)}, {}, {},
      {placeholder_vertex_control_points_var_}, {GetControlPoints(vertex)});
}

// Compatible with Expression, Formula, Binding<Cost>, and Binding<Constraint>.
template <typename T>
T Subgraph::SubstituteEdgePlaceholderVariables(T e, const Edge& edge) const {
  // Check that a user hasn't used the vertex placeholder variables.
  const std::string error_message =
      "Vertex placeholder variables cannot be used to add edge costs or "
      "constraints.";
  ThrowIfContainsVariables(
      e,
      FlattenVariables({placeholder_vertex_duration_var_}, {},
                       {placeholder_vertex_control_points_var_}),
      error_message);

  const Vertex& v1 = edge.u();
  const Vertex& v2 = edge.v();

  return SubstituteAllVariables(e,
                                {placeholder_edge_durations_var_.first,
                                 placeholder_edge_durations_var_.second},
                                {GetTimeScaling(v1), GetTimeScaling(v2)}, {},
                                {},
                                {placeholder_edge_control_points_var_.first,
                                 placeholder_edge_control_points_var_.second},
                                {GetControlPoints(v1), GetControlPoints(v2)});
}

void Subgraph::AddVertexCost(
    const Expression& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddVertexCost(e, use_in_transcription);
}
void Subgraph::AddVertexCost(
    const Binding<Cost>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddVertexCost(binding, use_in_transcription);
}

// Compatible with Expression and Binding<Cost>.
template <typename T>
void Subgraph::DoAddVertexCost(
    const T& e, const std::unordered_set<Transcription>& use_in_transcription) {
  for (Vertex*& vertex : vertices_) {
    T post_substitution = SubstituteVertexPlaceholderVariables(e, *vertex);
    vertex->AddCost(post_substitution, use_in_transcription);
  }
}

void Subgraph::AddVertexConstraint(
    const Formula& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddVertexConstraint(e, use_in_transcription);
}
void Subgraph::AddVertexConstraint(
    const Binding<Constraint>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddVertexConstraint(binding, use_in_transcription);
}

// Compatible with Formula and Binding<Constraint>.
template <typename T>
void Subgraph::DoAddVertexConstraint(
    const T& e, const std::unordered_set<Transcription>& use_in_transcription) {
  for (Vertex*& vertex : vertices_) {
    T post_substitution = SubstituteVertexPlaceholderVariables(e, *vertex);
    vertex->AddConstraint(post_substitution, use_in_transcription);
  }
}
void Subgraph::AddEdgeCost(
    const Expression& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddEdgeCost(e, use_in_transcription);
}

void Subgraph::AddEdgeCost(
    const Binding<Cost>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddEdgeCost(binding, use_in_transcription);
}

// Compatible with Expression and Binding<Cost>.
template <typename T>
void Subgraph::DoAddEdgeCost(
    const T& e, const std::unordered_set<Transcription>& use_in_transcription) {
  for (Edge*& edge : edges_) {
    T post_substitution = SubstituteEdgePlaceholderVariables(e, *edge);
    edge->AddCost(post_substitution, use_in_transcription);
  }
}

void Subgraph::AddEdgeConstraint(
    const Formula& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddEdgeConstraint(e, use_in_transcription);
}

void Subgraph::AddEdgeConstraint(
    const Binding<Constraint>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddEdgeConstraint(binding, use_in_transcription);
}

// Compatible with Formula and Binding<Constraint>.
template <typename T>
void Subgraph::DoAddEdgeConstraint(
    const T& e, const std::unordered_set<Transcription>& use_in_transcription) {
  for (Edge*& edge : edges_) {
    T post_substitution = SubstituteEdgePlaceholderVariables(e, *edge);
    edge->AddConstraint(post_substitution, use_in_transcription);
  }
}

EdgesBetweenSubgraphs::EdgesBetweenSubgraphs(
    const Subgraph& from_subgraph, const Subgraph& to_subgraph,
    const ConvexSet* subspace, GcsTrajectoryOptimization* traj_opt,
    const std::vector<std::pair<int, int>>* edges_between_regions,
    const std::vector<Eigen::VectorXd>* edge_offsets)
    : traj_opt_(*traj_opt),
      from_subgraph_(from_subgraph),
      to_subgraph_(to_subgraph) {
  if (edge_offsets) {
    if (!edges_between_regions) {
      throw std::runtime_error(
          "If edge_offsets are specified, then edges_between_regions must also "
          "be specified.");
    }
    DRAKE_THROW_UNLESS(edge_offsets->size() == edges_between_regions->size());
  }
  // Formulate edge costs and constraints.
  if (subspace != nullptr) {
    if (subspace->ambient_dimension() != num_positions()) {
      throw std::runtime_error(
          "Subspace dimension must match the number of positions.");
    }
    if (typeid(*subspace) != typeid(Point) &&
        typeid(*subspace) != typeid(HPolyhedron)) {
      throw std::runtime_error("Subspace must be a Point or HPolyhedron.");
    }
  }

  ur_trajectory_ = BezierCurve<double>(
      0, 1, MatrixXd::Zero(num_positions(), from_subgraph_.order() + 1));
  vr_trajectory_ = BezierCurve<double>(
      0, 1, MatrixXd::Zero(num_positions(), to_subgraph_.order() + 1));

  // Zeroth order continuity constraints.
  //  ur_control.col(-1) == vr_control.col(0).
  SparseMatrix<double> A(num_positions(), 2 * num_positions());  // A = [I, -I].
  std::vector<Eigen::Triplet<double>> tripletList;
  tripletList.reserve(2 * num_positions());
  for (int i = 0; i < num_positions(); ++i) {
    tripletList.push_back(Eigen::Triplet<double>(i, i, 1.0));
    tripletList.push_back(Eigen::Triplet<double>(i, num_positions() + i, -1.0));
  }
  A.setFromTriplets(tripletList.begin(), tripletList.end());

  std::vector<VectorXd> sets_A_subspace_offset;
  if (subspace != nullptr) {
    std::vector<std::pair<double, double>> subspace_revolute_joints_bbox =
        GetMinimumAndMaximumValueAlongDimension(*subspace,
                                                continuous_revolute_joints());
    for (const auto& region : from_subgraph.regions()) {
      std::vector<std::pair<double, double>> set_revolute_joints_bbox =
          GetMinimumAndMaximumValueAlongDimension(*region,
                                                  continuous_revolute_joints());
      sets_A_subspace_offset.push_back(ComputeOffsetContinuousRevoluteJoints(
          num_positions(), continuous_revolute_joints(),
          set_revolute_joints_bbox, subspace_revolute_joints_bbox));
    }
  }

  // These will hold the edge data if they weren't given as arguments.
  std::optional<std::vector<std::pair<int, int>>> maybe_edges_between_regions;
  std::optional<std::vector<Eigen::VectorXd>> maybe_edge_offsets;

  if (edges_between_regions) {
    for (const auto& [i, j] : *edges_between_regions) {
      DRAKE_THROW_UNLESS(0 <= i && 0 <= j && i < from_subgraph_.size() &&
                         j < to_subgraph_.size());
    }

    if (!edge_offsets) {
      // Compute bounding boxes for the regions along dimensions corresponding
      // to continuous revolute joints, and use them to determine the edge
      // offsets.
      std::vector<std::vector<std::pair<double, double>>> continuous_bboxes_A;
      continuous_bboxes_A.reserve(ssize(from_subgraph.regions()));
      for (const auto& region_ptr : from_subgraph.regions()) {
        continuous_bboxes_A.push_back(GetMinimumAndMaximumValueAlongDimension(
            *region_ptr, continuous_revolute_joints()));
      }
      std::vector<std::vector<std::pair<double, double>>> continuous_bboxes_B;
      continuous_bboxes_B.reserve(ssize(to_subgraph.regions()));
      for (const auto& region_ptr : to_subgraph.regions()) {
        continuous_bboxes_B.push_back(GetMinimumAndMaximumValueAlongDimension(
            *region_ptr, continuous_revolute_joints()));
      }
      maybe_edge_offsets = std::vector<VectorXd>{};
      maybe_edge_offsets->reserve(edges_between_regions->size());
      for (const auto& [i, j] : *edges_between_regions) {
        maybe_edge_offsets->push_back(ComputeOffsetContinuousRevoluteJoints(
            num_positions(), continuous_revolute_joints(),
            continuous_bboxes_A[i], continuous_bboxes_B[j]));
      }
      edge_offsets = &(maybe_edge_offsets.value());
    }
  } else {
    std::tie(maybe_edges_between_regions, maybe_edge_offsets) =
        ComputePairwiseIntersections(from_subgraph.regions(),
                                     to_subgraph.regions(),
                                     continuous_revolute_joints());
    DRAKE_DEMAND(maybe_edges_between_regions->size() ==
                 maybe_edge_offsets->size());
    edges_between_regions = &(maybe_edges_between_regions.value());
    edge_offsets = &(maybe_edge_offsets.value());
  }

  for (int edge_idx = 0; edge_idx < ssize(*edges_between_regions); ++edge_idx) {
    int i = (*edges_between_regions)[edge_idx].first;
    int j = (*edges_between_regions)[edge_idx].second;
    // If the user specified edges_between_regions but not edge_offsets, then
    // edge_offsets are implicitly assumed to be zero. That is indicated here by
    // edge_offsets being nullptr.
    const Eigen::VectorXd& edge_offset = (*edge_offsets)[edge_idx];

    // Check if the overlap between the sets is contained in the subspace.
    if (subspace != nullptr) {
      Eigen::VectorXd subspace_offset = sets_A_subspace_offset[i];
      if (!RegionsConnectThroughSubspace(*from_subgraph.regions()[i],
                                         *to_subgraph.regions()[j], *subspace,
                                         &edge_offset, &subspace_offset)) {
        continue;
      }
    }

    // Add edge.
    Vertex* u = from_subgraph.vertices_[i];
    Vertex* v = to_subgraph.vertices_[j];
    Edge* uv_edge = traj_opt_.AddEdge(u, v);
    edges_.emplace_back(uv_edge);

    // Add path continuity constraints. We instead enforce the constraint
    // u - v = -tau_uv, via Ax = -edge_offset, A = [I, -I],
    // x = [u_controls.col(order); v_controls.col(0)].
    const auto path_continuity_constraint =
        std::make_shared<LinearEqualityConstraint>(A, -edge_offset);
    uv_edge->AddConstraint(Binding<Constraint>(
        path_continuity_constraint,
        ConcatenateVariableRefList(
            {GetControlPointsU(*uv_edge).col(from_subgraph_.order()),
             GetControlPointsV(*uv_edge).col(0)})));

    if (subspace != nullptr) {
      // Add subspace constraints to the last control point of the u vertex.
      const auto vars = GetControlPointsU(*uv_edge).col(from_subgraph_.order());

      // subspace will either be a Point or HPolyhedron. We check if it's a
      // Point via the method ConvexSet::MaybeGetPoint(), which will return a
      // value for Point, and won't return a value for HPolyhedron.
      std::optional<VectorXd> subspace_point = subspace->MaybeGetPoint();
      if (subspace_point.has_value()) {
        // Encode u = subspace_point + subspace_offset via the linear equality
        // constraint [I][u] = [subspace_point + subspace_offset].
        const auto subgraph_constraint =
            std::make_shared<LinearEqualityConstraint>(
                Eigen::MatrixXd::Identity(num_positions(), num_positions()),
                subspace_point.value() + sets_A_subspace_offset[i]);
        uv_edge->AddConstraint(Binding<Constraint>(subgraph_constraint, vars));
      } else if (typeid(*subspace) == typeid(HPolyhedron)) {
        const HPolyhedron* hpoly = dynamic_cast<const HPolyhedron*>(subspace);
        const Eigen::MatrixXd& hpoly_A = hpoly->A();
        const Eigen::VectorXd& hpoly_b = hpoly->b();
        solvers::MathematicalProgram prog;
        const VectorX<symbolic::Variable> x =
            prog.NewContinuousVariables(num_positions(), "x");
        // To translate the HPolyhedron, rewrite A*(x+offset)<=b as
        // Ax<=b-A*offset.
        prog.AddLinearConstraint(
            hpoly_A,
            Eigen::VectorXd::Constant(hpoly_b.size(),
                                      -std::numeric_limits<double>::infinity()),
            hpoly_b - (hpoly_A * sets_A_subspace_offset[i]), x);
        for (const auto& binding : prog.GetAllConstraints()) {
          const std::shared_ptr<Constraint>& constraint = binding.evaluator();
          uv_edge->AddConstraint(Binding<Constraint>(constraint, vars));
        }
      }
    }
  }

  // Construct placeholder variables.
  placeholder_edge_durations_var_ =
      std::make_pair(symbolic::Variable("tu"), symbolic::Variable("tv"));
  placeholder_edge_control_points_var_ =
      std::make_pair(MakeMatrixContinuousVariable(
                         num_positions(), from_subgraph_.order() + 1, "xu"),
                     MakeMatrixContinuousVariable(
                         num_positions(), to_subgraph_.order() + 1, "xv"));
}

EdgesBetweenSubgraphs::~EdgesBetweenSubgraphs() = default;

bool EdgesBetweenSubgraphs::RegionsConnectThroughSubspace(
    const ConvexSet& A, const ConvexSet& B, const ConvexSet& subspace,
    const VectorXd* maybe_set_B_offset, const VectorXd* maybe_subspace_offset) {
  DRAKE_THROW_UNLESS(A.ambient_dimension() > 0);
  DRAKE_THROW_UNLESS(A.ambient_dimension() == B.ambient_dimension());
  DRAKE_THROW_UNLESS(A.ambient_dimension() == subspace.ambient_dimension());
  // If the set_B offset is given, the subspace_offset must be given as well.
  DRAKE_THROW_UNLESS((maybe_set_B_offset && maybe_subspace_offset) ||
                     (!maybe_set_B_offset && !maybe_subspace_offset));
  if (maybe_set_B_offset) {
    DRAKE_THROW_UNLESS(maybe_set_B_offset->size() == A.ambient_dimension());
    DRAKE_THROW_UNLESS(maybe_subspace_offset->size() == A.ambient_dimension());
  }
  if (std::optional<VectorXd> subspace_point = subspace.MaybeGetPoint()) {
    // If the subspace is a point, then the point must be in both A and B.
    if (maybe_set_B_offset) {
      // Compute the value of the point such that it can be directly checked for
      // containment in A. We take its given value, and subtract the offset from
      // the set to the subspace.
      const VectorXd point_in_A_coords =
          *subspace_point - *maybe_subspace_offset;
      // Compute the value of the point such that it can be directly checked for
      // containment in B. We take the value of the point that can be checked
      // for A, and add the offset form A to B.
      const VectorXd point_in_B_coords =
          point_in_A_coords + *maybe_set_B_offset;
      return A.PointInSet(point_in_A_coords) && B.PointInSet(point_in_B_coords);
    } else {
      return A.PointInSet(*subspace_point) && B.PointInSet(*subspace_point);
    }
  } else if (!maybe_set_B_offset) {
    // Otherwise, we can formulate a problem to check if a point is contained in
    // A, B and the subspace.
    Intersection intersection(MakeConvexSets(A, B, subspace));
    return !intersection.IsEmpty();
  } else {
    // The program has to be different if there's an offset.
    const int dimension = A.ambient_dimension();
    MathematicalProgram prog;
    VectorXDecisionVariable x = prog.NewContinuousVariables(dimension);
    VectorXDecisionVariable y = prog.NewContinuousVariables(dimension);
    VectorXDecisionVariable z = prog.NewContinuousVariables(dimension);
    A.AddPointInSetConstraints(&prog, x);
    B.AddPointInSetConstraints(&prog, y);
    subspace.AddPointInSetConstraints(&prog, z);

    MatrixXd equality_constraint_matrix(dimension, 2 * dimension);
    equality_constraint_matrix.leftCols(dimension) =
        MatrixXd::Identity(dimension, dimension);
    equality_constraint_matrix.rightCols(dimension) =
        -MatrixXd::Identity(dimension, dimension);
    // y = x + B_offset as [I, -I][y; x] = [B_offset].
    prog.AddLinearEqualityConstraint(equality_constraint_matrix,
                                     *maybe_set_B_offset, {x, y});
    // z = x + subspace_offset as [I, -I][z; x] = [subspace_offset]
    prog.AddLinearEqualityConstraint(equality_constraint_matrix,
                                     *maybe_subspace_offset, {x, z});

    const auto result = Solve(prog);
    return result.is_success();
  }
}

void EdgesBetweenSubgraphs::AddVelocityBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());

  // We have q̇(t) = drds * dsdt = ṙ(s) / h, and h >= 0, so we
  // use h * lb <= ṙ(s) <= h * ub, formulated as:
  //     0 <= ṙ(s) - h * lb <= inf,
  // - inf <= ṙ(s) - h * ub <= 0.

  // We will enforce the velocity bounds on the last control point of the u set
  // and on the first control point of the v set unless one of the sets are of
  // order zero. In the zero order case, velocity doesn't matter since its a
  // point.

  const Vector1d kVecInf = Vector1d::Constant(kInf);
  const Vector1d kVecZero = Vector1d::Zero();

  if (from_subgraph_.order() == 0 && to_subgraph_.order() == 0) {
    throw std::runtime_error(
        "Cannot add velocity bounds to a subgraph edges where both subgraphs "
        "have zero order.");
  }

  if (from_subgraph_.order() > 0) {
    // Add velocity bounds to the last control point of the u set.
    // See BezierCurve::AsLinearInControlPoints().
    solvers::VectorXDecisionVariable vars(from_subgraph_.order() + 2);
    SparseMatrix<double> m = ur_trajectory_.AsLinearInControlPoints(1)
                                 .col(from_subgraph_.order() - 1)
                                 .transpose();
    SparseMatrix<double> H_lb(
        1 /* we are only constraining the last point in the u set */,
        from_subgraph_.order() +
            2 /* number of control points for one row of r(s) + 1*/);
    H_lb.leftCols(from_subgraph_.order() + 1) = m;
    SparseMatrix<double> H_ub = H_lb;
    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
      H_lb.coeffRef(0, from_subgraph_.order() + 1) = -lb[i];
      const auto lb_constraint =
          std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
      // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
      H_ub.coeffRef(0, from_subgraph_.order() + 1) = -ub[i];
      const auto ub_constraint =
          std::make_shared<LinearConstraint>(H_ub, -kVecInf, kVecZero);
      for (Edge* edge : edges_) {
        // vars = [control_points.row(i).T; time_scaling]
        vars << GetControlPointsU(*edge).row(i).transpose(),
            GetTimeScalingU(*edge);
        edge->AddConstraint(Binding<LinearConstraint>(lb_constraint, vars));
        edge->AddConstraint(Binding<LinearConstraint>(ub_constraint, vars));
      }
    }
  }

  if (to_subgraph_.order() > 0) {
    // Add velocity bounds to the first control point of the v set.
    // See Subgraph::AddVelocityBounds().
    solvers::VectorXDecisionVariable vars(to_subgraph_.order() + 2);
    SparseMatrix<double> m =
        vr_trajectory_.AsLinearInControlPoints(1).col(0).transpose();
    SparseMatrix<double> H_lb(
        1 /* we are only constraining the last point in the u set */,
        to_subgraph_.order() +
            2 /* number of control points for one row of r(s) + 1*/);
    H_lb.leftCols(to_subgraph_.order() + 1) = m;
    SparseMatrix<double> H_ub = H_lb;
    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
      H_lb.coeffRef(0, to_subgraph_.order() + 1) = -lb[i];
      const auto lb_constraint =
          std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
      // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
      H_ub.coeffRef(0, to_subgraph_.order() + 1) = -ub[i];
      const auto ub_constraint =
          std::make_shared<LinearConstraint>(H_ub, -kVecInf, kVecZero);
      for (Edge* edge : edges_) {
        // vars = [control_points.row(i).T; time_scaling]
        vars << GetControlPointsV(*edge).row(i).transpose(),
            GetTimeScalingV(*edge);
        edge->AddConstraint(Binding<LinearConstraint>(lb_constraint, vars));
        edge->AddConstraint(Binding<LinearConstraint>(ub_constraint, vars));
      }
    }
  }
}

void EdgesBetweenSubgraphs::AddNonlinearDerivativeBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, int derivative_order) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());

  if (derivative_order < 1) {
    throw std::runtime_error("Derivative order must be greater than 1.");
  }

  if (derivative_order == 1) {
    throw std::runtime_error(
        "Use AddVelocityBounds instead of AddNonlinearDerivativeBounds with "
        "derivative_order=1; velocity bounds are linear.");
  }

  if (derivative_order > from_subgraph_.order() &&
      derivative_order > to_subgraph_.order()) {
    throw std::runtime_error(fmt::format(
        "Cannot add derivative bounds to subgraph edges where both subgraphs "
        "have less than derivative order.\n From subgraph order: {}\n To "
        "subgraph order: {}\n Derivative order: {}",
        from_subgraph_.order(), to_subgraph_.order(), derivative_order));
  }

  // See see Subgraph::AddNonlinearDerivativeBounds for details on how the
  // nonlinear derivative constraints are formulated.

  // We will enforce the derivative bounds on the last control point of the u
  // set and on the first control point of the v set unless one of the sets
  // order is less than the derivative order.
  const VectorXd kVecInf = VectorXd::Constant(2, kInf);
  const VectorXd kVecZero = VectorXd::Zero(2);
  const double h0 = 1.0;

  if (from_subgraph_.order() >= derivative_order) {
    // Add derivative bounds to the last control point of the u set.
    // See BezierCurve::AsLinearInControlPoints().

    solvers::VectorXDecisionVariable vars(from_subgraph_.order() + 2);
    SparseMatrix<double> M_transpose =
        ur_trajectory_.AsLinearInControlPoints(derivative_order)
            .col(from_subgraph_.order() - derivative_order)
            .transpose();

    Eigen::MatrixXd H(
        2 /* we are only constraining the last point in the u set */,
        from_subgraph_.order() +
            2 /* number of control points for one row of r(s) + 1*/);
    H.block(0, 0, 1, from_subgraph_.order() + 1) = M_transpose;
    H.block(1, 0, 1, from_subgraph_.order() + 1) = -M_transpose;

    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound: 0 <=   (dᴺr(s=1) / dsᴺ).row(i) - h₀ᴺ⁻¹ h * lb[i] <= inf,
      // Upper bound: 0 <= - (dᴺr(s=1) / dsᴺ).row(i) + h₀ᴺ⁻¹ h * ub[i] <= inf.
      H(0, from_subgraph_.order() + 1) =
          -std::pow(h0, derivative_order - 1) * lb[i];
      H(1, from_subgraph_.order() + 1) =
          std::pow(h0, derivative_order - 1) * ub[i];
      const auto convex_derivative_constraint =
          std::make_shared<LinearConstraint>(H.sparseView(), kVecZero, kVecInf);

      const auto nonlinear_derivative_constraint =
          std::make_shared<NonlinearDerivativeConstraint>(
              M_transpose, lb[i], ub[i], derivative_order);
      for (Edge* edge : edges_) {
        vars << GetControlPointsU(*edge).row(i).transpose(),
            GetTimeScalingU(*edge);
        // Add convex surrogate.
        edge->AddConstraint(
            Binding<LinearConstraint>(convex_derivative_constraint, vars),
            {Transcription::kRelaxation});
        // Add nonlinear constraint.
        edge->AddConstraint(Binding<NonlinearDerivativeConstraint>(
                                nonlinear_derivative_constraint, vars),
                            {Transcription::kMIP, Transcription::kRestriction});
      }
    }
  }

  if (to_subgraph_.order() >= derivative_order) {
    // Add velocity bounds to the first control point of the v set.
    // See BezierCurve::AsLinearInControlPoints().

    solvers::VectorXDecisionVariable vars(to_subgraph_.order() + 2);
    SparseMatrix<double> M_transpose =
        vr_trajectory_.AsLinearInControlPoints(derivative_order)
            .col(0)
            .transpose();

    Eigen::MatrixXd H(
        2 /* we are only constraining the last point in the v set */,
        to_subgraph_.order() +
            2 /* number of control points for one row of r(s) + 1*/);
    H.block(0, 0, 1, to_subgraph_.order() + 1) = M_transpose;
    H.block(1, 0, 1, to_subgraph_.order() + 1) = -M_transpose;

    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound: 0 <=   (dᴺr(s=0) / dsᴺ).row(i) - h₀ᴺ⁻¹h * lb[i] <= inf,
      // Upper bound: 0 <= - (dᴺr(s=0) / dsᴺ).row(i) + h₀ᴺ⁻¹h * ub[i] <= inf.
      H(0, to_subgraph_.order() + 1) =
          -std::pow(h0, derivative_order - 1) * lb[i];
      H(1, to_subgraph_.order() + 1) =
          std::pow(h0, derivative_order - 1) * ub[i];
      const auto convex_derivative_constraint =
          std::make_shared<LinearConstraint>(H.sparseView(), kVecZero, kVecInf);

      const auto nonlinear_derivative_constraint =
          std::make_shared<NonlinearDerivativeConstraint>(
              M_transpose, lb[i], ub[i], derivative_order);
      for (Edge* edge : edges_) {
        vars << GetControlPointsV(*edge).row(i).transpose(),
            GetTimeScalingU(*edge);
        // Add convex surrogate.
        edge->AddConstraint(
            Binding<LinearConstraint>(convex_derivative_constraint, vars),
            {Transcription::kRelaxation});
        // Add nonlinear constraint.
        edge->AddConstraint(Binding<NonlinearDerivativeConstraint>(
                                nonlinear_derivative_constraint, vars),
                            {Transcription::kMIP, Transcription::kRestriction});
      }
    }
  }
}

void EdgesBetweenSubgraphs::AddZeroDerivativeConstraints(int derivative_order) {
  if (derivative_order < 1) {
    throw std::runtime_error("Derivative order must be greater than 1.");
  }

  if (derivative_order > from_subgraph_.order() &&
      derivative_order > to_subgraph_.order()) {
    throw std::runtime_error(fmt::format(
        "Cannot add derivative bounds to subgraph edges where both subgraphs "
        "have less than derivative order.\n From subgraph order: {}\n To "
        "subgraph order: {}\n Derivative order: {}",
        from_subgraph_.order(), to_subgraph_.order(), derivative_order));
  }

  // We have dᴺq(t) / dtᴺ = dᴺr(s)/(dsᴺ * hᴺ) and h >= 0, which is nonlinear.
  // To constraint zero velocity we can set the numerator to zero, which is
  // convex:
  // dᴺr(s) / dsᴺ = 0.

  const Vector1d kVecZero = Vector1d::Zero();

  if (from_subgraph_.order() >= derivative_order) {
    // Add derivative bounds to the last control point of the u set.
    // See BezierCurve::AsLinearInControlPoints().
    SparseMatrix<double> M_transpose =
        ur_trajectory_.AsLinearInControlPoints(derivative_order)
            .col(from_subgraph_.order() - derivative_order)
            .transpose();

    // Equality constraint.
    // (dᴺr(s) / dsᴺ).row(i) = 0.
    const auto zero_derivative_constraint =
        std::make_shared<LinearEqualityConstraint>(M_transpose, kVecZero);
    for (int i = 0; i < num_positions(); ++i) {
      for (Edge* edge : edges_) {
        edge->AddConstraint(Binding<LinearEqualityConstraint>(
            zero_derivative_constraint,
            GetControlPointsU(*edge).row(i).transpose()));
      }
    }
  }

  if (to_subgraph_.order() >= derivative_order) {
    // Add derivative bounds to the first control point of the v set.
    // See BezierCurve::AsLinearInControlPoints().
    SparseMatrix<double> M_transpose =
        vr_trajectory_.AsLinearInControlPoints(derivative_order)
            .col(0)
            .transpose();
    // Equality constraint:
    // (dᴺr(s) / dsᴺ).row(i) = 0.
    const auto zero_derivative_constraint =
        std::make_shared<LinearEqualityConstraint>(M_transpose, kVecZero);

    for (int i = 0; i < num_positions(); ++i) {
      for (Edge* edge : edges_) {
        edge->AddConstraint(Binding<LinearEqualityConstraint>(
            zero_derivative_constraint,
            GetControlPointsV(*edge).row(i).transpose()));
      }
    }
  }
}

void EdgesBetweenSubgraphs::AddPathContinuityConstraints(int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }

  if (continuity_order > from_subgraph_.order() ||
      continuity_order > to_subgraph_.order()) {
    throw std::runtime_error(
        "Cannot add continuity constraint to a subgraph edge where both "
        "subgraphs order are not greater than or equal to the requested "
        "continuity order.");
  }

  // The continuity on derivatives of r(s) will be enforced between the last
  // control point of the u set and the first control point of the v set in an
  // edge. urdot_control.col(order-continuity_order) - vrdot_control.col(0) = 0,
  // The latter can be achieved by getting the control point matrix M.
  // A = [M.col(from_subgraph.order() - continuity_order).T, -M.col(0).T],
  // x = [u_controls.row(i); v_controls.row(i)].
  // Ax = 0,

  SparseMatrix<double> Mu_transpose =
      ur_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(from_subgraph_.order() - continuity_order)
          .transpose();

  SparseMatrix<double> Mv_transpose =
      vr_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(0)
          .transpose();

  // Concatenate Mu_transpose and Mv_transpose.
  // A = [Mu.T, - Mv.T]

  // The A matrix will have one row since sparsity allows us to enforce the
  // continuity in each dimension. The number of columns matches the number of
  // control points for one row of r_u(s) and r_v(s).
  SparseMatrix<double> A(
      1, (from_subgraph_.order() + 1) + (to_subgraph_.order() + 1));
  A.leftCols(from_subgraph_.order() + 1) = Mu_transpose;
  A.rightCols(to_subgraph_.order() + 1) = -Mv_transpose;

  const auto continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(A, VectorXd::Zero(1));

  for (int i = 0; i < num_positions(); ++i) {
    for (Edge* edge : edges_) {
      // Add continuity constraints.
      edge->AddConstraint(Binding<LinearEqualityConstraint>(
          continuity_constraint,
          {GetControlPointsU(*edge).row(i), GetControlPointsV(*edge).row(i)}));
    }
  }
}

void EdgesBetweenSubgraphs::AddContinuityConstraints(int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }

  if (continuity_order > from_subgraph_.order() ||
      continuity_order > to_subgraph_.order()) {
    throw std::runtime_error(
        "Cannot add continuity constraint to a subgraph edge where both "
        "subgraphs order are not greater than or equal to the requested "
        "continuity order.");
  }

  // See see Subgraph::AddContinuityConstraints for details on how the
  // nonlinear derivative constraints are formulated.

  // The continuity on derivatives of q(s) will be enforced between the last
  // control point of the u set and the first control point of the v set in an
  // edge.

  const double hu0 = 1.0;
  const double hv0 = 1.0;

  SparseMatrix<double> Mu_transpose =
      ur_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(from_subgraph_.order() - continuity_order)
          .transpose();

  SparseMatrix<double> Mv_transpose =
      vr_trajectory_.AsLinearInControlPoints(continuity_order)
          .col(0)
          .transpose();

  const auto nonlinear_continuity_constraint =
      std::make_shared<NonlinearContinuityConstraint>(
          Mu_transpose, Mv_transpose, continuity_order);
  solvers::VectorXDecisionVariable vars(from_subgraph_.order() + 2 +
                                        to_subgraph_.order() + 2);

  // The A matrix will have one row since sparsity allows us to enforce the
  // continuity in each dimension. The number of columns matches the number of
  // control points for one row of r_u(s) and r_v(s).
  SparseMatrix<double> A(
      1, (from_subgraph_.order() + 1) + (to_subgraph_.order() + 1));
  A.leftCols(from_subgraph_.order() + 1) = Mu_transpose * hv0;
  A.rightCols(to_subgraph_.order() + 1) = -Mv_transpose * hu0;

  const auto path_continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(A, VectorXd::Zero(1));

  for (int i = 0; i < num_positions(); ++i) {
    for (Edge* edge : edges_) {
      // Add convex surrogate.
      edge->AddConstraint(
          Binding<LinearEqualityConstraint>(path_continuity_constraint,
                                            {GetControlPointsU(*edge).row(i),
                                             GetControlPointsV(*edge).row(i)}),
          {Transcription::kRelaxation});

      // Add nonlinear constraint.
      vars << GetControlPointsU(*edge).row(i).transpose(),
          GetTimeScalingU(*edge), GetControlPointsV(*edge).row(i).transpose(),
          GetTimeScalingV(*edge);
      edge->AddConstraint(Binding<NonlinearContinuityConstraint>(
                              nonlinear_continuity_constraint, vars),
                          {Transcription::kMIP, Transcription::kRestriction});
    }
  }
}

std::vector<const GraphOfConvexSets::Edge*> EdgesBetweenSubgraphs::Edges()
    const {
  std::vector<const GraphOfConvexSets::Edge*> edges;
  edges.reserve(edges_.size());
  for (const auto& e : edges_) {
    edges.push_back(e);
  }
  return edges;
}

template <typename T>
T EdgesBetweenSubgraphs::SubstituteEdgePlaceholderVariables(
    T e, const Edge& edge) const {
  return SubstituteAllVariables(
      e,
      {placeholder_edge_durations_var_.first,
       placeholder_edge_durations_var_.second},
      {GetTimeScalingU(edge), GetTimeScalingV(edge)}, {}, {},
      {placeholder_edge_control_points_var_.first,
       placeholder_edge_control_points_var_.second},
      {GetControlPointsU(edge), GetControlPointsV(edge)});
}

void EdgesBetweenSubgraphs::AddEdgeCost(
    const Expression& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  return DoAddEdgeCost(e, use_in_transcription);
}

void EdgesBetweenSubgraphs::AddEdgeCost(
    const Binding<Cost>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  return DoAddEdgeCost(binding, use_in_transcription);
}

// Compatible with Expression and Binding<Cost>.
template <typename T>
void EdgesBetweenSubgraphs::DoAddEdgeCost(
    const T& e, const std::unordered_set<Transcription>& use_in_transcription) {
  for (Edge*& edge : edges_) {
    T post_substitution = SubstituteEdgePlaceholderVariables(e, *edge);
    edge->AddCost(post_substitution, use_in_transcription);
  }
}

void EdgesBetweenSubgraphs::AddEdgeConstraint(
    const Formula& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddEdgeConstraint(e, use_in_transcription);
}

void EdgesBetweenSubgraphs::AddEdgeConstraint(
    const Binding<Constraint>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DoAddEdgeConstraint(binding, use_in_transcription);
}
// Compatible with Formula and Binding<Constraint>.
template <typename T>
void EdgesBetweenSubgraphs::DoAddEdgeConstraint(
    const T& e, const std::unordered_set<Transcription>& use_in_transcription) {
  for (Edge*& edge : edges_) {
    T post_substitution = SubstituteEdgePlaceholderVariables(e, *edge);
    edge->AddConstraint(post_substitution, use_in_transcription);
  }
}

Eigen::Map<const MatrixX<symbolic::Variable>>
EdgesBetweenSubgraphs::GetControlPointsU(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xu().size() ==
               num_positions() * (from_subgraph_.order() + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      e.xu().data(), num_positions(), from_subgraph_.order() + 1);
}

Eigen::Map<const MatrixX<symbolic::Variable>>
EdgesBetweenSubgraphs::GetControlPointsV(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xv().size() ==
               num_positions() * (to_subgraph_.order() + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      e.xv().data(), num_positions(), to_subgraph_.order() + 1);
}

symbolic::Variable EdgesBetweenSubgraphs::GetTimeScalingU(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xu().size() ==
               num_positions() * (from_subgraph_.order() + 1) + 1);
  return e.xu()(e.xu().size() - 1);
}

symbolic::Variable EdgesBetweenSubgraphs::GetTimeScalingV(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xv().size() ==
               num_positions() * (to_subgraph_.order() + 1) + 1);
  return e.xv()(e.xv().size() - 1);
}

GcsTrajectoryOptimization::GcsTrajectoryOptimization(
    int num_positions, std::vector<int> continuous_revolute_joints)
    : num_positions_(num_positions),
      continuous_revolute_joints_(std::move(continuous_revolute_joints)) {
  DRAKE_THROW_UNLESS(num_positions >= 1);
  ThrowsForInvalidContinuousJointsList(num_positions,
                                       continuous_revolute_joints_);
}

GcsTrajectoryOptimization::~GcsTrajectoryOptimization() = default;

Subgraph& GcsTrajectoryOptimization::AddRegions(
    const ConvexSets& regions,
    const std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double h_min, double h_max, std::string name,
    const std::vector<VectorXd>* edge_offsets) {
  if (edge_offsets) {
    DRAKE_THROW_UNLESS(edge_offsets->size() == edges_between_regions.size());
  }
  if (name.empty()) {
    name = fmt::format("Subgraph{}", subgraphs_.size());
  }
  Subgraph* subgraph =
      new Subgraph(regions, edges_between_regions, order, h_min, h_max,
                   std::move(name), edge_offsets, this);

  // Add global costs to the subgraph.
  for (double weight : global_time_costs_) {
    subgraph->AddTimeCost(weight);
  }

  if (order > 0) {
    // These costs and constraints rely on the derivative of the trajectory.
    for (const MatrixXd& weight_matrix : global_path_length_costs_) {
      subgraph->AddPathLengthCost(weight_matrix);
    }
    for (const MatrixXd& weight_matrix : global_path_energy_costs_) {
      subgraph->AddPathEnergyCost(weight_matrix);
    }
    for (const auto& [lb, ub] : global_velocity_bounds_) {
      subgraph->AddVelocityBounds(lb, ub);
    }
  }

  for (auto& [lb, ub, derivative_order] : global_nonlinear_derivative_bounds_) {
    if (order >= derivative_order) {
      subgraph->AddNonlinearDerivativeBounds(lb, ub, derivative_order);
    }
  }

  // Add global continuity constraints to the subgraph.
  for (int continuity_order : global_path_continuity_constraints_) {
    if (order >= continuity_order) {
      subgraph->AddPathContinuityConstraints(continuity_order);
    }
  }

  for (int continuity_order : global_continuity_constraints_) {
    if (order >= continuity_order) {
      subgraph->AddContinuityConstraints(continuity_order);
    }
  }

  return *subgraphs_.emplace_back(subgraph);
}

Subgraph& GcsTrajectoryOptimization::AddRegions(const ConvexSets& regions,
                                                int order, double h_min,
                                                double h_max,
                                                std::string name) {
  DRAKE_DEMAND(regions.size() > 0);
  auto [edges_between_regions, edge_offsets] =
      ComputePairwiseIntersections(regions, continuous_revolute_joints());
  return GcsTrajectoryOptimization::AddRegions(regions, edges_between_regions,
                                               order, h_min, h_max,
                                               std::move(name), &edge_offsets);
}

void GcsTrajectoryOptimization::RemoveSubgraph(const Subgraph& subgraph) {
  // Check if the subgraph is in the list of subgraphs.
  if (!std::any_of(subgraphs_.begin(), subgraphs_.end(),
                   [&](const std::unique_ptr<Subgraph>& s) {
                     return s.get() == &subgraph;
                   })) {
    throw std::runtime_error(fmt::format(
        "Subgraph {} is not registered with `this`", subgraph.name()));
  }

  // Remove the underlying edges between subgraphs from the gcs problem.
  for (const std::unique_ptr<EdgesBetweenSubgraphs>& subgraph_edge :
       subgraph_edges_) {
    if (&subgraph_edge->from_subgraph_ == &subgraph ||
        &subgraph_edge->to_subgraph_ == &subgraph) {
      for (Edge* edge : subgraph_edge->edges_) {
        gcs_.RemoveEdge(edge);
      }
    }
  }

  // Remove the edges between subgraphs objects.
  subgraph_edges_.erase(
      std::remove_if(subgraph_edges_.begin(), subgraph_edges_.end(),
                     [&](const std::unique_ptr<EdgesBetweenSubgraphs>& e) {
                       return &e->from_subgraph_ == &subgraph ||
                              &e->to_subgraph_ == &subgraph;
                     }),
      subgraph_edges_.end());

  // Remove all vertices in the subgraph.
  for (Vertex* v : subgraph.vertices_) {
    // This will also remove all edges connected to the vertex.
    gcs_.RemoveVertex(v);
  }

  // Remove the subgraph from the list of subgraphs.
  subgraphs_.erase(std::remove_if(subgraphs_.begin(), subgraphs_.end(),
                                  [&](const std::unique_ptr<Subgraph>& s) {
                                    return s.get() == &subgraph;
                                  }),
                   subgraphs_.end());
}

EdgesBetweenSubgraphs& GcsTrajectoryOptimization::AddEdges(
    const Subgraph& from_subgraph, const Subgraph& to_subgraph,
    const ConvexSet* subspace,
    const std::vector<std::pair<int, int>>* edges_between_regions,
    const std::vector<Eigen::VectorXd>* edge_offsets) {
  EdgesBetweenSubgraphs* subgraph_edge =
      new EdgesBetweenSubgraphs(from_subgraph, to_subgraph, subspace, this,
                                edges_between_regions, edge_offsets);

  // Add global continuity constraints to the edges between subgraphs.
  for (int continuity_order : global_path_continuity_constraints_) {
    if (subgraph_edge->from_subgraph_.order() >= continuity_order &&
        subgraph_edge->to_subgraph_.order() >= continuity_order) {
      subgraph_edge->AddPathContinuityConstraints(continuity_order);
    }
  }

  for (int continuity_order : global_continuity_constraints_) {
    if (subgraph_edge->from_subgraph_.order() >= continuity_order &&
        subgraph_edge->to_subgraph_.order() >= continuity_order) {
      subgraph_edge->AddContinuityConstraints(continuity_order);
    }
  }

  return *subgraph_edges_.emplace_back(subgraph_edge);
}

void GcsTrajectoryOptimization::AddTimeCost(double weight) {
  // Add time cost to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    subgraph->AddTimeCost(weight);
  }
  global_time_costs_.push_back(weight);
}

void GcsTrajectoryOptimization::AddPathLengthCost(
    const MatrixXd& weight_matrix) {
  // Add path length cost to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() > 0) {
      subgraph->AddPathLengthCost(weight_matrix);
    }
  }
  global_path_length_costs_.push_back(weight_matrix);
}

void GcsTrajectoryOptimization::AddPathEnergyCost(
    const MatrixXd& weight_matrix) {
  // Add path energy cost to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() > 0) {
      subgraph->AddPathEnergyCost(weight_matrix);
    }
  }
  global_path_energy_costs_.push_back(weight_matrix);
}

void GcsTrajectoryOptimization::AddPathLengthCost(double weight) {
  const MatrixXd weight_matrix =
      weight * MatrixXd::Identity(num_positions(), num_positions());
  return GcsTrajectoryOptimization::AddPathLengthCost(weight_matrix);
}

void GcsTrajectoryOptimization::AddPathEnergyCost(double weight) {
  const MatrixXd weight_matrix =
      weight * 2 * MatrixXd::Identity(num_positions(), num_positions());
  return GcsTrajectoryOptimization::AddPathEnergyCost(weight * weight_matrix);
}

void GcsTrajectoryOptimization::AddVelocityBounds(
    const Eigen::Ref<const VectorXd>& lb,
    const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());
  // Add path velocity bounds to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() > 0) {
      subgraph->AddVelocityBounds(lb, ub);
    }
  }
  global_velocity_bounds_.push_back({lb, ub});
}

void GcsTrajectoryOptimization::AddNonlinearDerivativeBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub, int derivative_order) {
  DRAKE_THROW_UNLESS(lb.size() == num_positions());
  DRAKE_THROW_UNLESS(ub.size() == num_positions());
  if (derivative_order == 1) {
    throw std::runtime_error(
        "Use AddVelocityBounds instead of AddNonlinearDerivativeBounds with "
        "derivative_order=1; velocity bounds are linear.");
  }
  if (derivative_order < 1) {
    throw std::runtime_error("Derivative order must be greater than 1.");
  }

  // Add nonlinear derivative bounds to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() >= derivative_order) {
      subgraph->AddNonlinearDerivativeBounds(lb, ub, derivative_order);
    }
  }
  global_nonlinear_derivative_bounds_.push_back({lb, ub, derivative_order});
}

void GcsTrajectoryOptimization::AddPathContinuityConstraints(
    int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }
  // Add continuity constraints to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() >= continuity_order) {
      subgraph->AddPathContinuityConstraints(continuity_order);
    }
  }
  // Add continuity constraints to the edges between subgraphs.
  for (std::unique_ptr<EdgesBetweenSubgraphs>& subgraph_edge :
       subgraph_edges_) {
    if (subgraph_edge->from_subgraph_.order() >= continuity_order &&
        subgraph_edge->to_subgraph_.order() >= continuity_order) {
      subgraph_edge->AddPathContinuityConstraints(continuity_order);
    }
  }

  global_path_continuity_constraints_.push_back(continuity_order);
}

void GcsTrajectoryOptimization::AddContinuityConstraints(int continuity_order) {
  if (continuity_order == 0) {
    throw std::runtime_error(
        "Path continuity is enforced by default. Choose a higher order.");
  }
  if (continuity_order < 1) {
    throw std::runtime_error("Order must be greater than or equal to 1.");
  }
  // Add continuity constraints to each subgraph.
  for (std::unique_ptr<Subgraph>& subgraph : subgraphs_) {
    if (subgraph->order() >= continuity_order) {
      subgraph->AddContinuityConstraints(continuity_order);
    }
  }
  // Add continuity constraints to the edges between subgraphs.
  for (std::unique_ptr<EdgesBetweenSubgraphs>& subgraph_edge :
       subgraph_edges_) {
    if (subgraph_edge->from_subgraph_.order() >= continuity_order &&
        subgraph_edge->to_subgraph_.order() >= continuity_order) {
      subgraph_edge->AddContinuityConstraints(continuity_order);
    }
  }

  global_continuity_constraints_.push_back(continuity_order);
}

std::pair<CompositeTrajectory<double>, solvers::MathematicalProgramResult>
GcsTrajectoryOptimization::SolvePath(
    const Subgraph& source, const Subgraph& target,
    const GraphOfConvexSetsOptions& specified_options) {
  // Fill in default options. Note: if these options change, they must also be
  // updated in the method documentation.
  GraphOfConvexSetsOptions options = specified_options;
  if (!options.convex_relaxation) {
    options.convex_relaxation = true;
  }
  if (!options.preprocessing) {
    options.preprocessing = true;
  }
  if (!options.max_rounded_paths) {
    options.max_rounded_paths = 5;
  }

  const VectorXd empty_vector;

  Vertex* source_vertex = source.vertices_[0];
  Vertex* dummy_source = nullptr;

  Vertex* target_vertex = target.vertices_[0];
  Vertex* dummy_target = nullptr;

  if (source.size() != 1) {
    // Source subgraph has more than one region. Add a dummy source vertex.
    dummy_source = gcs_.AddVertex(Point(empty_vector), "Dummy source");
    source_vertex = dummy_source;
    for (Vertex* v : source.vertices_) {
      AddEdge(dummy_source, v);
    }
  }
  const ScopeExit cleanup_dummy_source_before_returning([&]() {
    if (dummy_source != nullptr) {
      gcs_.RemoveVertex(dummy_source);
    }
  });

  if (target.size() != 1) {
    // Target subgraph has more than one region. Add a dummy target vertex.
    dummy_target = gcs_.AddVertex(Point(empty_vector), "Dummy target");
    target_vertex = dummy_target;
    for (Vertex* v : target.vertices_) {
      AddEdge(v, dummy_target);
    }
  }
  const ScopeExit cleanup_dummy_target_before_returning([&]() {
    if (dummy_target != nullptr) {
      gcs_.RemoveVertex(dummy_target);
    }
  });

  solvers::MathematicalProgramResult result =
      gcs_.SolveShortestPath(*source_vertex, *target_vertex, options);
  if (!result.is_success()) {
    return {CompositeTrajectory<double>({}), result};
  }

  const double kTolerance = 1.0;  // take any path we can get.
  std::vector<const Edge*> path_edges =
      gcs_.GetSolutionPath(*source_vertex, *target_vertex, result, kTolerance);

  // Remove the dummy edges from the path.
  if (dummy_source != nullptr) {
    DRAKE_DEMAND(!path_edges.empty());
    path_edges.erase(path_edges.begin());
  }
  if (dummy_target != nullptr) {
    DRAKE_DEMAND(!path_edges.empty());
    path_edges.erase(path_edges.end() - 1);
  }

  return {ReconstructTrajectoryFromSolutionPath(path_edges, result), result};
}

std::pair<trajectories::CompositeTrajectory<double>,
          solvers::MathematicalProgramResult>
GcsTrajectoryOptimization::SolveConvexRestriction(
    const std::vector<const Vertex*>& active_vertices,
    const GraphOfConvexSetsOptions& options) {
  DRAKE_DEMAND(active_vertices.size() > 1);

  std::vector<const Edge*> active_edges;
  for (size_t i = 0; i < active_vertices.size() - 1; ++i) {
    bool vertices_connected = false;
    for (const Edge* e : active_vertices[i]->outgoing_edges()) {
      if (e->v().id() == active_vertices[i + 1]->id()) {
        if (vertices_connected) {
          throw std::runtime_error(fmt::format(
              "Vertex: {} is connected to vertex: {} through multiple edges.",
              active_vertices[i]->name(), active_vertices[i + 1]->name()));
        }
        active_edges.push_back(e);
        vertices_connected = true;
      }
    }

    if (!vertices_connected) {
      throw std::runtime_error(fmt::format(
          "Vertex: {} is not connected to vertex: {}.",
          active_vertices[i]->name(), active_vertices[i + 1]->name()));
    }
  }

  solvers::MathematicalProgramResult result =
      gcs_.SolveConvexRestriction(active_edges, options);
  if (!result.is_success()) {
    return {CompositeTrajectory<double>({}), result};
  }

  return {ReconstructTrajectoryFromSolutionPath(active_edges, result), result};
}

CompositeTrajectory<double>
GcsTrajectoryOptimization::ReconstructTrajectoryFromSolutionPath(
    std::vector<const Edge*> edges,
    const solvers::MathematicalProgramResult& result) {
  // Extract the path from the edges.
  std::vector<copyable_unique_ptr<Trajectory<double>>> bezier_curves;
  for (const Edge* e : edges) {
    // Extract phi from the solution to rescale the control points and duration
    // in case we get the relaxed solution.
    const double phi_inv = 1 / result.GetSolution(e->phi());
    // Extract the control points from the solution.
    const int num_control_points = vertex_to_subgraph_[&e->u()]->order() + 1;
    const MatrixX<double> edge_path_points =
        phi_inv *
        Eigen::Map<MatrixX<double>>(result.GetSolution(e->xu()).data(),
                                    num_positions(), num_control_points);

    // Extract the duration from the solution.
    double h = phi_inv * result.GetSolution(e->xu()).tail<1>().value();
    const double start_time =
        bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();

    // Skip edges with a single control point that spend near zero time in the
    // region, since zero order continuity constraint is sufficient. These edges
    // would result in a discontinuous trajectory for velocities and higher
    // derivatives.
    if (num_control_points > 1 &&
        h < PiecewiseTrajectory<double>::kEpsilonTime) {
      throw std::runtime_error(
          fmt::format("GcsTrajectoryOptimization returned a trajectory segment "
                      "with near-zero duration. Make sure you set h_min to be "
                      "at least {} for regions whose subgraph order is at "
                      "least 1 or impose velocity limits.",
                      PiecewiseTrajectory<double>::kEpsilonTime));
    } else if (!(num_control_points == 1 &&
                 vertex_to_subgraph_[&e->u()]->h_min_ == 0)) {
      bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
          start_time, start_time + h, edge_path_points));
    }
  }

  // Get the final control points from the solution.
  const Edge& last_edge = *edges.back();
  const double phi_inv = 1 / result.GetSolution(last_edge.phi());
  const int num_control_points =
      vertex_to_subgraph_[&last_edge.v()]->order() + 1;
  const MatrixX<double> edge_path_points =
      phi_inv *
      Eigen::Map<MatrixX<double>>(result.GetSolution(last_edge.xv()).data(),
                                  num_positions(), num_control_points);

  double h = phi_inv * result.GetSolution(last_edge.xv()).tail<1>().value();
  const double start_time =
      bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();

  // Skip edges with a single control point that spend near zero time in the
  // region, since zero order continuity constraint is sufficient.
  if (num_control_points > 1 && h < PiecewiseTrajectory<double>::kEpsilonTime) {
    throw std::runtime_error(fmt::format(
        "GcsTrajectoryOptimization returned a trajectory segment with "
        "near-zero duration. Make sure you set h_min to be at least {} for "
        "regions whose subgraph order is at least 1 or impose velocity limits.",
        PiecewiseTrajectory<double>::kEpsilonTime));
  } else if (!(num_control_points == 1 &&
               vertex_to_subgraph_[&last_edge.v()]->h_min_ == 0)) {
    bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
        start_time, start_time + h, edge_path_points));
  }
  return CompositeTrajectory<double>(bezier_curves);
}

Edge* GcsTrajectoryOptimization::AddEdge(Vertex* u, Vertex* v) {
  return gcs_.AddEdge(u, v, fmt::format("{} -> {}", u->name(), v->name()));
}

double GcsTrajectoryOptimization::EstimateComplexity() const {
  double result = 0;
  // TODO(ggould) A more correct computation estimate would be:
  // If each vertex and edge problem were solved only once, the cost would be
  // | constraint_var_sum = variables per constraint summed over constraints
  // | cost_vars = total unique variables over all costs
  // | constraint_vars = total unique variables over all constraints
  // : constraint_var_sum * cost_vars * constraint_vars^2
  // In fact each vertex must be solved at least as many times as it has
  // edges, so multiply the vertex cost by the vertex's arity.
  for (const auto* v : gcs_.Vertices()) {
    for (const auto& c : v->GetCosts()) {
      result += c.GetNumElements();
    }
    for (const auto& c : v->GetConstraints()) {
      result += c.GetNumElements();
    }
  }
  for (const auto* e : gcs_.Edges()) {
    for (const auto& c : e->GetCosts()) {
      result += c.GetNumElements();
    }
    for (const auto& c : e->GetConstraints()) {
      result += c.GetNumElements();
    }
  }
  return result;
}

std::vector<Subgraph*> GcsTrajectoryOptimization::GetSubgraphs() const {
  std::vector<Subgraph*> subgraphs;
  subgraphs.reserve(subgraphs_.size());
  for (const auto& subgraph : subgraphs_) {
    subgraphs.push_back(subgraph.get());
  }
  return subgraphs;
}

std::vector<EdgesBetweenSubgraphs*>
GcsTrajectoryOptimization::GetEdgesBetweenSubgraphs() const {
  std::vector<EdgesBetweenSubgraphs*> subgraph_edges;
  subgraph_edges.reserve(subgraph_edges_.size());
  for (const auto& edge : subgraph_edges_) {
    subgraph_edges.push_back(edge.get());
  }
  return subgraph_edges;
}

trajectories::CompositeTrajectory<double>
GcsTrajectoryOptimization::NormalizeSegmentTimes(
    const trajectories::CompositeTrajectory<double>& trajectory) {
  std::vector<copyable_unique_ptr<Trajectory<double>>> normalized_bezier_curves;

  double start_time = trajectory.start_time();
  for (int i = 0; i < trajectory.get_number_of_segments(); ++i) {
    // Create a new BezierCurve with the same control points, but with a
    // duration of one second.
    if (const BezierCurve<double>* gcs_segment =
            dynamic_cast<const BezierCurve<double>*>(&trajectory.segment(i))) {
      normalized_bezier_curves.emplace_back(
          std::make_unique<BezierCurve<double>>(start_time, start_time + 1.0,
                                                gcs_segment->control_points()));
      start_time += 1.0;
    } else {
      throw std::runtime_error(
          "NormalizeSegmentTimes: All segments in the gcs trajectory "
          "must be of type "
          "BezierCurve<double>.");
    }
  }
  return CompositeTrajectory<double>(normalized_bezier_curves);
}

namespace {
bool IsMultipleOf2Pi(double value, double tol = 1e-8) {
  // We allow some tolerance for trajectories coming out of GCS.
  return std::abs(value - 2 * M_PI * std::round(value / (2 * M_PI))) < tol;
}

// Unwrap the angle to the range [2π * round, 2π * (round+1)).
double UnwrapAngle(const double angle, const int round) {
  const int twopi_factors_to_remove =
      static_cast<int>(std::floor(angle / (2 * M_PI))) - round;
  return angle - 2 * M_PI * twopi_factors_to_remove;
}
}  // namespace

trajectories::CompositeTrajectory<double>
GcsTrajectoryOptimization::UnwrapToContinuousTrajectory(
    const trajectories::CompositeTrajectory<double>& gcs_trajectory,
    std::vector<int> continuous_revolute_joints,
    std::optional<std::vector<int>> starting_rounds, double tol) {
  if (starting_rounds.has_value()) {
    DRAKE_THROW_UNLESS(starting_rounds->size() ==
                       continuous_revolute_joints.size());
  }
  // TODO(@anyone): make this a unique_ptr and use std::move to avoid copying.
  std::vector<copyable_unique_ptr<Trajectory<double>>> unwrapped_trajectories;
  int dim = gcs_trajectory.rows();
  ThrowsForInvalidContinuousJointsList(dim, continuous_revolute_joints);
  Eigen::VectorXd last_segment_finish;
  for (int i = 0; i < gcs_trajectory.get_number_of_segments(); ++i) {
    const auto& traj_segment = gcs_trajectory.segment(i);
    const auto* bezier_segment =
        dynamic_cast<const BezierCurve<double>*>(&traj_segment);
    if (bezier_segment == nullptr) {
      throw std::runtime_error(
          "UnwrapToContinuousTrajectory: All segments in the gcs_trajectory "
          "must be of type "
          "BezierCurve<double>.");
    }
    Eigen::MatrixXd new_control_points = bezier_segment->control_points();
    const Eigen::MatrixXd& old_control_points =
        bezier_segment->control_points();
    const Eigen::VectorXd& old_start = old_control_points.col(0);
    std::vector<double> shift;
    if (i == 0) {
      // there is no shift from previous segment.
      if (starting_rounds.has_value()) {
        for (int j = 0; j < ssize(continuous_revolute_joints); ++j) {
          const int joint_index = continuous_revolute_joints.at(j);
          const int start_round = starting_rounds->at(j);
          // This value will be subtracted from the old_start to get the
          // shift.
          const double joint_shift =
              old_start(joint_index) -
              UnwrapAngle(old_start(joint_index), start_round);
          DRAKE_DEMAND(IsMultipleOf2Pi(joint_shift, tol));
          shift.push_back(joint_shift);
        }
      } else {
        shift = std::vector<double>(ssize(continuous_revolute_joints), 0);
      }
    } else {
      DRAKE_DEMAND(last_segment_finish.rows() == gcs_trajectory.rows());
      // See how much shift is needed to match the previous new segment.
      for (const int joint_index : continuous_revolute_joints) {
        const double joint_shift =
            old_start(joint_index) - last_segment_finish(joint_index);
        if (!IsMultipleOf2Pi(joint_shift, tol)) {
          throw std::runtime_error(
              fmt::format("UnwrapToContinuousTrajectory: The shift from "
                          "previous segment: {} is not a multiple "
                          "of 2π at segment {}, joint {}.",
                          joint_shift, i, joint_index));
        }
        shift.push_back(joint_shift);
      }
    }
    for (int j = 0; j < ssize(continuous_revolute_joints); ++j) {
      const int joint_index = continuous_revolute_joints[j];
      // Shift all the columns of the control points by the shift.
      new_control_points.row(joint_index) -=
          Eigen::VectorXd::Constant(old_control_points.cols(), shift.at(j));
    }
    last_segment_finish = new_control_points.rightCols(1);
    unwrapped_trajectories.emplace_back(std::make_unique<BezierCurve<double>>(
        bezier_segment->start_time(), bezier_segment->end_time(),
        new_control_points));
  }
  return CompositeTrajectory<double>(unwrapped_trajectories);
}

std::vector<int> GetContinuousRevoluteJointIndices(
    const multibody::MultibodyPlant<double>& plant) {
  std::vector<int> indices;
  for (JointIndex i : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(i);
    // The first possibility we check for is a revolute joint with no joint
    // limits.
    if (joint.type_name() == RevoluteJoint<double>::kTypeName) {
      if (joint.position_lower_limits()[0] == -kInf &&
          joint.position_upper_limits()[0] == kInf) {
        indices.push_back(joint.position_start());
      }
      continue;
    }
    // The second possibility we check for is a planar joint. If it is (and
    // the angle component has no joint limits), we only add the third entry
    // of the position vector, corresponding to theta.
    if (joint.type_name() == PlanarJoint<double>::kTypeName) {
      if (joint.position_lower_limits()[2] == -kInf &&
          joint.position_upper_limits()[2] == kInf) {
        indices.push_back(joint.position_start() + 2);
      }
      continue;
    }
    // The third possibility we check for is a roll-pitch-yaw floating joint. If
    // it is, we check each of its three revolute components (stored in the
    // first three indices) for unbounded joint limits.
    if (joint.type_name() == RpyFloatingJoint<double>::kTypeName) {
      for (int j = 0; j < 3; ++j) {
        if (joint.position_lower_limits()[j] == -kInf &&
            joint.position_upper_limits()[j] == kInf) {
          indices.push_back(joint.position_start() + j);
        }
      }
      continue;
    }
    // The fourth possibility we check for is a universal joint. If it is, we
    // check each of its two configuration values for unbounded joint limits
    // (both are revolute).
    if (joint.type_name() == UniversalJoint<double>::kTypeName) {
      for (int j = 0; j < 2; ++j) {
        if (joint.position_lower_limits()[j] == -kInf &&
            joint.position_upper_limits()[j] == kInf) {
          indices.push_back(joint.position_start() + j);
        }
      }
      continue;
    }
    // The fifth possibility we check for is a roll-pitch-yaw ball joint. If it
    // is, we check each of its three configuration values for unbounded joint
    // limits (all are revolute).
    if (joint.type_name() == BallRpyJoint<double>::kTypeName) {
      for (int j = 0; j < 3; ++j) {
        if (joint.position_lower_limits()[j] == -kInf &&
            joint.position_upper_limits()[j] == kInf) {
          indices.push_back(joint.position_start() + j);
        }
      }
      continue;
    }
  }
  return indices;
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
