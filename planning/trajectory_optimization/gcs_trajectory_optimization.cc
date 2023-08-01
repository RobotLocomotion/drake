#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"

#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/pointer_cast.h"
#include "drake/common/scope_exit.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/geometry/optimization/point.h"
#include "drake/math/matrix_util.h"
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
using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Intersection;
using geometry::optimization::Point;
using solvers::Binding;
using solvers::ConcatenateVariableRefList;
using solvers::Constraint;
using solvers::Cost;
using solvers::L2NormCost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using symbolic::DecomposeLinearExpressions;
using symbolic::Expression;
using symbolic::MakeMatrixContinuousVariable;
using symbolic::MakeVectorContinuousVariable;
using trajectories::BezierCurve;
using trajectories::CompositeTrajectory;
using trajectories::Trajectory;
using Vertex = GraphOfConvexSets::Vertex;
using Edge = GraphOfConvexSets::Edge;
using VertexId = GraphOfConvexSets::VertexId;
using EdgeId = GraphOfConvexSets::EdgeId;

const double kInf = std::numeric_limits<double>::infinity();

Subgraph::Subgraph(
    const ConvexSets& regions,
    const std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double h_min, double h_max, std::string name,
    GcsTrajectoryOptimization* traj_opt)
    : regions_(regions),
      order_(order),
      h_min_(h_min),
      name_(std::move(name)),
      traj_opt_(*traj_opt) {
  DRAKE_THROW_UNLESS(order >= 0);
  DRAKE_THROW_UNLESS(!regions_.empty());

  // Make sure all regions have the same ambient dimension.
  for (const std::unique_ptr<ConvexSet>& region : regions_) {
    DRAKE_THROW_UNLESS(region != nullptr);
    DRAKE_THROW_UNLESS(region->ambient_dimension() == num_positions());
  }
  // Make time scaling set once to avoid many allocations when adding the
  // vertices to GCS.
  const HPolyhedron time_scaling_set =
      HPolyhedron::MakeBox(Vector1d(h_min), Vector1d(h_max));

  // Add Regions with time scaling set.
  for (size_t i = 0; i < regions_.size(); ++i) {
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
  const auto path_continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(
          A, VectorXd::Zero(num_positions()));

  // Connect vertices with edges.
  for (const auto& [u_index, v_index] : edges_between_regions) {
    // Add edge.
    Vertex* u = vertices_[u_index];
    Vertex* v = vertices_[v_index];
    Edge* uv_edge = traj_opt_.AddEdge(u, v);

    edges_.emplace_back(uv_edge);

    // Add path continuity constraints.
    uv_edge->AddConstraint(Binding<Constraint>(
        path_continuity_constraint,
        {GetControlPoints(*u).col(order), GetControlPoints(*v).col(0)}));
  }
}

Subgraph::~Subgraph() = default;

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

void Subgraph::AddPathLengthCost(double weight) {
  const MatrixXd weight_matrix =
      weight * MatrixXd::Identity(num_positions(), num_positions());
  return Subgraph::AddPathLengthCost(weight_matrix);
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

EdgesBetweenSubgraphs::EdgesBetweenSubgraphs(
    const Subgraph& from_subgraph, const Subgraph& to_subgraph,
    const ConvexSet* subspace, GcsTrajectoryOptimization* traj_opt)
    : traj_opt_(*traj_opt),
      from_subgraph_order_(from_subgraph.order()),
      to_subgraph_order_(to_subgraph.order()) {
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
      0, 1, MatrixXd::Zero(num_positions(), from_subgraph_order_ + 1));
  vr_trajectory_ = BezierCurve<double>(
      0, 1, MatrixXd::Zero(num_positions(), to_subgraph_order_ + 1));

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

  const auto path_continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(
          A, VectorXd::Zero(num_positions()));

  // TODO(wrangelvid) this can be parallelized.
  for (int i = 0; i < from_subgraph.size(); ++i) {
    for (int j = 0; j < to_subgraph.size(); ++j) {
      if (from_subgraph.regions()[i]->IntersectsWith(
              *to_subgraph.regions()[j])) {
        if (subspace != nullptr) {
          // Check if the regions are connected through the subspace.
          if (!RegionsConnectThroughSubspace(*from_subgraph.regions()[i],
                                             *to_subgraph.regions()[j],
                                             *subspace)) {
            continue;
          }
        }

        // Add edge.
        Vertex* u = from_subgraph.vertices_[i];
        Vertex* v = to_subgraph.vertices_[j];
        Edge* uv_edge = traj_opt_.AddEdge(u, v);
        edges_.emplace_back(uv_edge);

        // Add path continuity constraints.
        uv_edge->AddConstraint(Binding<LinearEqualityConstraint>(
            path_continuity_constraint,
            ConcatenateVariableRefList(
                {GetControlPointsU(*uv_edge).col(from_subgraph_order_),
                 GetControlPointsV(*uv_edge).col(0)})));

        if (subspace != nullptr) {
          // Add subspace constraints to the first control point of the v
          // vertex. Since we are using zeroth order continuity, the last
          // control point
          const auto vars = v->x().segment(0, num_positions());
          solvers::MathematicalProgram prog;
          const VectorX<symbolic::Variable> x =
              prog.NewContinuousVariables(num_positions(), "x");
          subspace->AddPointInSetConstraints(&prog, x);
          for (const auto& binding : prog.GetAllConstraints()) {
            const std::shared_ptr<Constraint>& constraint = binding.evaluator();
            uv_edge->AddConstraint(Binding<Constraint>(constraint, vars));
          }
        }
      }
    }
  }
}

EdgesBetweenSubgraphs::~EdgesBetweenSubgraphs() = default;

bool EdgesBetweenSubgraphs::RegionsConnectThroughSubspace(
    const ConvexSet& A, const ConvexSet& B, const ConvexSet& subspace) {
  DRAKE_THROW_UNLESS(A.ambient_dimension() > 0);
  DRAKE_THROW_UNLESS(A.ambient_dimension() == B.ambient_dimension());
  DRAKE_THROW_UNLESS(A.ambient_dimension() == subspace.ambient_dimension());
  if (std::optional<VectorXd> subspace_point = subspace.MaybeGetPoint()) {
    // If the subspace is a point, then the point must be in both A and B.
    return A.PointInSet(*subspace_point) && B.PointInSet(*subspace_point);
  } else {
    // Otherwise, we can formulate a problem to check if a point is contained in
    // A, B and the subspace.
    Intersection intersection(MakeConvexSets(A, B, subspace));
    return !intersection.IsEmpty();
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

  if (from_subgraph_order_ == 0 && to_subgraph_order_ == 0) {
    throw std::runtime_error(
        "Cannot add velocity bounds to a subgraph edges where both subgraphs "
        "have zero order.");
  }

  if (from_subgraph_order_ > 0) {
    // Add velocity bounds to the last control point of the u set.
    // See BezierCurve::AsLinearInControlPoints().
    solvers::VectorXDecisionVariable vars(from_subgraph_order_ + 2);
    SparseMatrix<double> m = ur_trajectory_.AsLinearInControlPoints(1)
                                 .col(from_subgraph_order_ - 1)
                                 .transpose();
    SparseMatrix<double> H_lb(
        1 /* we are only constraining the last point in the u set */,
        from_subgraph_order_ +
            2 /* number of control points for one row of r(s) + 1*/);
    H_lb.leftCols(from_subgraph_order_ + 1) = m;
    SparseMatrix<double> H_ub = H_lb;
    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
      H_lb.coeffRef(0, from_subgraph_order_ + 1) = -lb[i];
      const auto lb_constraint =
          std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
      // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
      H_ub.coeffRef(0, from_subgraph_order_ + 1) = -ub[i];
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

  if (to_subgraph_order_ > 0) {
    // Add velocity bounds to the first control point of the v set.
    // See Subgraph::AddVelocityBounds().
    solvers::VectorXDecisionVariable vars(to_subgraph_order_ + 2);
    SparseMatrix<double> m =
        vr_trajectory_.AsLinearInControlPoints(1).col(0).transpose();
    SparseMatrix<double> H_lb(
        1 /* we are only constraining the last point in the u set */,
        to_subgraph_order_ +
            2 /* number of control points for one row of r(s) + 1*/);
    H_lb.leftCols(to_subgraph_order_ + 1) = m;
    SparseMatrix<double> H_ub = H_lb;
    for (int i = 0; i < num_positions(); ++i) {
      // Lower bound.  0 <= ṙ(s).row(i) - h * lb <= inf.
      H_lb.coeffRef(0, to_subgraph_order_ + 1) = -lb[i];
      const auto lb_constraint =
          std::make_shared<LinearConstraint>(H_lb, kVecZero, kVecInf);
      // Upper bound. -inf <= ṙ(s).row(i) - h * ub <= 0.
      H_ub.coeffRef(0, to_subgraph_order_ + 1) = -ub[i];
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

Eigen::Map<const MatrixX<symbolic::Variable>>
EdgesBetweenSubgraphs::GetControlPointsU(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xu().size() ==
               num_positions() * (from_subgraph_order_ + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      e.xu().data(), num_positions(), from_subgraph_order_ + 1);
}

Eigen::Map<const MatrixX<symbolic::Variable>>
EdgesBetweenSubgraphs::GetControlPointsV(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xv().size() == num_positions() * (to_subgraph_order_ + 1) + 1);
  return Eigen::Map<const MatrixX<symbolic::Variable>>(
      e.xv().data(), num_positions(), to_subgraph_order_ + 1);
}

symbolic::Variable EdgesBetweenSubgraphs::GetTimeScalingU(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xu().size() ==
               num_positions() * (from_subgraph_order_ + 1) + 1);
  return e.xu()(e.xu().size() - 1);
}

symbolic::Variable EdgesBetweenSubgraphs::GetTimeScalingV(
    const geometry::optimization::GraphOfConvexSets::Edge& e) const {
  DRAKE_DEMAND(e.xv().size() == num_positions() * (to_subgraph_order_ + 1) + 1);
  return e.xv()(e.xv().size() - 1);
}

GcsTrajectoryOptimization::GcsTrajectoryOptimization(int num_positions)
    : num_positions_(num_positions) {
  DRAKE_THROW_UNLESS(num_positions >= 1);
}

GcsTrajectoryOptimization::~GcsTrajectoryOptimization() = default;

Subgraph& GcsTrajectoryOptimization::AddRegions(
    const ConvexSets& regions,
    const std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double h_min, double h_max, std::string name) {
  if (name.empty()) {
    name = fmt::format("Subgraph{}", subgraphs_.size());
  }
  Subgraph* subgraph = new Subgraph(regions, edges_between_regions, order,
                                    h_min, h_max, std::move(name), this);

  // Add global costs to the subgraph.
  for (double weight : global_time_costs_) {
    subgraph->AddTimeCost(weight);
  }

  if (order > 0) {
    // These costs and constraints rely on the derivative of the trajectory.
    for (const MatrixXd& weight_matrix : global_path_length_costs_) {
      subgraph->AddPathLengthCost(weight_matrix);
    }
    for (const auto& [lb, ub] : global_velocity_bounds_) {
      subgraph->AddVelocityBounds(lb, ub);
    }
  }
  return *subgraphs_.emplace_back(subgraph);
}

Subgraph& GcsTrajectoryOptimization::AddRegions(const ConvexSets& regions,
                                                int order, double h_min,
                                                double h_max,
                                                std::string name) {
  // TODO(wrangelvid): This is O(n^2) and can be improved.
  std::vector<std::pair<int, int>> edges_between_regions;
  for (size_t i = 0; i < regions.size(); ++i) {
    for (size_t j = i + 1; j < regions.size(); ++j) {
      if (regions[i]->IntersectsWith(*regions[j])) {
        // Regions are overlapping, add edge.
        edges_between_regions.emplace_back(i, j);
        edges_between_regions.emplace_back(j, i);
      }
    }
  }

  return GcsTrajectoryOptimization::AddRegions(
      regions, edges_between_regions, order, h_min, h_max, std::move(name));
}

EdgesBetweenSubgraphs& GcsTrajectoryOptimization::AddEdges(
    const Subgraph& from_subgraph, const Subgraph& to_subgraph,
    const ConvexSet* subspace) {
  return *subgraph_edges_.emplace_back(
      new EdgesBetweenSubgraphs(from_subgraph, to_subgraph, subspace, this));
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

void GcsTrajectoryOptimization::AddPathLengthCost(double weight) {
  const MatrixXd weight_matrix =
      weight * MatrixXd::Identity(num_positions(), num_positions());
  return GcsTrajectoryOptimization::AddPathLengthCost(weight_matrix);
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

  // Extract the path from the edges.
  std::vector<copyable_unique_ptr<Trajectory<double>>> bezier_curves;
  for (const Edge* e : path_edges) {
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
    if (!(num_control_points == 1 &&
          vertex_to_subgraph_[&e->u()]->h_min_ == 0)) {
      bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
          start_time, start_time + h, edge_path_points));
    }
  }

  // Get the final control points from the solution.
  const Edge& last_edge = *path_edges.back();
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
  if (!(num_control_points == 1 &&
        vertex_to_subgraph_[&last_edge.v()]->h_min_ == 0)) {
    bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
        start_time, start_time + h, edge_path_points));
  }

  return {CompositeTrajectory<double>(bezier_curves), result};
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

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
