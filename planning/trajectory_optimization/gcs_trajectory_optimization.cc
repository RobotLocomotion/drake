#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"

#include "iostream"

#include "drake/common/pointer_cast.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

using Subgraph = GCSTrajectoryOptimization::Subgraph;
using SubgraphEdges = GCSTrajectoryOptimization::SubgraphEdges;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using geometry::optimization::CartesianProduct;
using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Point;
using math::EigenToStdVector;
using solvers::Binding;
using solvers::Constraint;
using solvers::Cost;
using solvers::L2NormCost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::PerspectiveQuadraticCost;
using std::numeric_limits;
using symbolic::DecomposeLinearExpressions;
using symbolic::Expression;
using symbolic::MakeMatrixContinuousVariable;
using symbolic::MakeVectorContinuousVariable;
using trajectories::BezierCurve;
using trajectories::CompositeTrajectory;
using trajectories::Trajectory;
using Vertex = geometry::optimization::GraphOfConvexSets::Vertex;
using Edge = geometry::optimization::GraphOfConvexSets::Edge;
using VertexId = geometry::optimization::GraphOfConvexSets::VertexId;
using EdgeId = geometry::optimization::GraphOfConvexSets::EdgeId;

const double inf = std::numeric_limits<double>::infinity();

Subgraph::Subgraph(const ConvexSets& regions,
                   std::vector<std::pair<int, int>>& edges_between_regions,
                   int order, double d_min, double d_max,
                   const std::string& name, GCSTrajectoryOptimization* gcs)
    : regions_(regions), order_(order), name_(name), gcs_(gcs) {
  if (order_ < 0) {
    throw std::runtime_error("Order must be positive.");
  }

  // Make sure all regions have the same ambient dimension.
  for (const auto& region : regions_) {
    DRAKE_DEMAND(region->ambient_dimension() == gcs->num_positions());
  }
  // Make time scaling set.
  HPolyhedron time_scaling_set = HPolyhedron::MakeBox(
      d_min * Eigen::VectorXd::Ones(1), d_max * Eigen::VectorXd::Ones(1));

  // Formulate edge costs and constraints.
  auto u_control =
      MakeMatrixContinuousVariable(gcs_->num_positions(), order_ + 1, "xu");
  auto v_control =
      MakeMatrixContinuousVariable(gcs_->num_positions(), order_ + 1, "xv");

  auto u_control_vars = Eigen::Map<VectorX<symbolic::Variable>>(
      u_control.data(), u_control.size());
  auto v_control_vars = Eigen::Map<VectorX<symbolic::Variable>>(
      v_control.data(), v_control.size());

  u_duration_ = MakeVectorContinuousVariable(1, "Tu");
  auto v_duration = MakeVectorContinuousVariable(1, "Tv");

  u_vars_ = solvers::ConcatenateVariableRefList({u_control_vars, u_duration_});
  auto edge_vars = solvers::ConcatenateVariableRefList(
      {u_control_vars, u_duration_, v_control_vars, v_duration});

  u_r_trajectory_ = BezierCurve<Expression>(0, 1, u_control.cast<Expression>());

  auto v_r_trajectory =
      BezierCurve<Expression>(0, 1, v_control.cast<Expression>());

  // Zeroth order continuity constraints.
  const VectorX<Expression> path_continuity_error =
      v_r_trajectory.control_points().col(0) -
      u_r_trajectory_.control_points().col(order);
  Eigen::MatrixXd M(gcs_->num_positions(), edge_vars.size());
  DecomposeLinearExpressions(path_continuity_error, edge_vars, &M);

  auto path_continuity_constraint = std::make_shared<LinearEqualityConstraint>(
      M, VectorXd::Zero(gcs_->num_positions()));

  // Add Regions with time scaling set.
  for (size_t i = 0; i < regions_.size(); i++) {
    // Assign each control point to a separate set.
    ConvexSets vertex_set;
    for (int j = 0; j < order + 1; j++) {
      vertex_set.emplace_back(*regions_[i]);
    }
    // Add time scaling set.
    vertex_set.emplace_back(time_scaling_set);

    vertices_.emplace_back(gcs_->gcs_.AddVertex(
        CartesianProduct(vertex_set), name + ": " + std::to_string(i)));
  }

  // Connect vertices with edges.
  for (const auto& [u_idx, v_idx] : edges_between_regions) {
    // Add edge.
    Vertex* u = vertices_[u_idx];
    Vertex* v = vertices_[v_idx];
    Edge* uv_edge = gcs_->gcs_.AddEdge(*u, *v, u->name() + " -> " + v->name());

    edges_.emplace_back(uv_edge);

    // Add path continuity constraints.
    uv_edge->AddConstraint(
        Binding<Constraint>(path_continuity_constraint, {u->x(), v->x()}));
  }

  if (order_ > 0) {
    // This cost rely on the derivative of the trajectory.
    for (auto weight_matrix : gcs_->global_path_length_costs_) {
      Subgraph::AddPathLengthCost(weight_matrix);
    }
  }
}

void Subgraph::AddPathLengthCost(const Eigen::MatrixXd& weight_matrix) {
  /*
    We will upper bound the path integral by the sum of the distances between
    the control points. ∑ ||rᵢ − rᵢ₊₁||₂

    In the case of a Bezier curve, the path length is given by the integral of
    the norm of the derivative of the curve.

    So the path length cost becomes: ∑ ||ṙᵢ||₂ / order
  */
  DRAKE_DEMAND(weight_matrix.rows() == gcs_->num_positions());
  DRAKE_DEMAND(weight_matrix.cols() == gcs_->num_positions());

  if (order() == 0) {
    throw std::runtime_error(
        "Path length cost is not defined for a set of order 0.");
  }

  auto u_rdot_control =
      dynamic_pointer_cast_or_throw<BezierCurve<symbolic::Expression>>(
          u_r_trajectory_.MakeDerivative())
          ->control_points();

  for (int i = 0; i < u_rdot_control.cols(); i++) {
    Eigen::MatrixXd M(gcs_->num_positions(), u_vars_.size());
    DecomposeLinearExpressions(u_rdot_control.col(i) / order(), u_vars_, &M);

    auto path_length_cost = std::make_shared<L2NormCost>(
        weight_matrix * M, Eigen::VectorXd::Zero(gcs_->num_positions()));

    for (const auto& v : vertices_) {
      // The duration variable is the last element of the vertex.
      v->AddCost(Binding<L2NormCost>(path_length_cost, v->x()));
    }
  }
}

void Subgraph::AddPathLengthCost(double weight) {
  auto weight_matrix =
      weight *
      Eigen::MatrixXd::Identity(gcs_->num_positions(), gcs_->num_positions());
  return Subgraph::AddPathLengthCost(weight_matrix);
}

SubgraphEdges::SubgraphEdges(const Subgraph* from, const Subgraph* to,
                             const ConvexSet* subspace,
                             GCSTrajectoryOptimization* gcs)
    : from_(from), to_(to), gcs_(gcs) {
  // Formulate edge costs and constraints.
  if (subspace != nullptr) {
    if (subspace->ambient_dimension() != gcs_->num_positions()) {
      throw std::runtime_error(
          "Subspace dimension must match the number of positions.");
    }
    if (typeid(*subspace) != typeid(Point) &&
        typeid(*subspace) != typeid(HPolyhedron)) {
      throw std::runtime_error("Subspace must be a Point or HPolyhedron.");
    }
  }

  auto u_control = MakeMatrixContinuousVariable(gcs_->num_positions(),
                                                from->order() + 1, "xu");
  auto v_control = MakeMatrixContinuousVariable(gcs_->num_positions(),
                                                to->order() + 1, "xv");

  auto u_control_vars = Eigen::Map<VectorX<symbolic::Variable>>(
      u_control.data(), u_control.size());
  auto v_control_vars = Eigen::Map<VectorX<symbolic::Variable>>(
      v_control.data(), v_control.size());

  u_duration_ = MakeVectorContinuousVariable(1, "Tu");
  v_duration_ = MakeVectorContinuousVariable(1, "Tv");

  u_vars_ = solvers::ConcatenateVariableRefList({u_control_vars, u_duration_});
  v_vars_ = solvers::ConcatenateVariableRefList({v_control_vars, v_duration_});
  auto edge_vars = solvers::ConcatenateVariableRefList(
      {u_control_vars, u_duration_, v_control_vars, v_duration_});

  u_r_trajectory_ = BezierCurve<Expression>(0, 1, u_control.cast<Expression>());

  v_r_trajectory_ = BezierCurve<Expression>(0, 1, v_control.cast<Expression>());

  // Zeroth order continuity constraints.
  const VectorX<Expression> path_continuity_error =
      v_r_trajectory_.control_points().col(0) -
      u_r_trajectory_.control_points().col(from->order());
  Eigen::MatrixXd M(gcs_->num_positions(), edge_vars.size());
  DecomposeLinearExpressions(path_continuity_error, edge_vars, &M);

  auto path_continuity_constraint = std::make_shared<LinearEqualityConstraint>(
      M, VectorXd::Zero(gcs_->num_positions()));

  // TODO(wrangelvid) this can be parallelized.
  for (int i = 0; i < from->size(); i++) {
    for (int j = 0; j < to->size(); j++) {
      if (from->regions()[i]->IntersectsWith(*to->regions()[j])) {
        if (subspace != nullptr) {
          // Check if the regions are connected through the subspace.
          if (!RegionsConnectThroughSubspace(*from->regions()[i],
                                             *to->regions()[j], *subspace)) {
            continue;
          }
        }
        // Add edge.
        Vertex* u = from->vertices()[i];
        Vertex* v = to->vertices()[j];
        Edge* uv_edge =
            gcs_->gcs_.AddEdge(*u, *v, u->name() + " -> " + v->name());

        edges_.emplace_back(uv_edge);

        // Add path continuity constraints.
        uv_edge->AddConstraint(Binding<LinearEqualityConstraint>(
            path_continuity_constraint, {u->x(), v->x()}));

        if (subspace != nullptr) {
          // Add subspace constraints to the first control point of the v
          // vertex. Since we are using zeroth order continuity, the last
          // control point
          auto vars = v->x().segment(0, gcs_->num_positions());
          solvers::MathematicalProgram prog{};
          const auto& x =
              prog.NewContinuousVariables(gcs_->num_positions(), "x");
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

bool SubgraphEdges::RegionsConnectThroughSubspace(const ConvexSet& A,
                                                  const ConvexSet& B,
                                                  const ConvexSet& subspace) {
  DRAKE_THROW_UNLESS(A.ambient_dimension() == B.ambient_dimension() &&
                     A.ambient_dimension() == subspace.ambient_dimension());
  if (typeid(subspace) == typeid(Point)) {
    // If the subspace is a point, then the point must be in both A and B.
    return A.PointInSet(static_cast<const Point&>(subspace).x()) &&
           B.PointInSet(static_cast<const Point&>(subspace).x());
  } else {
    // Otherwise, we can formulate a problem to check if a point is contained in
    // A, B and the subspace.
    solvers::MathematicalProgram prog{};
    const auto& x = prog.NewContinuousVariables(gcs_->num_positions(), "x");
    A.AddPointInSetConstraints(&prog, x);
    B.AddPointInSetConstraints(&prog, x);
    subspace.AddPointInSetConstraints(&prog, x);
    solvers::MathematicalProgramResult result = solvers::Solve(prog);
    return result.is_success();
  }
}

GCSTrajectoryOptimization::GCSTrajectoryOptimization(int positions) {
  if (positions < 1) {
    throw std::runtime_error("Dimension must be greater than 0.");
  }
  positions_ = positions;
}

Subgraph& GCSTrajectoryOptimization::AddRegions(
    const geometry::optimization::ConvexSets& regions,
    std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double d_min, double d_max, std::string name) {
  return *subgraphs_
              .emplace_back(std::unique_ptr<Subgraph>(
                  new Subgraph(regions, edges_between_regions, order, d_min,
                               d_max, name, this)))
              .get();
}

Subgraph& GCSTrajectoryOptimization::AddRegions(
    const geometry::optimization::ConvexSets& regions, int order, double d_min,
    double d_max, std::string name) {
  if (name.empty()) {
    name = fmt::format("S{}", subgraphs_.size());
  }
  // TODO(wrangelvid): This is O(n^2) and can be improved.
  std::vector<std::pair<int, int>> edges_between_regions;
  for (size_t i = 0; i < regions.size(); i++) {
    for (size_t j = i + 1; j < regions.size(); j++) {
      if (regions[i]->IntersectsWith(*regions[j])) {
        // Regions are overlapping, add edge.
        edges_between_regions.emplace_back(i, j);
        edges_between_regions.emplace_back(j, i);
      }
    }
  }

  return GCSTrajectoryOptimization::AddRegions(regions, edges_between_regions,
                                               order, d_min, d_max, name);
}

SubgraphEdges& GCSTrajectoryOptimization::AddEdges(
    const Subgraph* from, const Subgraph* to,
    const geometry::optimization::ConvexSet* subspace) {
  return *subgraph_edges_
              .emplace_back(std::unique_ptr<SubgraphEdges>(
                  new SubgraphEdges(from, to, subspace, this)))
              .get();
}

void GCSTrajectoryOptimization::AddPathLengthCost(
    const Eigen::MatrixXd& weight_matrix) {
  // Add path length cost to each subgraph.
  for (auto& subgraph : subgraphs_) {
    if (subgraph->order() > 0) {
      subgraph->AddPathLengthCost(weight_matrix);
    }
  }
  global_path_length_costs_.push_back(weight_matrix);
}

void GCSTrajectoryOptimization::AddPathLengthCost(double weight) {
  auto weight_matrix =
      weight * Eigen::MatrixXd::Identity(num_positions(), num_positions());
  return GCSTrajectoryOptimization::AddPathLengthCost(weight_matrix);
};

std::pair<CompositeTrajectory<double>, solvers::MathematicalProgramResult>
GCSTrajectoryOptimization::SolvePath(Subgraph& source, Subgraph& target,
                                     const GraphOfConvexSetsOptions& options) {
  Eigen::VectorXd empty_vector;
  VertexId source_id = source.vertices()[0]->id();
  Vertex* dummy_source = nullptr;

  VertexId target_id = target.vertices()[0]->id();
  Vertex* dummy_target = nullptr;

  if (source.size() != 1) {
    // Source subgraph has more than one region. Add a dummy source vertex.
    dummy_source = gcs_.AddVertex(Point(empty_vector), "Dummy Source");
    source_id = dummy_source->id();
    for (const auto& v : source.vertices()) {
      gcs_.AddEdge(*dummy_source, *v,
                   dummy_source->name() + " -> " + v->name());
    }
  }
  if (target.size() != 1) {
    // Target subgraph has more than one region. Add a dummy target vertex.
    dummy_target = gcs_.AddVertex(Point(empty_vector), "Dummy target");
    target_id = dummy_target->id();
    for (const auto& v : target.vertices()) {
      gcs_.AddEdge(*v, *dummy_target,
                   v->name() + " -> " + dummy_target->name());
    }
  }

  solvers::MathematicalProgramResult result =
      gcs_.SolveShortestPath(source_id, target_id, options);

  if (!result.is_success()) {
    if (dummy_source != nullptr) {
      gcs_.RemoveVertex(dummy_source->id());
    }

    if (dummy_target != nullptr) {
      gcs_.RemoveVertex(dummy_target->id());
    }
    return {CompositeTrajectory<double>({}), result};
  }

  // Extract the flow from the solution.
  std::map<VertexId, std::vector<Edge*>> outgoing_edges;
  std::map<EdgeId, double> flows;
  for (auto& edge : gcs_.Edges()) {
    outgoing_edges[edge->u().id()].push_back(edge);
    flows[edge->id()] = result.GetSolution(edge->phi());
  }

  // Extract the path by traversing the graph with a depth first search.
  std::vector<VertexId> visited_vertex_ids{source_id};
  std::vector<VertexId> path_vertex_ids{source_id};
  std::vector<Edge*> path_edges{};
  while (path_vertex_ids.back() != target_id) {
    // Find the edge with the maximum flow from the current node.
    double maximum_flow = 0;
    VertexId max_flow_vertex_id;
    Edge* max_flow_edge = nullptr;
    for (Edge* e : outgoing_edges[path_vertex_ids.back()]) {
      double next_flow = flows[e->id()];
      VertexId next_vertex_id = e->v().id();

      // If the edge has not been visited and has a flow greater than the
      // current maximum, update the maximum flow and the vertex id.
      if (std::find(visited_vertex_ids.begin(), visited_vertex_ids.end(),
                    e->v().id()) == visited_vertex_ids.end() &&
          next_flow > maximum_flow && next_flow > options.flow_tolerance) {
        maximum_flow = next_flow;
        max_flow_vertex_id = next_vertex_id;
        max_flow_edge = e;
      }
    }

    if (max_flow_edge == nullptr) {
      // If no candidate edges are found, backtrack to the previous node and
      // continue the search.
      path_vertex_ids.pop_back();
      continue;
    } else {
      // If the maximum flow is non-zero, add the vertex to the path and
      // continue the search.
      visited_vertex_ids.push_back(max_flow_vertex_id);
      path_vertex_ids.push_back(max_flow_vertex_id);
      path_edges.push_back(max_flow_edge);
    }
  }

  // Remove the dummy edges from the path.
  if (dummy_source != nullptr) {
    path_edges.erase(path_edges.begin());
    gcs_.RemoveVertex(dummy_source->id());
  }

  if (dummy_target != nullptr) {
    path_edges.erase(path_edges.end() - 1);
    gcs_.RemoveVertex(dummy_target->id());
  }

  // Extract the path from the edges.
  std::vector<copyable_unique_ptr<Trajectory<double>>> bezier_curves{};
  for (auto& edge : path_edges) {
    // Extract the control points from the solution.
    int num_control_points = (edge->xu().size() - 1) / num_positions();
    MatrixX<double> edge_path_points =
        Eigen::Map<MatrixX<double>>(result.GetSolution(edge->xu()).data(),
                                    num_positions(), num_control_points);

    // Extract the duration from the solution.
    double duration = result.GetSolution(edge->xu()).tail<1>().value();
    double start_time =
        bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();
    bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
        start_time, start_time + duration, edge_path_points));
  }

  // Get the final control points from the solution.
  int num_control_points =
      (path_edges.back()->xv().size() - 1) / num_positions();
  MatrixX<double> edge_path_points = Eigen::Map<MatrixX<double>>(
      result.GetSolution(path_edges.back()->xv()).data(), num_positions(),
      num_control_points);

  double duration =
      result.GetSolution(path_edges.back()->xv()).tail<1>().value();
  double start_time =
      bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();
  bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
      start_time, start_time + duration, edge_path_points));

  return {CompositeTrajectory<double>(bezier_curves), result};
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake