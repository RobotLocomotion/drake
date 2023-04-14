#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"

#include <unordered_map>
#include <unordered_set>

#include "drake/common/pointer_cast.h"
#include "drake/common/scope_exit.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

using Subgraph = GcsTrajectoryOptimization::Subgraph;
using EdgesBetweenSubgraphs = GcsTrajectoryOptimization::EdgesBetweenSubgraphs;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using geometry::optimization::CartesianProduct;
using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using geometry::optimization::GraphOfConvexSets;
using geometry::optimization::GraphOfConvexSetsOptions;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Point;
using solvers::Binding;
using solvers::Constraint;
using solvers::Cost;
using solvers::L2NormCost;
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

Subgraph::Subgraph(
    const ConvexSets& regions,
    const std::vector<std::pair<int, int>>& edges_between_regions, int order,
    double h_min, double h_max, std::string name,
    GcsTrajectoryOptimization* traj_opt)
    : regions_(regions),
      order_(order),
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

  // Allocating variables and control points to be used in the constraints.
  // Bindings allow formulating the constraints once, and then pass them to all
  // the edges.
  // An edge goes from the vertex u to the vertex v. Where its control points
  // and trajectories are needed for continuity constraints. Saving the
  // variables for u simplifies the cost/constraint formulation for optional
  // costs like the path length cost.
  const MatrixX<symbolic::Variable> u_control =
      MakeMatrixContinuousVariable(num_positions(), order_ + 1, "xu");
  const MatrixX<symbolic::Variable> v_control =
      MakeMatrixContinuousVariable(num_positions(), order_ + 1, "xv");
  const Eigen::Map<const VectorX<symbolic::Variable>> u_control_vars(
      u_control.data(), u_control.size());
  const Eigen::Map<const VectorX<symbolic::Variable>> v_control_vars(
      v_control.data(), v_control.size());

  u_h_ = MakeVectorContinuousVariable(1, "hu");
  const VectorX<symbolic::Variable> v_h = MakeVectorContinuousVariable(1, "hv");

  u_vars_ = solvers::ConcatenateVariableRefList({u_control_vars, u_h_});
  const VectorX<symbolic::Variable> edge_vars =
      solvers::ConcatenateVariableRefList(
          {u_control_vars, u_h_, v_control_vars, v_h});

  u_r_trajectory_ = BezierCurve<Expression>(0, 1, u_control.cast<Expression>());

  const auto v_r_trajectory =
      BezierCurve<Expression>(0, 1, v_control.cast<Expression>());

  // TODO(wrangelvid) Pull this out into a function once we have a better way to
  // extract M from bezier curves.
  const VectorX<Expression> path_continuity_error =
      v_r_trajectory.control_points().col(0) -
      u_r_trajectory_.control_points().col(order);
  MatrixXd M(num_positions(), edge_vars.size());
  DecomposeLinearExpressions(path_continuity_error, edge_vars, &M);
  // TODO(wrangelvid) The matrix M might be sparse, so we could drop columns
  // here.

  const auto path_continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(
          M, VectorXd::Zero(num_positions()));

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
        CartesianProduct(vertex_set), fmt::format("{}: {}", name, i)));
  }

  // Connect vertices with edges.
  for (const auto& [u_index, v_index] : edges_between_regions) {
    // Add edge.
    const Vertex& u = *vertices_[u_index];
    const Vertex& v = *vertices_[v_index];
    Edge* uv_edge = traj_opt_.AddEdge(u, v);

    edges_.emplace_back(uv_edge);

    // Add path continuity constraints.
    uv_edge->AddConstraint(
        Binding<Constraint>(path_continuity_constraint, {u.x(), v.x()}));
  }
}

Subgraph::~Subgraph() = default;

void Subgraph::AddPathLengthCost(const MatrixXd& weight_matrix) {
  /*
    We will upper bound the trajectory length by the sum of the distances
    between the control points. ∑ ||rᵢ − rᵢ₊₁||₂

    In the case of a Bézier curve, the path length is given by the integral of
    the norm of the derivative of the curve.

    So the previous upper bound is equivalent to: ∑ ||ṙᵢ||₂ / order
    Because ||ṙᵢ||₂ = ||rᵢ₊₁ − rᵢ||₂ * order
  */
  DRAKE_THROW_UNLESS(weight_matrix.rows() == num_positions());
  DRAKE_THROW_UNLESS(weight_matrix.cols() == num_positions());

  if (order() == 0) {
    throw std::runtime_error(
        "Path length cost is not defined for a set of order 0.");
  }

  const MatrixX<Expression> u_rdot_control =
      dynamic_pointer_cast_or_throw<BezierCurve<Expression>>(
          u_r_trajectory_.MakeDerivative())
          ->control_points();

  for (int i = 0; i < u_rdot_control.cols(); ++i) {
    MatrixXd M(num_positions(), u_vars_.size());
    DecomposeLinearExpressions(u_rdot_control.col(i) / order(), u_vars_, &M);
    // TODO(wrangelvid) The matrix M might be sparse, so we could drop columns
    // here.

    const auto path_length_cost = std::make_shared<L2NormCost>(
        weight_matrix * M, VectorXd::Zero(num_positions()));

    for (Vertex* v : vertices_) {
      // The duration variable is the last element of the vertex.
      v->AddCost(Binding<L2NormCost>(path_length_cost, v->x()));
    }
  }
}

void Subgraph::AddPathLengthCost(double weight) {
  const MatrixXd weight_matrix =
      weight * MatrixXd::Identity(num_positions(), num_positions());
  return Subgraph::AddPathLengthCost(weight_matrix);
}

EdgesBetweenSubgraphs::EdgesBetweenSubgraphs(
    const Subgraph& from, const Subgraph& to, const ConvexSet* subspace,
    GcsTrajectoryOptimization* traj_opt)
    : traj_opt_(*traj_opt) {
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

  // Allocating variables and control points to be used in the constraints.
  // Bindings allow formulating the constraints once, and then pass
  // them to all the edges.
  // An edge goes from the vertex u to the vertex v. Where its control points
  // and trajectories are needed for continuity constraints. Saving the
  // variables for u simplifies the cost/constraint formulation for optional
  // constraints like the velocity bounds.

  const MatrixX<symbolic::Variable> u_control =
      MakeMatrixContinuousVariable(num_positions(), from.order() + 1, "xu");
  const MatrixX<symbolic::Variable> v_control =
      MakeMatrixContinuousVariable(num_positions(), to.order() + 1, "xv");
  Eigen::Map<const VectorX<symbolic::Variable>> u_control_vars(
      u_control.data(), u_control.size());
  Eigen::Map<const VectorX<symbolic::Variable>> v_control_vars(
      v_control.data(), v_control.size());

  u_h_ = MakeVectorContinuousVariable(1, "Tu");
  v_h_ = MakeVectorContinuousVariable(1, "Tv");

  u_vars_ = solvers::ConcatenateVariableRefList({u_control_vars, u_h_});
  v_vars_ = solvers::ConcatenateVariableRefList({v_control_vars, v_h_});
  const VectorX<symbolic::Variable> edge_vars =
      solvers::ConcatenateVariableRefList(
          {u_control_vars, u_h_, v_control_vars, v_h_});

  u_r_trajectory_ = BezierCurve<Expression>(0, 1, u_control.cast<Expression>());

  v_r_trajectory_ = BezierCurve<Expression>(0, 1, v_control.cast<Expression>());

  // Zeroth order continuity constraints.
  // TODO(wrangelvid) Pull this out into a function once we have a better way to
  // extract M from bezier curves.
  const VectorX<Expression> path_continuity_error =
      v_r_trajectory_.control_points().col(0) -
      u_r_trajectory_.control_points().col(from.order());
  MatrixXd M(num_positions(), edge_vars.size());
  DecomposeLinearExpressions(path_continuity_error, edge_vars, &M);
  // TODO(wrangelvid) The matrix M might be sparse, so we could drop columns
  // here.

  const auto path_continuity_constraint =
      std::make_shared<LinearEqualityConstraint>(
          M, VectorXd::Zero(num_positions()));

  // TODO(wrangelvid) this can be parallelized.
  for (int i = 0; i < from.size(); ++i) {
    for (int j = 0; j < to.size(); ++j) {
      if (from.regions()[i]->IntersectsWith(*to.regions()[j])) {
        if (subspace != nullptr) {
          // Check if the regions are connected through the subspace.
          if (!RegionsConnectThroughSubspace(*from.regions()[i],
                                             *to.regions()[j], *subspace)) {
            continue;
          }
        }

        // Add edge.
        const Vertex& u = *from.vertices_[i];
        const Vertex& v = *to.vertices_[j];
        Edge* uv_edge = traj_opt_.AddEdge(u, v);
        edges_.emplace_back(uv_edge);

        // Add path continuity constraints.
        uv_edge->AddConstraint(Binding<LinearEqualityConstraint>(
            path_continuity_constraint, {u.x(), v.x()}));

        if (subspace != nullptr) {
          // Add subspace constraints to the first control point of the v
          // vertex. Since we are using zeroth order continuity, the last
          // control point
          const auto vars = v.x().segment(0, num_positions());
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
  DRAKE_THROW_UNLESS(A.ambient_dimension() == B.ambient_dimension() &&
                     A.ambient_dimension() == subspace.ambient_dimension());
  // TODO(wrangelvid) Replace dynamic cast with a function that checks if the
  // convex set degenerates to a point.
  if (const Point* pt = dynamic_cast<const Point*>(&subspace)) {
    // If the subspace is a point, then the point must be in both A and B.
    return A.PointInSet(pt->x()) && B.PointInSet(pt->x());
  } else {
    // Otherwise, we can formulate a problem to check if a point is contained in
    // A, B and the subspace.
    solvers::MathematicalProgram prog;
    const VectorX<symbolic::Variable> x =
        prog.NewContinuousVariables(num_positions(), "x");
    A.AddPointInSetConstraints(&prog, x);
    B.AddPointInSetConstraints(&prog, x);
    subspace.AddPointInSetConstraints(&prog, x);
    solvers::MathematicalProgramResult result = solvers::Solve(prog);
    return result.is_success();
  }
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
  Subgraph* subgraph = new Subgraph(regions, edges_between_regions, order,
                                    h_min, h_max, std::move(name), this);

  // Add global costs to the subgraph.
  if (order > 0) {
    // These costs rely on the derivative of the trajectory.
    for (const MatrixXd& weight_matrix : global_path_length_costs_) {
      subgraph->AddPathLengthCost(weight_matrix);
    }
  }
  return *subgraphs_.emplace_back(subgraph);
}

Subgraph& GcsTrajectoryOptimization::AddRegions(const ConvexSets& regions,
                                                int order, double h_min,
                                                double h_max,
                                                std::string name) {
  if (name.empty()) {
    name = fmt::format("S{}", subgraphs_.size());
  }
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
    const Subgraph& from, const Subgraph& to, const ConvexSet* subspace) {
  return *subgraph_edges_.emplace_back(
      new EdgesBetweenSubgraphs(from, to, subspace, this));
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

std::pair<CompositeTrajectory<double>, solvers::MathematicalProgramResult>
GcsTrajectoryOptimization::SolvePath(const Subgraph& source,
                                     const Subgraph& target,
                                     const GraphOfConvexSetsOptions& options) {
  const VectorXd empty_vector;

  VertexId source_id = source.vertices_[0]->id();
  Vertex* dummy_source = nullptr;

  VertexId target_id = target.vertices_[0]->id();
  Vertex* dummy_target = nullptr;

  if (source.size() != 1) {
    // Source subgraph has more than one region. Add a dummy source vertex.
    dummy_source = gcs_.AddVertex(Point(empty_vector), "Dummy source");
    source_id = dummy_source->id();
    for (const Vertex* v : source.vertices_) {
      AddEdge(*dummy_source, *v);
    }
  }
  const ScopeExit cleanup_dummy_source_before_returning([&]() {
    if (dummy_source != nullptr) {
      gcs_.RemoveVertex(dummy_source->id());
    }
  });

  if (target.size() != 1) {
    // Target subgraph has more than one region. Add a dummy target vertex.
    dummy_target = gcs_.AddVertex(Point(empty_vector), "Dummy target");
    target_id = dummy_target->id();
    for (const Vertex* v : target.vertices_) {
      AddEdge(*v, *dummy_target);
    }
  }
  const ScopeExit cleanup_dummy_target_before_returning([&]() {
    if (dummy_target != nullptr) {
      gcs_.RemoveVertex(dummy_target->id());
    }
  });

  solvers::MathematicalProgramResult result =
      gcs_.SolveShortestPath(source_id, target_id, options);
  if (!result.is_success()) {
    return {CompositeTrajectory<double>({}), result};
  }

  // Extract the flow from the solution.
  std::unordered_map<VertexId, std::vector<Edge*>> outgoing_edges;
  std::unordered_map<EdgeId, double> flows;
  for (Edge* edge : gcs_.Edges()) {
    outgoing_edges[edge->u().id()].push_back(edge);
    flows[edge->id()] = result.GetSolution(edge->phi());
  }

  // Extract the path by traversing the graph with a depth first search.
  std::unordered_set<VertexId> visited_vertex_ids{source_id};
  std::vector<VertexId> path_vertex_ids{source_id};
  std::vector<Edge*> path_edges;
  while (path_vertex_ids.back() != target_id) {
    // Find the edge with the maximum flow from the current node.
    double maximum_flow = 0;
    VertexId max_flow_vertex_id;
    Edge* max_flow_edge = nullptr;
    for (Edge* e : outgoing_edges[path_vertex_ids.back()]) {
      const double next_flow = flows[e->id()];
      const VertexId next_vertex_id = e->v().id();

      // If the edge has not been visited and has a flow greater than the
      // current maximum, update the maximum flow and the vertex id.
      if (visited_vertex_ids.count(e->v().id()) == 0 &&
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
      DRAKE_DEMAND(!path_vertex_ids.empty());
      continue;
    } else {
      // If the maximum flow is non-zero, add the vertex to the path and
      // continue the search.
      visited_vertex_ids.insert(max_flow_vertex_id);
      path_vertex_ids.push_back(max_flow_vertex_id);
      path_edges.push_back(max_flow_edge);
    }
  }

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
  for (Edge* edge : path_edges) {
    // Extract phi from the solution to rescale the control points and duration
    // in case we get the relaxed solution.
    const double phi_inv = 1 / result.GetSolution(edge->phi());
    // Extract the control points from the solution.
    const int num_control_points = (edge->xu().size() - 1) / num_positions();
    const MatrixX<double> edge_path_points =
        phi_inv *
        Eigen::Map<MatrixX<double>>(result.GetSolution(edge->xu()).data(),
                                    num_positions(), num_control_points);

    // Extract the duration from the solution.
    const double h = phi_inv * result.GetSolution(edge->xu()).tail<1>().value();
    const double start_time =
        bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();
    bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
        start_time, start_time + h, edge_path_points));
  }

  // Get the final control points from the solution.
  const double phi_inv = 1 / result.GetSolution(path_edges.back()->phi());
  const int num_control_points =
      (path_edges.back()->xv().size() - 1) / num_positions();
  const MatrixX<double> edge_path_points =
      phi_inv * Eigen::Map<MatrixX<double>>(
                    result.GetSolution(path_edges.back()->xv()).data(),
                    num_positions(), num_control_points);

  const double h =
      phi_inv * result.GetSolution(path_edges.back()->xv()).tail<1>().value();
  const double start_time =
      bezier_curves.empty() ? 0 : bezier_curves.back()->end_time();
  bezier_curves.emplace_back(std::make_unique<BezierCurve<double>>(
      start_time, start_time + h, edge_path_points));

  return {CompositeTrajectory<double>(bezier_curves), result};
}

Edge* GcsTrajectoryOptimization::AddEdge(const Vertex& u, const Vertex& v) {
  return gcs_.AddEdge(u, v, fmt::format("{} -> {}", u.name(), v.name()));
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
