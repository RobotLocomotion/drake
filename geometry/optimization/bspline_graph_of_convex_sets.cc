#include "drake/geometry/optimization/bspline_graph_of_convex_sets.h"

#include <algorithm>
#include <limits>
#include <map>

#include <fmt/format.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/symbolic.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/knot_vector_type.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Vector1d = Vector1<double>;
using Eigen::VectorXd;
using math::BsplineBasis;
using math::KnotVectorType;
using symbolic::DecomposeLinearExpressions;
using symbolic::Expression;
using symbolic::ExtractVariablesFromExpression;
using symbolic::intersect;
using symbolic::Variable;
using symbolic::Variables;
using trajectories::BsplineTrajectory;

BsplineTrajectoryThroughUnionOfHPolyhedra::
    BsplineTrajectoryThroughUnionOfHPolyhedra(
        const Eigen::Ref<const Eigen::VectorXd>& source,
        const Eigen::Ref<const Eigen::VectorXd>& target,
        const std::vector<HPolyhedron>& regions)
    : source_index_(regions.size()),
      target_index_(regions.size() + 1),
      source_(source),
      target_(target),
      regions_(regions) {
  DRAKE_THROW_UNLESS(source.size() == ambient_dimension());
  DRAKE_THROW_UNLESS(target.size() == ambient_dimension());
  max_velocity_ = VectorXd::Constant(ambient_dimension(),
                                     std::numeric_limits<double>::infinity());
  for (const auto& region : regions_) {
    DRAKE_THROW_UNLESS(region.ambient_dimension() == ambient_dimension());
  }
  regions_.push_back(HPolyhedron::MakeBox(source, source));
  regions_.push_back(HPolyhedron::MakeBox(target, target));
}

std::optional<trajectories::BsplineTrajectory<double>>
BsplineTrajectoryThroughUnionOfHPolyhedra::Solve(bool use_rounding) const {
  DRAKE_THROW_UNLESS(max_repetitions() == 1);
  GraphOfConvexSets problem{};

  // Construct edges between all intersecting regions.
  std::multimap<int, int> edge_descriptions;
  for (int i{0}; i < num_regions(); ++i) {
    for (int j{i + 1}; j < num_regions(); ++j) {
      if (regions_[i].IntersectsWith(regions_[j])) {
        edge_descriptions.insert({i, j});
        edge_descriptions.insert({j, i});
      }
    }
  }

  std::vector<HPolyhedron> replicated_regions{};
  const int control_points_per_region =
      order() - 1 + extra_control_points_per_region_;
  HPolyhedron duration_scaling_set =
      HPolyhedron::MakeBox(Vector1d(0.0), Vector1d(100.0));
  for (const auto& region : regions_) {
    replicated_regions.push_back(
        region.CartesianPower(control_points_per_region)
            .CartesianProduct(duration_scaling_set));
  }
  std::vector<GraphOfConvexSets::Vertex*> vertices;
  for (const auto& region : replicated_regions) {
    vertices.push_back(problem.AddVertex(region));
  }
  GraphOfConvexSets::Vertex* source_region = vertices[source_index_];
  GraphOfConvexSets::Vertex* target_region = vertices[target_index_];

  // Set up edge cost.
  int control_points_per_edge = 2 * control_points_per_region;
  MatrixXd H{};
  std::vector<MatrixXd> velocity_control_point_mappings{};
  MatrixXd velocity_limit_mapping{};
  {
    MatrixX<Variable> u_control_points = symbolic::MakeMatrixVariable(
        ambient_dimension(), control_points_per_region, "xu");
    VectorX<Variable> u_duration_scaling =
        symbolic::MakeVectorContinuousVariable(1, "Tu");
    MatrixX<Variable> v_control_points = symbolic::MakeMatrixVariable(
        ambient_dimension(), control_points_per_region, "xv");
    VectorX<Variable> v_duration_scaling =
        symbolic::MakeVectorContinuousVariable(1, "Tv");
    std::vector<MatrixX<Expression>> edge_control_points;
    edge_control_points.reserve(control_points_per_edge);
    for (int i{0}; i < control_points_per_region; ++i) {
      edge_control_points.push_back(u_control_points.col(i));
    }
    for (int i{0}; i < control_points_per_region; ++i) {
      edge_control_points.push_back(v_control_points.col(i));
    }
    BsplineTrajectory<Expression> position_trajectory{
        BsplineBasis<Expression>(order(), control_points_per_edge,
                                 KnotVectorType::kUniform, 0.0,
                                 control_points_per_edge - (order() - 1.0)),
        edge_control_points};
    int min_derivative_order{std::min(1, order()-1)};
    int max_derivative_order{std::min(order()-1, order()-1)};
    std::vector<Expression> cost_components{};
    Variables variables_in_u_control_points{Eigen::Map<VectorX<Variable>>(
        u_control_points.data(), u_control_points.size())};
    for (int derivative_order{min_derivative_order};
         derivative_order <= max_derivative_order; ++derivative_order) {
      std::unique_ptr<BsplineTrajectory<Expression>> derivative_trajectory{
          dynamic_pointer_cast_or_throw<BsplineTrajectory<Expression>>(
              position_trajectory.MakeDerivative(derivative_order))};
      for (const auto& control_point :
           derivative_trajectory->control_points()) {
        for (int i{0}; i < ambient_dimension(); ++i) {
          Variables variables_in_this_expression{
              ExtractVariablesFromExpression(control_point(i, 0)).first};
          if (!intersect(variables_in_u_control_points,
                         variables_in_this_expression)
                   .empty()) {
            cost_components.push_back(control_point(i, 0));
          }
        }
      }
    }
    cost_components.push_back(1e1 * u_duration_scaling(0));
    cost_components.push_back(1e1 * v_duration_scaling(0));
    VectorX<Variable> edge_variables(
        control_points_per_edge * ambient_dimension() + 2);
    edge_variables << Eigen::Map<VectorX<Variable>>(u_control_points.data(),
                                                    u_control_points.size()),
        u_duration_scaling,
        Eigen::Map<VectorX<Variable>>(v_control_points.data(),
                                      u_control_points.size()),
        v_duration_scaling;
    H.resize(cost_components.size(), edge_variables.size());
    DecomposeLinearExpressions(
        Eigen::Map<VectorX<Expression>>(cost_components.data(),
                                        cost_components.size()),
        edge_variables, &H);
    H /= H.norm();

    // Compute edge_variables to velocity control point mappings.
    std::unique_ptr<BsplineTrajectory<Expression>> velocity_trajectory{
        dynamic_pointer_cast_or_throw<BsplineTrajectory<Expression>>(
            position_trajectory.MakeDerivative())};
    for (const auto& control_point : velocity_trajectory->control_points()) {
      Variables variables_in_this_expression{
          ExtractVariablesFromExpression(control_point(0, 0)).first};
      if (!intersect(variables_in_u_control_points,
                     variables_in_this_expression)
               .empty()) {
        velocity_control_point_mappings.emplace_back(ambient_dimension(),
                                                     edge_variables.size());
        DecomposeLinearExpressions(control_point, edge_variables,
                                   &(velocity_control_point_mappings.back()));
        drake::log()->debug(velocity_control_point_mappings.back());
      }
    }
    velocity_limit_mapping.resize(ambient_dimension(), edge_variables.size());
    DecomposeLinearExpressions(
        VectorX<Expression>::Constant(
            ambient_dimension(),
            u_duration_scaling(0) / (control_points_per_edge - (order() - 1.0)))
            .cwiseProduct(max_velocity_),
        edge_variables, &velocity_limit_mapping);
  }
  auto cost =
      std::make_shared<solvers::L2NormCost>(H, VectorXd::Zero(H.rows()));
  drake::log()->debug("H =\n{}", H);

  // Add edges.
  std::vector<GraphOfConvexSets::Edge*> edges;
  edges.reserve(edge_descriptions.size());
  for (const auto& edge_description : edge_descriptions) {
    edges.push_back(problem.AddEdge(
        *vertices[edge_description.first], *vertices[edge_description.second],
        fmt::format("({}, {})", edge_description.first,
                    edge_description.second)));
    GraphOfConvexSets::Edge& edge = *edges.back();
    edge.AddCost(solvers::Binding(cost, {edge.xu(), edge.xv()}));

    // Constrain the first `order() -1` control points in Xv to also be in Xu.
    const Eigen::Map<const MatrixX<Variable>> v_control_points(
        edge.xv().data(), ambient_dimension(), control_points_per_region);
    const HPolyhedron& Xu = regions_.at(edge_description.first);
    for (int j{0}; j < order() - 1; ++j) {
      edge.AddConstraint(
          {std::make_shared<solvers::LinearConstraint>(
               Xu.A(),
               VectorXd::Constant(Xu.b().size(),
                                  -std::numeric_limits<double>::infinity()),
               Xu.b()),
           v_control_points.col(j)});
    }

    // Constrain duration scaling to match between Xu and Xv.
    const Variable& u_duration_scaling = edge.xu().tail(1)(0);
    const Variable& v_duration_scaling = edge.xv().tail(1)(0);
    edge.AddConstraint(u_duration_scaling == v_duration_scaling);
    drake::log()->debug("Added edge {} ...", edge.name());

    // Add velocity constraints.
    if (velocity_limit_mapping.array().isFinite().all()) {
      for (const auto& velocity_control_point_mapping :
           velocity_control_point_mappings) {
        drake::log()->debug(velocity_control_point_mapping);
        drake::log()->debug(velocity_limit_mapping);
        edge.AddConstraint(solvers::Binding(
            std::make_shared<solvers::LinearConstraint>(
                velocity_control_point_mapping - velocity_limit_mapping,
                VectorXd::Constant(ambient_dimension(),
                                   -std::numeric_limits<double>::infinity()),
                VectorXd::Zero(ambient_dimension())),
            {edge.xu(), edge.xv()}));
        edge.AddConstraint(solvers::Binding(
            std::make_shared<solvers::LinearConstraint>(
                -velocity_control_point_mapping - velocity_limit_mapping,
                VectorXd::Constant(ambient_dimension(),
                                   -std::numeric_limits<double>::infinity()),
                VectorXd::Zero(ambient_dimension())),
            {edge.xu(), edge.xv()}));
        drake::log()->debug("Added velocity constraints for edge {}",
                           edge.name());
      }
    }
  }

  solvers::MathematicalProgramResult result =
      problem.SolveShortestPath(*source_region, *target_region, use_rounding);
  drake::log()->debug("Cost: {}, Solver ID: {}", result.get_optimal_cost(),
                     result.get_solver_id());

  if (!result.is_success()) return std::nullopt;

  // Identify the active path through the graph.
  std::vector<GraphOfConvexSets::Edge*> active_edges;
  GraphOfConvexSets::Edge* next_active_edge;
  double max_phi = 0;
  for (const auto& edge : edges) {
    drake::log()->debug("Checking edge {}", edge->name());
    const double phi{result.GetSolution(edge->phi())};
    if (&edge->u() == source_region && phi > max_phi) {
      next_active_edge = edge;
      max_phi = phi;
    }
  }
  active_edges.push_back(next_active_edge);
  drake::log()->debug("Found first active edge ...");
  drake::log()->debug("Added edge {} with phi = {} ...",
                      next_active_edge->name(), max_phi);
  while (&active_edges.back()->v() != target_region) {
    drake::log()->debug("Looking for successor to {}",
                       active_edges.back()->name());
    max_phi = 0;
    for (const auto& edge : edges) {
      const double phi{result.GetSolution(edge->phi())};
      drake::log()->debug("Checking edge {}", edge->name());
      if (&edge->u() == &active_edges.back()->v() && phi > max_phi) {
        next_active_edge = edge;
        max_phi = phi;
      }
    }
    active_edges.push_back(next_active_edge);
    drake::log()->debug("Added edge {} with phi = {} ...",
                        next_active_edge->name(), max_phi);
  }
  if (use_rounding) {
    for (const auto& edge : edges) {
      if (std::count(active_edges.begin(), active_edges.end(), edge) > 0) {
        edge->AddPhiConstraint(true);
      } else {
        edge->AddPhiConstraint(false);
      }
    }
    result =
      problem.SolveShortestPath(*source_region, *target_region, use_rounding);
    drake::log()->debug("Cost: {}, Solver ID: {}", result.get_optimal_cost(),
                       result.get_solver_id());
  }

  // Extract the solution trajectory.
  std::vector<MatrixXd> control_points{};
  control_points.reserve(control_points_per_region * (active_edges.size() + 1));
  {
    VectorXd vertex_control_points_flattened =
        result.GetSolution(active_edges.front()->xu());
    drake::log()->debug("vertex_control_points_flattened.size(): {}",
                       vertex_control_points_flattened.size());
    Eigen::Map<MatrixXd> vertex_control_points(
        vertex_control_points_flattened.data(), ambient_dimension(),
        control_points_per_region);
    drake::log()->debug("vertex_control_points:\n{}", vertex_control_points);
    for (int i{0}; i < control_points_per_region; ++i) {
      control_points.push_back(vertex_control_points.col(i));
    }
  }
  for (const auto& edge : active_edges) {
    VectorXd vertex_control_points_flattened = result.GetSolution(edge->xv());
    drake::log()->debug("vertex_control_points_flattened.size(): {}",
                       vertex_control_points_flattened.size());
    Eigen::Map<MatrixXd> vertex_control_points(
        vertex_control_points_flattened.data(), ambient_dimension(),
        control_points_per_region);
    drake::log()->debug("vertex_control_points:\n{}", vertex_control_points);
    for (int i{0}; i < control_points_per_region; ++i) {
      control_points.push_back(vertex_control_points.col(i));
    }
  }
  drake::log()->debug("Solution trajectory has {} control points",
                     control_points.size());

  for (const auto& control_point : control_points) {
    drake::log()->debug(control_point.transpose());
  }

  double nominal_duration = (control_points.size() - (order() - 1.0)) /
                            (control_points_per_edge - (order() - 1.0));
  double duration_scaling =
      result.GetSolution(active_edges.back()->xu().tail(1)(0));

  return trajectories::BsplineTrajectory<double>(
      math::BsplineBasis<double>(order(), control_points.size(),
                                 math::KnotVectorType::kUniform, 0,
                                 nominal_duration * duration_scaling),
      control_points);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
