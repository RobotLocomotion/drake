// TODO(jwnimmer-tri) Port GetGraphvizString to fmt, once we have sufficient
// options there to control precision and scientific formatting.
#undef EIGEN_NO_IO

#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/math/quadratic_form.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Edge = GraphOfConvexSets::Edge;
using EdgeId = GraphOfConvexSets::EdgeId;
using Vertex = GraphOfConvexSets::Vertex;
using VertexId = GraphOfConvexSets::VertexId;

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::RowVector2d;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::Cost;
using solvers::L1NormCost;
using solvers::L2NormCost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::LInfNormCost;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::MatrixXDecisionVariable;
using solvers::PerspectiveQuadraticCost;
using solvers::QuadraticCost;
using solvers::SolutionResult;
using solvers::VariableRefList;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

GraphOfConvexSets::~GraphOfConvexSets() = default;

Vertex::Vertex(VertexId id, const ConvexSet& set, std::string name)
    : id_(id),
      set_(set.Clone()),
      name_(std::move(name)),
      placeholder_x_(symbolic::MakeVectorContinuousVariable(
          set_->ambient_dimension(), name_)) {}

Vertex::~Vertex() = default;

std::pair<Variable, Binding<Cost>> Vertex::AddCost(
    const symbolic::Expression& e) {
  return AddCost(solvers::internal::ParseCost(e));
}

std::pair<Variable, Binding<Cost>> Vertex::AddCost(
    const Binding<Cost>& binding) {
  DRAKE_THROW_UNLESS(
      Variables(binding.variables()).IsSubsetOf(Variables(placeholder_x_)));
  const int n = ell_.size();
  ell_.conservativeResize(n + 1);
  ell_[n] = Variable(fmt::format("v_ell{}", n), Variable::Type::CONTINUOUS);
  costs_.emplace_back(binding);
  return std::pair<Variable, Binding<Cost>>(ell_[n], costs_.back());
}

Binding<Constraint> Vertex::AddConstraint(const symbolic::Formula& f) {
  return AddConstraint(solvers::internal::ParseConstraint(f));
}

Binding<Constraint> Vertex::AddConstraint(const Binding<Constraint>& binding) {
  DRAKE_THROW_UNLESS(
      Variables(binding.variables()).IsSubsetOf(Variables(placeholder_x_)));
  constraints_.emplace_back(binding);
  return binding;
}

double Vertex::GetSolutionCost(const MathematicalProgramResult& result) const {
  return result.GetSolution(ell_).sum();
}

VectorXd Vertex::GetSolution(const MathematicalProgramResult& result) const {
  return result.GetSolution(placeholder_x_);
}

Edge::Edge(const EdgeId& id, const Vertex* u, const Vertex* v, std::string name)
    : id_{id},
      u_{u},
      v_{v},
      allowed_vars_{u_->x()},
      phi_{"phi", symbolic::Variable::Type::BINARY},
      name_(std::move(name)),
      y_{symbolic::MakeVectorContinuousVariable(u_->ambient_dimension(), "y")},
      z_{symbolic::MakeVectorContinuousVariable(v_->ambient_dimension(), "z")},
      x_to_yz_{static_cast<size_t>(y_.size() + z_.size())} {
  DRAKE_DEMAND(u_ != nullptr);
  DRAKE_DEMAND(v_ != nullptr);
  allowed_vars_.insert(Variables(v_->x()));
  for (int i = 0; i < u_->ambient_dimension(); ++i) {
    x_to_yz_.emplace(u_->x()[i], y_[i]);
  }
  for (int i = 0; i < v_->ambient_dimension(); ++i) {
    x_to_yz_.emplace(v_->x()[i], z_[i]);
  }
}

Edge::~Edge() = default;

std::pair<Variable, Binding<Cost>> Edge::AddCost(
    const symbolic::Expression& e) {
  return AddCost(solvers::internal::ParseCost(e));
}

std::pair<Variable, Binding<Cost>> Edge::AddCost(const Binding<Cost>& binding) {
  DRAKE_THROW_UNLESS(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
  const int n = ell_.size();
  ell_.conservativeResize(n + 1);
  ell_[n] = Variable(fmt::format("ell{}", n), Variable::Type::CONTINUOUS);
  costs_.emplace_back(binding);
  return std::pair<Variable, Binding<Cost>>(ell_[n], costs_.back());
}

Binding<Constraint> Edge::AddConstraint(const symbolic::Formula& f) {
  return AddConstraint(solvers::internal::ParseConstraint(f));
}

Binding<Constraint> Edge::AddConstraint(const Binding<Constraint>& binding) {
  DRAKE_THROW_UNLESS(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
  constraints_.emplace_back(binding);
  return binding;
}

void Edge::AddPhiConstraint(bool phi_value) { phi_value_ = phi_value; }

void Edge::ClearPhiConstraints() { phi_value_ = std::nullopt; }

double Edge::GetSolutionCost(const MathematicalProgramResult& result) const {
  return result.GetSolution(ell_).sum();
}

Eigen::VectorXd Edge::GetSolutionPhiXu(
    const solvers::MathematicalProgramResult& result) const {
  return result.GetSolution(y_);
}

Eigen::VectorXd Edge::GetSolutionPhiXv(
    const solvers::MathematicalProgramResult& result) const {
  return result.GetSolution(z_);
}

Vertex* GraphOfConvexSets::AddVertex(const ConvexSet& set, std::string name) {
  if (name.empty()) {
    name = fmt::format("v{}", vertices_.size());
  }
  VertexId id = VertexId::get_new_id();
  auto [iter, success] = vertices_.try_emplace(id, new Vertex(id, set, name));
  DRAKE_DEMAND(success);
  return iter->second.get();
}

Edge* GraphOfConvexSets::AddEdge(VertexId u_id, VertexId v_id,
                                 std::string name) {
  auto u_iter = vertices_.find(u_id);
  DRAKE_DEMAND(u_iter != vertices_.end());
  auto v_iter = vertices_.find(v_id);
  DRAKE_DEMAND(v_iter != vertices_.end());

  if (name.empty()) {
    name = fmt::format("e{}", edges_.size());
  }
  EdgeId id = EdgeId::get_new_id();
  auto [iter, success] = edges_.try_emplace(
      id, new Edge(id, u_iter->second.get(), v_iter->second.get(), name));
  DRAKE_DEMAND(success);
  return iter->second.get();
}

Edge* GraphOfConvexSets::AddEdge(const Vertex& u, const Vertex& v,
                                 std::string name) {
  return AddEdge(u.id(), v.id(), std::move(name));
}

void GraphOfConvexSets::RemoveVertex(VertexId vertex_id) {
  DRAKE_DEMAND(vertices_.find(vertex_id) != vertices_.end());
  const auto& last_edge = edges_.end();
  for (auto e = edges_.begin(); e != last_edge;) {
    if (e->second->u().id() == vertex_id || e->second->v().id() == vertex_id) {
      e = edges_.erase(e);
    } else {
      ++e;
    }
  }
  vertices_.erase(vertex_id);
}

void GraphOfConvexSets::RemoveVertex(const Vertex& vertex) {
  RemoveVertex(vertex.id());
}

void GraphOfConvexSets::RemoveEdge(EdgeId edge_id) {
  DRAKE_DEMAND(edges_.find(edge_id) != edges_.end());
  edges_.erase(edge_id);
}

void GraphOfConvexSets::RemoveEdge(const Edge& edge) { RemoveEdge(edge.id()); }

std::vector<Vertex*> GraphOfConvexSets::Vertices() {
  std::vector<Vertex*> vertices;
  vertices.reserve(vertices_.size());
  for (const auto& v : vertices_) {
    vertices.push_back(v.second.get());
  }
  return vertices;
}

std::vector<const Vertex*> GraphOfConvexSets::Vertices() const {
  std::vector<const Vertex*> vertices;
  vertices.reserve(vertices_.size());
  for (const auto& v : vertices_) {
    vertices.push_back(v.second.get());
  }
  return vertices;
}

std::vector<Edge*> GraphOfConvexSets::Edges() {
  std::vector<Edge*> edges;
  edges.reserve(edges_.size());
  for (const auto& e : edges_) {
    edges.push_back(e.second.get());
  }
  return edges;
}

std::vector<const Edge*> GraphOfConvexSets::Edges() const {
  std::vector<const Edge*> edges;
  edges.reserve(edges_.size());
  for (const auto& e : edges_) {
    edges.push_back(e.second.get());
  }
  return edges;
}
void GraphOfConvexSets::ClearAllPhiConstraints() {
  for (const auto& e : edges_) {
    e.second->ClearPhiConstraints();
  }
}

// TODO(russt): We could get fancy and dim the color of the nodes/edges
// according to phi, using e.g. https://graphviz.org/docs/attr-types/color/ .
std::string GraphOfConvexSets::GetGraphvizString(
    const std::optional<solvers::MathematicalProgramResult>& result,
    bool show_slacks, int precision, bool scientific) const {
  // Note: We use stringstream instead of fmt in order to control the
  // formatting of the Eigen output and double output in a consistent way.
  std::stringstream graphviz;
  graphviz.precision(precision);
  if (!scientific) graphviz << std::fixed;
  graphviz << "digraph GraphOfConvexSets {\n";
  graphviz << "labelloc=t;\n";
  for (const auto& [v_id, v] : vertices_) {
    graphviz << "v" << v_id << " [label=\"" << v->name();
    if (result) {
      graphviz << "\n x = [" << result->GetSolution(v->x()).transpose() << "]";
    }
    graphviz << "\"]\n";
  }
  for (const auto& [e_id, e] : edges_) {
    unused(e_id);
    graphviz << "v" << e->u().id() << " -> v" << e->v().id();
    graphviz << " [label=\"" << e->name();
    if (result) {
      graphviz << "\n";
      graphviz << "cost = " << e->GetSolutionCost(*result);
      if (show_slacks) {
        graphviz << ",\n";
        graphviz << "ϕ = " << result->GetSolution(e->phi()) << ",\n";
        graphviz << "ϕ xᵤ = [" << e->GetSolutionPhiXu(*result).transpose()
                 << "],\n";
        graphviz << "ϕ xᵥ = [" << e->GetSolutionPhiXv(*result).transpose()
                 << "]";
      }
    }
    graphviz << "\"];\n";
  }
  graphviz << "}\n";
  return graphviz.str();
}

// Implements the preprocessing scheme put forth in Appendix A.2 of
// "Motion Planning around Obstacles with Convex Optimization":
// https://arxiv.org/abs/2205.04422
std::set<EdgeId> GraphOfConvexSets::PreprocessShortestPath(
    VertexId source_id, VertexId target_id) const {
  DRAKE_DEMAND(vertices_.find(source_id) != vertices_.end());
  DRAKE_DEMAND(vertices_.find(target_id) != vertices_.end());

  std::map<VertexId, std::vector<int>> incoming_edges;
  std::map<VertexId, std::vector<int>> outgoing_edges;
  std::set<EdgeId> unusable_edges;

  int edge_count = 0;
  for (const auto& [edge_id, e] : edges_) {
    // Turn off edges into source or out of target
    if (e->v().id() == source_id || e->u().id() == target_id) {
      unusable_edges.insert(edge_id);
    } else {
      outgoing_edges[e->u().id()].push_back(edge_count);
      incoming_edges[e->v().id()].push_back(edge_count);
    }

    edge_count++;
  }

  int nE = edges_.size();

  // Given an edge (u,v) check if a path from source to u and another from v to
  // target exist without sharing edges.
  MathematicalProgram prog;

  // Flow for each edge is between 0 and 1 for both paths.
  VectorXDecisionVariable f = prog.NewContinuousVariables(nE, "flow_su");
  Binding<solvers::BoundingBoxConstraint> f_limits =
      prog.AddBoundingBoxConstraint(0, 1, f);
  VectorXDecisionVariable g = prog.NewContinuousVariables(nE, "flow_vt");
  Binding<solvers::BoundingBoxConstraint> g_limits =
      prog.AddBoundingBoxConstraint(0, 1, g);

  std::map<VertexId, Binding<LinearEqualityConstraint>> conservation_f;
  std::map<VertexId, Binding<LinearEqualityConstraint>> conservation_g;
  std::map<VertexId, Binding<LinearConstraint>> degree;
  for (const auto& [vertex_id, v] : vertices_) {
    std::vector<int> Ev_in = incoming_edges[vertex_id];
    std::vector<int> Ev_out = outgoing_edges[vertex_id];
    std::vector<int> Ev = Ev_in;
    Ev.insert(Ev.end(), Ev_out.begin(), Ev_out.end());

    if (Ev.size() > 0) {
      RowVectorXd A_flow(Ev.size());
      A_flow << RowVectorXd::Ones(Ev_in.size()),
          -1 * RowVectorXd::Ones(Ev_out.size());
      VectorXDecisionVariable fv(Ev.size());
      VectorXDecisionVariable gv(Ev.size());
      for (size_t ii = 0; ii < Ev.size(); ++ii) {
        fv(ii) = f(Ev[ii]);
        gv(ii) = g(Ev[ii]);
      }

      // Conservation of flow for f: ∑ f_in - ∑ f_out = -δ(is_source).
      if (vertex_id == source_id) {
        conservation_f.insert(
            {vertex_id, prog.AddLinearEqualityConstraint(A_flow, -1, fv)});
      } else {
        conservation_f.insert(
            {vertex_id, prog.AddLinearEqualityConstraint(A_flow, 0, fv)});
      }

      // Conservation of flow for g: ∑ g_in - ∑ g_out = δ(is_target).
      if (vertex_id == target_id) {
        conservation_g.insert(
            {vertex_id, prog.AddLinearEqualityConstraint(A_flow, 1, gv)});
      } else {
        conservation_g.insert(
            {vertex_id, prog.AddLinearEqualityConstraint(A_flow, 0, gv)});
      }
    }

    // Degree constraints (redundant if indegree of w is 0):
    // 0 <= ∑ f_in + ∑ g_in <= 1
    if (Ev_in.size() > 0) {
      RowVectorXd A_degree = RowVectorXd::Ones(2 * Ev_in.size());
      VectorXDecisionVariable fgin(2 * Ev_in.size());
      for (size_t ii = 0; ii < Ev_in.size(); ++ii) {
        fgin(ii) = f(Ev_in[ii]);
        fgin(Ev_in.size() + ii) = g(Ev_in[ii]);
      }
      degree.insert(
          {vertex_id, prog.AddLinearConstraint(A_degree, 0, 1, fgin)});
    }
  }

  for (const auto& [edge_id, e] : edges_) {
    if (unusable_edges.count(edge_id)) {
      continue;
    }

    // Update bounds of conservation of flow:
    // ∑ f_in,u - ∑ f_out,u = 1 - δ(is_source).
    if (e->u().id() == source_id) {
      f_limits.evaluator()->set_bounds(VectorXd::Zero(nE), VectorXd::Zero(nE));
      conservation_f.at(e->u().id())
          .evaluator()
          ->set_bounds(Vector1d(0), Vector1d(0));
    } else {
      conservation_f.at(e->u().id())
          .evaluator()
          ->set_bounds(Vector1d(1), Vector1d(1));
    }
    // ∑ g_in,v - ∑ f_out,v = δ(is_target) - 1.
    if (e->v().id() == target_id) {
      g_limits.evaluator()->set_bounds(VectorXd::Zero(nE), VectorXd::Zero(nE));
      conservation_g.at(e->v().id())
          .evaluator()
          ->set_bounds(Vector1d(0), Vector1d(0));
    } else {
      conservation_g.at(e->v().id())
          .evaluator()
          ->set_bounds(Vector1d(-1), Vector1d(-1));
    }

    // Update bounds of degree constraints:
    // ∑ f_in,v + ∑ g_in,v = 0.
    degree.at(e->v().id()).evaluator()->set_bounds(Vector1d(0), Vector1d(0));

    // Check if edge e = (u,v) could be on a path from start to goal.
    auto result = Solve(prog);
    if (!result.is_success()) {
      unusable_edges.insert(edge_id);
    }

    // Reset constraint bounds.
    if (e->u().id() == source_id) {
      f_limits.evaluator()->set_bounds(VectorXd::Zero(nE), VectorXd::Ones(nE));
      conservation_f.at(e->u().id())
          .evaluator()
          ->set_bounds(Vector1d(-1), Vector1d(-1));
    } else {
      conservation_f.at(e->u().id())
          .evaluator()
          ->set_bounds(Vector1d(0), Vector1d(0));
    }
    if (e->v().id() == target_id) {
      g_limits.evaluator()->set_bounds(VectorXd::Zero(nE), VectorXd::Ones(nE));
      conservation_g.at(e->v().id())
          .evaluator()
          ->set_bounds(Vector1d(1), Vector1d(1));
    } else {
      conservation_g.at(e->v().id())
          .evaluator()
          ->set_bounds(Vector1d(0), Vector1d(0));
    }
    degree.at(e->v().id()).evaluator()->set_bounds(Vector1d(0), Vector1d(1));
  }
  return unusable_edges;
}

void GraphOfConvexSets::AddPerspectiveCost(
    MathematicalProgram* prog, const Binding<Cost>& binding,
    const VectorXDecisionVariable& vars) const {
  const double inf = std::numeric_limits<double>::infinity();

  // TODO(russt): Avoid this use of RTTI, which mirrors the current
  // pattern in MathematicalProgram::AddCost.
  Cost* cost = binding.evaluator().get();
  if (LinearCost* lc = dynamic_cast<LinearCost*>(cost)) {
    // TODO(russt): Consider setting a precision here (and exposing it to
    // the user) instead of using Eigen's dummy_precision.
    if (lc->a().isZero() && lc->b() < 0.0) {
      throw std::runtime_error(fmt::format(
          "Constant costs must be non-negative: {}", binding.to_string()));
    }
    // a*x + phi*b <= ell or [b, -1.0, a][phi; ell; x] <= 0
    RowVectorXd a(lc->a().size() + 2);
    a(0) = lc->b();
    a(1) = -1.0;
    a.tail(lc->a().size()) = lc->a();
    prog->AddLinearConstraint(a, -inf, 0.0, vars);
  } else if (QuadraticCost* qc = dynamic_cast<QuadraticCost*>(cost)) {
    // .5 x'Qx + b'x + c is restated as a rotated Lorentz cone constraint
    // enforcing that ℓ should be lower-bounded by the perspective, with
    // coefficient ϕ, of the quadratic form:
    // slack ≥ 0, and  slack ϕ ≥ .5x'Qx, with slack := ℓ - b'x - ϕ c.

    // TODO(russt): Allow users to set this tolerance.  (Probably in
    // solvers::QuadraticCost instead of here).
    const double tol = 1e-10;
    Eigen::MatrixXd R =
        math::DecomposePSDmatrixIntoXtransposeTimesX(.5 * qc->Q(), tol);
    // Note: QuadraticCost guarantees that Q() is symmetric.
    const VectorXd xmin = -qc->Q().ldlt().solve(qc->b());
    VectorXd minimum(1);
    qc->Eval(xmin, &minimum);
    if (minimum[0] < -tol) {
      throw std::runtime_error(fmt::format(
          "In order to prevent negative edge lengths, quadratic "
          "costs must be strictly non-negative: {} obtains a minimum of "
          "{}.",
          binding.to_string(), minimum[0]));
    }
    MatrixXd A_cone = MatrixXd::Zero(R.rows() + 2, vars.size());
    A_cone(0, 0) = 1.0;  // z₀ = ϕ.
    // z₁ = ℓ - b'x - ϕ c.
    A_cone(1, 1) = 1.0;
    A_cone.block(1, 2, 1, qc->b().rows()) = -qc->b().transpose();
    A_cone(1, 0) = -qc->c();
    // z₂ ... z_{n+1} = R x.
    A_cone.block(2, 2, R.rows(), R.cols()) = R;
    prog->AddRotatedLorentzConeConstraint(A_cone, VectorXd::Zero(A_cone.rows()),
                                          vars);
  } else if (L1NormCost* l1c = dynamic_cast<L1NormCost*>(cost)) {
    // |Ax + b|₁ becomes ℓ ≥ Σᵢ δᵢ and δᵢ ≥ |Aᵢx+bᵢϕ|.
    int A_rows = l1c->A().rows();
    int A_cols = l1c->A().cols();
    auto l1c_slack = prog->NewContinuousVariables(A_rows, "l1c_slack");
    VectorXDecisionVariable cost_vars(vars.size() + l1c_slack.size());
    cost_vars << vars, l1c_slack;
    MatrixXd A_linear = MatrixXd::Zero(2 * A_rows + 1, cost_vars.size());

    // δᵢ ≥ Aᵢx+bᵢϕ
    A_linear.block(0, 0, A_rows, 1) = l1c->b();       // bϕ.
    A_linear.block(0, 2, A_rows, A_cols) = l1c->A();  // Ax.
    A_linear.block(0, A_cols + 2, A_rows, l1c_slack.size()) =
        -MatrixXd::Identity(A_rows, A_rows);  // -δ.

    // -δᵢ ≤ Aᵢx+bᵢϕ
    A_linear.block(A_rows, 0, A_rows, 1) = -l1c->b();       // -bϕ.
    A_linear.block(A_rows, 2, A_rows, A_cols) = -l1c->A();  // -Ax.
    A_linear.block(A_rows, A_cols + 2, A_rows, l1c_slack.size()) =
        -MatrixXd::Identity(A_rows, A_rows);  // -δ.

    // ℓ ≥ Σᵢ δᵢ
    A_linear(2 * A_rows, 1) = -1;
    A_linear.block(2 * A_rows, A_cols + 2, 1, l1c_slack.size()) =
        RowVectorXd::Ones(l1c_slack.size());
    prog->AddLinearConstraint(A_linear,
                              VectorXd::Constant(A_linear.rows(), -inf),
                              VectorXd::Zero(A_linear.rows()), cost_vars);
  } else if (L2NormCost* l2c = dynamic_cast<L2NormCost*>(cost)) {
    // |Ax + b|₂ becomes ℓ ≥ |Ax+bϕ|₂.
    MatrixXd A_cone = MatrixXd::Zero(l2c->A().rows() + 1, vars.size());
    A_cone(0, 1) = 1.0;                                 // z₀ = ℓ.
    A_cone.block(1, 0, l2c->A().rows(), 1) = l2c->b();  // bϕ.
    A_cone.block(1, 2, l2c->A().rows(), l2c->A().cols()) = l2c->A();  // Ax.
    prog->AddLorentzConeConstraint(A_cone, VectorXd::Zero(A_cone.rows()), vars);
  } else if (LInfNormCost* linfc = dynamic_cast<LInfNormCost*>(cost)) {
    // |Ax + b|∞ becomes ℓ ≥ |Aᵢx+bᵢϕ| ∀ i.
    int A_rows = linfc->A().rows();
    MatrixXd A_linear(2 * A_rows, vars.size());
    A_linear.block(0, 0, A_rows, 1) = linfc->b();                    // bϕ.
    A_linear.block(0, 1, A_rows, 1) = -VectorXd::Ones(A_rows);       // -ℓ.
    A_linear.block(0, 2, A_rows, linfc->A().cols()) = linfc->A();    // Ax.
    A_linear.block(A_rows, 0, A_rows, 1) = -linfc->b();              // -bϕ.
    A_linear.block(A_rows, 1, A_rows, 1) = -VectorXd::Ones(A_rows);  // -ℓ.
    A_linear.block(A_rows, 2, A_rows, linfc->A().cols()) = -linfc->A();  // -Ax.
    prog->AddLinearConstraint(A_linear,
                              VectorXd::Constant(A_linear.rows(), -inf),
                              VectorXd::Zero(A_linear.rows()), vars);
  } else if (PerspectiveQuadraticCost* pqc =
                 dynamic_cast<PerspectiveQuadraticCost*>(cost)) {
    // (z_1^2 + ... + z_{n-1}^2) / z_0 for z = Ax + b becomes
    // ℓ z_0 ≥ z_1^2 + ... + z_{n-1}^2 for z = Ax + bϕ
    MatrixXd A_cone = MatrixXd::Zero(pqc->A().rows() + 1, vars.size());
    A_cone(0, 1) = 1.0;
    A_cone.block(1, 0, pqc->A().rows(), 1) = pqc->b();
    A_cone.block(1, 2, pqc->A().rows(), pqc->A().cols()) = pqc->A();
    prog->AddRotatedLorentzConeConstraint(
        A_cone, VectorXd::Zero(pqc->A().rows() + 1), vars);
  } else {
    throw std::runtime_error(fmt::format(
        "GraphOfConvexSets::Edge does not support this binding type: {}",
        binding.to_string()));
  }
}

void GraphOfConvexSets::AddPerspectiveConstraint(
    MathematicalProgram* prog, const Binding<Constraint>& binding,
    const VectorXDecisionVariable& vars) const {
  const double inf = std::numeric_limits<double>::infinity();

  Constraint* constraint = binding.evaluator().get();
  if (LinearEqualityConstraint* lec =
          dynamic_cast<LinearEqualityConstraint*>(constraint)) {
    // A*x = b becomes A*x = phi*b.
    // TODO(hongkai.dai): use sparse version of Aeq.
    const Eigen::MatrixXd& A = lec->GetDenseA();
    MatrixXd Aeq(A.rows(), A.cols() + 1);
    Aeq.col(0) = -lec->lower_bound();
    Aeq.rightCols(A.cols()) = A;
    prog->AddLinearEqualityConstraint(Aeq, VectorXd::Zero(A.rows()), vars);
    // Note that LinearEqualityConstraint must come before LinearConstraint,
    // because LinearEqualityConstraint isa LinearConstraint.
  } else if (LinearConstraint* lc =
                 dynamic_cast<LinearConstraint*>(constraint)) {
    // lb <= A*x <= ub becomes
    // A*x <= phi*ub and phi*lb <= A*x, which can be spelled
    // [-ub, A][phi; x] <= 0, and 0 <= [-lb, A][phi; x].
    // TODO(hongkai.dai): use a sparse version of a matrix.
    const Eigen::MatrixXd& A = lc->GetDenseA();
    RowVectorXd a(vars.size());
    for (int i = 0; i < A.rows(); ++i) {
      if (std::isfinite(lc->upper_bound()[i])) {
        a[0] = -lc->upper_bound()[i];
        a.tail(A.cols()) = A.row(i);
        prog->AddLinearConstraint(a, -inf, 0, vars);
      }
      if (std::isfinite(lc->lower_bound()[i])) {
        a[0] = -lc->lower_bound()[i];
        a.tail(A.cols()) = A.row(i);
        prog->AddLinearConstraint(a, 0, inf, vars);
      }
    }
  } else {
    throw std::runtime_error(
        fmt::format("ShortestPathProblem::Edge does not support this "
                    "binding type: {}",
                    binding.to_string()));
  }
}

namespace {
MathematicalProgramResult Solve(const MathematicalProgram& prog,
                                const GraphOfConvexSetsOptions& options,
                                bool rounding) {
  MathematicalProgramResult result;
  auto solver_options = (rounding && options.rounding_solver_options)
                            ? options.rounding_solver_options
                            : options.solver_options;
  if (options.solver) {
    options.solver->Solve(prog, {}, solver_options, &result);
  } else {
    result = solvers::Solve(prog, {}, solver_options);
  }
  return result;
}
}  // namespace

MathematicalProgramResult GraphOfConvexSets::SolveShortestPath(
    VertexId source_id, VertexId target_id,
    const GraphOfConvexSetsOptions& options) const {
  DRAKE_DEMAND(vertices_.find(source_id) != vertices_.end());
  DRAKE_DEMAND(vertices_.find(target_id) != vertices_.end());

  std::set<EdgeId> unusable_edges;
  if (options.preprocessing) {
    unusable_edges = PreprocessShortestPath(source_id, target_id);
  }

  MathematicalProgram prog;

  std::map<VertexId, std::vector<Edge*>> incoming_edges;
  std::map<VertexId, std::vector<Edge*>> outgoing_edges;
  std::map<VertexId, MatrixXDecisionVariable> vertex_edge_ell;
  std::vector<Edge*> excluded_edges;

  std::map<EdgeId, Variable> relaxed_phi;
  std::vector<Variable> excluded_phi;

  for (const auto& [edge_id, e] : edges_) {
    // If an edge is turned off (ϕ = 0) or excluded by preprocessing, don't
    // include it in the optimization.
    if (!e->phi_value_.value_or(true) || unusable_edges.count(edge_id)) {
      // Track excluded edges (ϕ = 0 and preprocessed) so that their variables
      // can be set in the optimization result.
      excluded_edges.emplace_back(e.get());
      if (options.convex_relaxation) {
        Variable phi("phi_excluded");
        excluded_phi.push_back(phi);
      }
      continue;
    }
    outgoing_edges[e->u().id()].emplace_back(e.get());
    incoming_edges[e->v().id()].emplace_back(e.get());

    Variable phi;
    if (options.convex_relaxation) {
      phi = prog.NewContinuousVariables<1>("phi")[0];
      prog.AddBoundingBoxConstraint(0, 1, phi);
      relaxed_phi.emplace(edge_id, phi);
    } else {
      phi = e->phi_;
      prog.AddDecisionVariables(Vector1<Variable>(phi));
    }
    if (e->phi_value_.has_value()) {
      DRAKE_DEMAND(*e->phi_value_);
      double phi_value = *e->phi_value_ ? 1.0 : 0.0;
      prog.AddBoundingBoxConstraint(phi_value, phi_value, phi);
    }
    prog.AddDecisionVariables(e->y_);
    prog.AddDecisionVariables(e->z_);
    prog.AddDecisionVariables(e->ell_);
    prog.AddLinearCost(VectorXd::Ones(e->ell_.size()), e->ell_);

    // Spatial non-negativity: y ∈ ϕX, z ∈ ϕX.
    e->u().set().AddPointInNonnegativeScalingConstraints(&prog, e->y_, phi);
    e->v().set().AddPointInNonnegativeScalingConstraints(&prog, e->z_, phi);

    // Edge costs.
    for (int i = 0; i < e->ell_.size(); ++i) {
      const Binding<Cost>& b = e->costs_[i];

      const VectorXDecisionVariable& old_vars = b.variables();
      VectorXDecisionVariable vars(old_vars.size() + 2);
      // vars = [phi; ell; yz_vars]
      vars[0] = phi;
      vars[1] = e->ell_[i];
      for (int j = 0; j < old_vars.size(); ++j) {
        vars[j + 2] = e->x_to_yz_.at(old_vars[j]);
      }

      AddPerspectiveCost(&prog, b, vars);
    }

    // Edge constraints.
    for (const Binding<Constraint>& b : e->constraints_) {
      const VectorXDecisionVariable& old_vars = b.variables();
      VectorXDecisionVariable vars(old_vars.size() + 1);
      // vars = [phi; yz_vars]
      vars[0] = phi;
      for (int j = 0; j < old_vars.size(); ++j) {
        vars[j + 1] = e->x_to_yz_.at(old_vars[j]);
      }

      // Note: The use of perspective functions here does not check (nor assume)
      // that the constraints describe a bounded set.  The boundedness is
      // ensured by the intersection of these constraints with the convex sets
      // (on the vertices).
      AddPerspectiveConstraint(&prog, b, vars);
    }
  }

  for (const std::pair<const VertexId, std::unique_ptr<Vertex>>& vpair :
       vertices_) {
    const Vertex* v = vpair.second.get();
    const bool is_source = (source_id == v->id());
    const bool is_target = (target_id == v->id());

    const std::vector<Edge*>& incoming = incoming_edges[v->id()];
    const std::vector<Edge*>& outgoing = outgoing_edges[v->id()];

    // TODO(russt): Make the bindings of these constraints available to the user
    // so that they can check the dual solution.  Or perhaps better, create more
    // placeholder variables for the dual solutions, and just pack them into the
    // program result in the standard way.

    if (incoming.size() + outgoing.size() > 0) {  // in degree + out degree
      VectorXDecisionVariable vars(incoming.size() + outgoing.size());
      RowVectorXd a(incoming.size() + outgoing.size());
      a << RowVectorXd::Constant(incoming.size(), -1.0),
          RowVectorXd::Ones(outgoing.size());

      // Conservation of flow: ∑ ϕ_out - ∑ ϕ_in = δ(is_source) - δ(is_target).
      int count = 0;
      for (const Edge* e : incoming) {
        vars[count++] =
            options.convex_relaxation ? relaxed_phi.at(e->id()) : e->phi_;
      }
      for (const Edge* e : outgoing) {
        vars[count++] =
            options.convex_relaxation ? relaxed_phi.at(e->id()) : e->phi_;
      }
      prog.AddLinearEqualityConstraint(
          a, (is_source ? 1.0 : 0.0) - (is_target ? 1.0 : 0.0), vars);

      // Spatial conservation of flow: ∑ z_in = ∑ y_out.
      if (!is_source && !is_target) {
        for (int i = 0; i < v->ambient_dimension(); ++i) {
          count = 0;
          for (const Edge* e : incoming) {
            vars[count++] = e->z_[i];
          }
          for (const Edge* e : outgoing) {
            vars[count++] = e->y_[i];
          }
          prog.AddLinearEqualityConstraint(a, 0, vars);
        }
      }
    }

    if (outgoing.size() > 0) {
      int n_v = v->ambient_dimension();
      VectorXDecisionVariable phi_out(outgoing.size());
      VectorXDecisionVariable yz_out(outgoing.size() * n_v);
      for (int i = 0; i < static_cast<int>(outgoing.size()); ++i) {
        phi_out[i] = options.convex_relaxation
                         ? relaxed_phi.at(outgoing[i]->id())
                         : outgoing[i]->phi_;
        yz_out.segment(i * n_v, n_v) = outgoing[i]->y_;
      }
      // Degree constraint: ∑ ϕ_out <= 1- δ(is_target).
      prog.AddLinearConstraint(RowVectorXd::Ones(outgoing.size()), 0.0,
                               is_target ? 0.0 : 1.0, phi_out);

      if (!is_source && !is_target) {
        RowVectorXd a = RowVectorXd::Ones(outgoing.size());
        MatrixXd A_yz(n_v, outgoing.size() * n_v);
        for (int i = 0; i < static_cast<int>(outgoing.size()); ++i) {
          A_yz.block(0, i * n_v, n_v, n_v) = MatrixXd::Identity(n_v, n_v);
        }
        for (int i = 0; i < static_cast<int>(outgoing.size()); ++i) {
          const Edge* e_out = outgoing[i];
          if (source_id == e_out->v().id() || target_id == e_out->v().id()) {
            continue;
          }
          for (const Edge* e_in : incoming) {
            if (e_in->u().id() == e_out->v().id()) {
              a[i] = -1.0;
              phi_out[i] = options.convex_relaxation
                               ? relaxed_phi.at(e_in->id())
                               : e_in->phi_;
              // Two-cycle constraint: ∑ ϕ_u,out - ϕ_uv - ϕ_vu >= 0
              prog.AddLinearConstraint(a, 0.0, 1.0, phi_out);
              A_yz.block(0, i * n_v, n_v, n_v) = -MatrixXd::Identity(n_v, n_v);
              yz_out.segment(i * n_v, n_v) = e_in->z_;
              // Two-cycle spatial constraint:
              // ∑ y_u - y_uv - z_vu ∈ (∑ ϕ_u,out - ϕ_uv - ϕ_vu) X_u
              v->set().AddPointInNonnegativeScalingConstraints(
                  &prog, A_yz, VectorXd::Zero(n_v), a, 0, yz_out, phi_out);

              a[i] = 1.0;
              phi_out[i] = options.convex_relaxation
                               ? relaxed_phi.at(e_out->id())
                               : e_out->phi_;
              A_yz.block(0, i * n_v, n_v, n_v) = MatrixXd::Identity(n_v, n_v);
              yz_out.segment(i * n_v, n_v) = e_out->y_;
            }
          }
        }
      }
    }

    const std::vector<Edge*>& cost_edges = is_target ? incoming : outgoing;

    // Vertex costs.
    if (v->ell_.size() > 0) {
      vertex_edge_ell[v->id()] =
          prog.NewContinuousVariables(cost_edges.size(), v->ell_.size());
      for (int ii = 0; ii < v->ell_.size(); ++ii) {
        const Binding<Cost>& b = v->costs_[ii];
        const VectorXDecisionVariable& old_vars = b.variables();

        VectorXDecisionVariable vertex_ell =
            vertex_edge_ell.at(v->id()).col(ii);
        prog.AddLinearCost(VectorXd::Ones(vertex_ell.size()), vertex_ell);

        for (int jj = 0; jj < static_cast<int>(cost_edges.size()); ++jj) {
          const Edge* e = cost_edges[jj];
          VectorXDecisionVariable vars(old_vars.size() + 2);
          // vars = [phi; ell; yz_vars]
          if (options.convex_relaxation) {
            vars[0] = relaxed_phi.at(e->id());
          } else {
            vars[0] = e->phi_;
          }
          vars[1] = vertex_ell[jj];
          for (int kk = 0; kk < old_vars.size(); ++kk) {
            vars[kk + 2] = e->x_to_yz_.at(old_vars[kk]);
          }

          AddPerspectiveCost(&prog, b, vars);
        }
      }
    }

    // Vertex constraints.
    for (const Binding<Constraint>& b : v->constraints_) {
      const VectorXDecisionVariable& old_vars = b.variables();

      for (const Edge* e : cost_edges) {
        VectorXDecisionVariable vars(old_vars.size() + 1);
        // vars = [phi; yz_vars]
        if (options.convex_relaxation) {
          vars[0] = relaxed_phi.at(e->id());
        } else {
          vars[0] = e->phi_;
        }
        for (int ii = 0; ii < old_vars.size(); ++ii) {
          vars[ii + 1] = e->x_to_yz_.at(old_vars[ii]);
        }

        // Note: The use of perspective functions here does not check (nor
        // assume) that the constraints describe a bounded set.  The boundedness
        // is ensured by the intersection of these constraints with the convex
        // sets (on the vertices).
        AddPerspectiveConstraint(&prog, b, vars);
      }
    }
  }

  MathematicalProgramResult result = Solve(prog, options, false);

  // Implements the rounding scheme put forth in Section 4.2 of
  // "Motion Planning around Obstacles with Convex Optimization":
  // https://arxiv.org/abs/2205.04422
  if (options.convex_relaxation && options.max_rounded_paths > 0 &&
      result.is_success()) {
    DRAKE_THROW_UNLESS(options.max_rounding_trials > 0);
    RandomGenerator generator(options.rounding_seed);
    std::uniform_real_distribution<double> uniform;
    std::vector<std::vector<const Edge*>> paths;
    std::map<EdgeId, double> flows;
    for (const auto& [edge_id, e] : edges_) {
      if (!e->phi_value_.value_or(true) || unusable_edges.count(edge_id)) {
        flows.emplace(edge_id, 0);
      } else {
        flows.emplace(edge_id, result.GetSolution(relaxed_phi[edge_id]));
      }
    }
    int num_trials = 0;
    MathematicalProgramResult best_rounded_result;
    while (static_cast<int>(paths.size()) < options.max_rounded_paths &&
           num_trials < options.max_rounding_trials) {
      ++num_trials;

      // Find candidate path by traversing the graph with a depth first search
      // where edges are taken with prbability proportional to their flow.
      std::vector<VertexId> visited_vertex_ids{source_id};
      std::vector<VertexId> path_vertex_ids{source_id};
      std::vector<const Edge*> new_path;
      while (path_vertex_ids.back() != target_id) {
        std::vector<const Edge*> candidate_edges;
        for (const Edge* e : outgoing_edges[path_vertex_ids.back()]) {
          if (std::find(visited_vertex_ids.begin(), visited_vertex_ids.end(),
                        e->v().id()) == visited_vertex_ids.end() &&
              flows[e->id()] > options.flow_tolerance) {
            candidate_edges.emplace_back(e);
          }
        }
        // If the depth first search finds itself at a node with no candidate
        // outbound edges, backtrack to the previous node and continue the
        // search.
        if (candidate_edges.size() == 0) {
          path_vertex_ids.pop_back();
          new_path.pop_back();
          continue;
        }
        Eigen::VectorXd candidate_flows(candidate_edges.size());
        for (size_t ii = 0; ii < candidate_edges.size(); ++ii) {
          candidate_flows(ii) = flows[candidate_edges[ii]->id()];
        }
        double edge_sample = uniform(generator) * candidate_flows.sum();
        for (size_t ii = 0; ii < candidate_edges.size(); ++ii) {
          if (edge_sample >= candidate_flows(ii)) {
            edge_sample -= candidate_flows(ii);
          } else {
            visited_vertex_ids.push_back(candidate_edges[ii]->v().id());
            path_vertex_ids.push_back(candidate_edges[ii]->v().id());
            new_path.emplace_back(candidate_edges[ii]);
            break;
          }
        }
      }

      if (std::find(paths.begin(), paths.end(), new_path) != paths.end()) {
        continue;
      }
      paths.push_back(new_path);

      // Optimize path
      std::vector<Binding<Constraint>> added_constraints;
      for (const auto& [edge_id, e] : edges_) {
        if (e->phi_value_.has_value() || unusable_edges.count(edge_id)) {
          continue;
        }
        if (std::find(new_path.begin(), new_path.end(), e.get()) !=
            new_path.end()) {
          added_constraints.push_back(
              prog.AddBoundingBoxConstraint(1, 1, relaxed_phi.at(edge_id)));
        } else {
          added_constraints.push_back(
              prog.AddBoundingBoxConstraint(0, 0, relaxed_phi.at(edge_id)));
          added_constraints.push_back(prog.AddLinearEqualityConstraint(
              e->y_.cast<Expression>(), VectorXd::Zero(e->y_.size())));
          added_constraints.push_back(prog.AddLinearEqualityConstraint(
              e->z_.cast<Expression>(), VectorXd::Zero(e->z_.size())));
          added_constraints.push_back(prog.AddLinearEqualityConstraint(
              e->ell_.cast<Expression>(), VectorXd::Zero(e->ell_.size())));
        }
      }

      MathematicalProgramResult rounded_result = Solve(prog, options, true);

      // Check path quality.
      if (rounded_result.is_success() &&
          (!best_rounded_result.is_success() ||
           rounded_result.get_optimal_cost() <
               best_rounded_result.get_optimal_cost())) {
        best_rounded_result = rounded_result;
      }

      for (Binding<Constraint>& con : added_constraints) {
        prog.RemoveConstraint(con);
      }
    }
    if (best_rounded_result.is_success()) {
      result = best_rounded_result;
    } else {
      result.set_solution_result(SolutionResult::kIterationLimit);
    }
  }

  // Push the placeholder variables and excluded edge variables into the result,
  // so that they can be accessed as if they were variables included in the
  // optimization.
  int num_placeholder_vars = relaxed_phi.size();
  for (const std::pair<const VertexId, std::unique_ptr<Vertex>>& vpair :
       vertices_) {
    num_placeholder_vars += vpair.second->ambient_dimension();
    num_placeholder_vars += vpair.second->ell_.size();
  }
  for (const Edge* e : excluded_edges) {
    num_placeholder_vars += e->y_.size() + e->z_.size() + e->ell_.size() + 1;
  }
  num_placeholder_vars += excluded_phi.size();
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index =
      prog.decision_variable_index();
  int count = result.get_x_val().size();
  Eigen::VectorXd x_val(count + num_placeholder_vars);
  x_val.head(count) = result.get_x_val();
  for (const Edge* e : excluded_edges) {
    for (int i = 0; i < e->y_.size(); ++i) {
      decision_variable_index.emplace(e->y_[i].get_id(), count);
      x_val[count++] = 0;
    }
    for (int i = 0; i < e->z_.size(); ++i) {
      decision_variable_index.emplace(e->z_[i].get_id(), count);
      x_val[count++] = 0;
    }
    for (int i = 0; i < e->ell_.size(); ++i) {
      decision_variable_index.emplace(e->ell_[i].get_id(), count);
      x_val[count++] = 0;
    }
    decision_variable_index.emplace(e->phi_.get_id(), count);
    x_val[count++] = 0;
  }
  for (const Variable& phi : excluded_phi) {
    decision_variable_index.emplace(phi.get_id(), count);
    x_val[count++] = 0;
  }
  for (const std::pair<const VertexId, std::unique_ptr<Vertex>>& vpair :
       vertices_) {
    const Vertex* v = vpair.second.get();
    const bool is_target = (target_id == v->id());
    VectorXd x_v = VectorXd::Zero(v->ambient_dimension());
    double sum_phi = 0;
    if (is_target) {
      sum_phi = 1.0;
      for (const auto& e : incoming_edges[v->id()]) {
        x_v += result.GetSolution(e->z_);
      }
    } else {
      for (const auto& e : outgoing_edges[v->id()]) {
        x_v += result.GetSolution(e->y_);
        sum_phi += result.GetSolution(
            options.convex_relaxation ? relaxed_phi.at(e->id()) : e->phi_);
      }
    }
    // In the convex relaxation, sum_relaxed_phi may not be one even for
    // vertices in the shortest path. We undo yₑ = ϕₑ xᵤ here to ensure that
    // xᵤ is in v->set(). If ∑ ϕₑ is small enough that numerical errors
    // prevent the projection back into the Xᵤ, then we prefer to return NaN.
    if (sum_phi < 100.0 * std::numeric_limits<double>::epsilon()) {
      x_v = VectorXd::Constant(v->ambient_dimension(),
                               std::numeric_limits<double>::quiet_NaN());
    } else if (options.convex_relaxation) {
      x_v /= sum_phi;
    }
    for (int i = 0; i < v->ambient_dimension(); ++i) {
      decision_variable_index.emplace(v->x()[i].get_id(), count);
      x_val[count++] = x_v[i];
    }
    for (int ii = 0; ii < v->ell_.size(); ++ii) {
      decision_variable_index.emplace(v->ell_[ii].get_id(), count);
      x_val[count++] =
          result.GetSolution(vertex_edge_ell.at(v->id()).col(ii)).sum();
    }
  }
  if (options.convex_relaxation) {
    // Write the value of the relaxed phi into the phi placeholder.
    for (const auto& [edge_id, relaxed_phi_var] : relaxed_phi) {
      decision_variable_index.emplace(edges_.at(edge_id)->phi_.get_id(), count);
      x_val[count++] = result.GetSolution(relaxed_phi_var);
    }
  }
  result.set_decision_variable_index(decision_variable_index);
  result.set_x_val(x_val);

  return result;
}

MathematicalProgramResult GraphOfConvexSets::SolveShortestPath(
    const Vertex& source, const Vertex& target,
    const GraphOfConvexSetsOptions& options) const {
  return SolveShortestPath(source.id(), target.id(), options);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
