// TODO(jwnimmer-tri) Port GetGraphvizString to fmt, once we have sufficient
// options there to control precision and scientific formatting.
#undef EIGEN_NO_IO

#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/math/quadratic_form.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/get_program_type.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace geometry {
namespace optimization {

using Edge = GraphOfConvexSets::Edge;
using EdgeId = GraphOfConvexSets::EdgeId;
using Transcription = GraphOfConvexSets::Transcription;
using Vertex = GraphOfConvexSets::Vertex;
using VertexId = GraphOfConvexSets::VertexId;

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::RowVector2d;
using Eigen::RowVectorXd;
using Eigen::SparseMatrix;
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
using solvers::LorentzConeConstraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::MatrixXDecisionVariable;
using solvers::PerspectiveQuadraticCost;
using solvers::PositiveSemidefiniteConstraint;
using solvers::ProgramType;
using solvers::QuadraticCost;
using solvers::RotatedLorentzConeConstraint;
using solvers::SolutionResult;
using solvers::VariableRefList;
using solvers::VectorXDecisionVariable;
using solvers::internal::CreateBinding;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

namespace {
MathematicalProgramResult Solve(const MathematicalProgram& prog,
                                const GraphOfConvexSetsOptions& options) {
  MathematicalProgramResult result;
  if (options.solver) {
    options.solver->Solve(prog, {}, options.solver_options, &result);

    // TODO(wrangelvid): Call the MixedIntegerBranchAndBound solver when
    // asking to solve the MIP without a solver that supports it.
  } else {
    std::unique_ptr<solvers::SolverInterface> solver{};
    try {
      solvers::SolverId solver_id = solvers::ChooseBestSolver(prog);
      solver = solvers::MakeSolver(solver_id);
    } catch (const std::exception&) {
      // We should only get here if the user is trying to solve the MIP.
      DRAKE_DEMAND(options.convex_relaxation == false);

      // TODO(russt): Consider calling MixedIntegerBranchAndBound automatically
      // here. The small trick is that we need to pass the SolverId into that
      // constructor manually, and ChooseBestSolver doesn't make it easy to
      // figure out what the best solver would be if we removed the integer
      // variables.

      throw std::runtime_error(
          "GraphOfConvexSets: There is no solver available that can solve the "
          "mixed-integer version of this problem. Please check "
          "https://drake.mit.edu/doxygen_cxx/group__solvers.html for more "
          "details about supported solvers and how to enable them.\n\n "
          "Alternatively, you can try to solve the problem without integer "
          "variables by setting options.convex_relaxation=true (and likely "
          "setting options.max_rounded_paths > 0 if you want an "
          "integer-feasible solution). See the documentation for "
          "GraphOfConvexSetsOptions for more details.");
    }
    DRAKE_DEMAND(solver != nullptr);
    solver->Solve(prog, {}, options.solver_options, &result);
  }
  return result;
}

struct VertexIdComparator {
  bool operator()(const Vertex* lhs, const Vertex* rhs) const {
    return lhs->id() < rhs->id();
  }
};

struct EdgeIdComparator {
  bool operator()(const Edge* lhs, const Edge* rhs) const {
    return lhs->id() < rhs->id();
  }
};

}  // namespace

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

Binding<Constraint> Vertex::AddConstraint(
    const symbolic::Formula& f,
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        use_in_transcription) {
  return AddConstraint(solvers::internal::ParseConstraint(f),
                       use_in_transcription);
}

Binding<Constraint> Vertex::AddConstraint(
    const Binding<Constraint>& binding,
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        use_in_transcription) {
  DRAKE_THROW_UNLESS(ambient_dimension() > 0);
  DRAKE_THROW_UNLESS(
      Variables(binding.variables()).IsSubsetOf(Variables(placeholder_x_)));
  DRAKE_THROW_UNLESS(use_in_transcription.size() > 0);
  constraints_.push_back({binding, use_in_transcription});
  return binding;
}

std::vector<solvers::Binding<solvers::Constraint>> Vertex::GetConstraints(
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        used_in_transcription) const {
  DRAKE_THROW_UNLESS(used_in_transcription.size() > 0);
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  // Add all constraints that are used in the transcription.
  for (const auto& [binding, transcriptions] : constraints_) {
    if (std::any_of(transcriptions.begin(), transcriptions.end(),
                    [&used_in_transcription](const auto& elem) {
                      return used_in_transcription.contains(elem);
                    })) {
      constraints.push_back(binding);
    }
  }
  return constraints;
}

double Vertex::GetSolutionCost(const MathematicalProgramResult& result) const {
  return result.GetSolution(ell_).sum();
}

VectorXd Vertex::GetSolution(const MathematicalProgramResult& result) const {
  return result.GetSolution(placeholder_x_);
}

void Vertex::AddIncomingEdge(Edge* e) {
  incoming_edges_.push_back(e);
}

void Vertex::AddOutgoingEdge(Edge* e) {
  outgoing_edges_.push_back(e);
}

void Vertex::RemoveIncomingEdge(Edge* e) {
  incoming_edges_.erase(
      std::remove(incoming_edges_.begin(), incoming_edges_.end(), e),
      incoming_edges_.end());
}

void Vertex::RemoveOutgoingEdge(Edge* e) {
  outgoing_edges_.erase(
      std::remove(outgoing_edges_.begin(), outgoing_edges_.end(), e),
      outgoing_edges_.end());
}

Edge::Edge(const EdgeId& id, Vertex* u, Vertex* v, std::string name)
    : id_{id},
      u_{u},
      v_{v},
      allowed_vars_{u_->x()},
      phi_{name + "phi", symbolic::Variable::Type::BINARY},
      name_(std::move(name)),
      y_{symbolic::MakeVectorContinuousVariable(u_->ambient_dimension(),
                                                name_ + "y")},
      z_{symbolic::MakeVectorContinuousVariable(v_->ambient_dimension(),
                                                name_ + "z")},
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
  ell_[n] =
      Variable(fmt::format("{}ell{}", name_, n), Variable::Type::CONTINUOUS);
  costs_.emplace_back(binding);
  return std::pair<Variable, Binding<Cost>>(ell_[n], costs_.back());
}

Binding<Constraint> Edge::AddConstraint(
    const symbolic::Formula& f,
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        use_in_transcription) {
  return AddConstraint(solvers::internal::ParseConstraint(f),
                       use_in_transcription);
}

Binding<Constraint> Edge::AddConstraint(
    const Binding<Constraint>& binding,
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        use_in_transcription) {
  const int total_ambient_dimension = allowed_vars_.size();
  DRAKE_THROW_UNLESS(total_ambient_dimension > 0);
  DRAKE_THROW_UNLESS(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
  DRAKE_THROW_UNLESS(use_in_transcription.size() > 0);
  constraints_.push_back({binding, use_in_transcription});
  return binding;
}

std::vector<solvers::Binding<solvers::Constraint>> Edge::GetConstraints(
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        used_in_transcription) const {
  DRAKE_THROW_UNLESS(used_in_transcription.size() > 0);
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  // Add all constraints that are used in the transcription.
  for (const auto& [binding, transcriptions] : constraints_) {
    if (std::any_of(transcriptions.begin(), transcriptions.end(),
                    [&used_in_transcription](const auto& elem) {
                      return used_in_transcription.contains(elem);
                    })) {
      constraints.push_back(binding);
    }
  }
  return constraints;
}

void Edge::AddPhiConstraint(bool phi_value) {
  phi_value_ = phi_value;
}

void Edge::ClearPhiConstraints() {
  phi_value_ = std::nullopt;
}

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

Edge* GraphOfConvexSets::AddEdge(Vertex* u, Vertex* v, std::string name) {
  DRAKE_DEMAND(u != nullptr);
  DRAKE_DEMAND(v != nullptr);
  if (name.empty()) {
    name = fmt::format("e{}", edges_.size());
  }
  EdgeId id = EdgeId::get_new_id();
  auto [iter, success] = edges_.try_emplace(id, new Edge(id, u, v, name));
  DRAKE_DEMAND(success);
  Edge* e = iter->second.get();
  u->AddOutgoingEdge(e);
  v->AddIncomingEdge(e);
  return e;
}

void GraphOfConvexSets::RemoveVertex(Vertex* vertex) {
  DRAKE_THROW_UNLESS(vertex != nullptr);
  VertexId vertex_id = vertex->id();
  DRAKE_THROW_UNLESS(vertices_.contains(vertex_id));
  for (auto it = edges_.begin(); it != edges_.end();) {
    if (it->second->u().id() == vertex_id) {
      it->second->v().RemoveIncomingEdge(it->second.get());
      it = edges_.erase(it);
    } else if (it->second->v().id() == vertex_id) {
      it->second->u().RemoveOutgoingEdge(it->second.get());
      it = edges_.erase(it);
    } else {
      ++it;
    }
  }
  vertices_.erase(vertex_id);
}

void GraphOfConvexSets::RemoveEdge(Edge* edge) {
  DRAKE_THROW_UNLESS(edge != nullptr);
  DRAKE_THROW_UNLESS(edges_.contains(edge->id()));
  edge->u().RemoveOutgoingEdge(edge);
  edge->v().RemoveIncomingEdge(edge);
  edges_.erase(edge->id());
}

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
      if (e->ell_.size() > 0) {
        // SolveConvexRestriction does not yet return the rewritten costs.
        if (result->get_decision_variable_index()->contains(
                e->ell_[0].get_id())) {
          graphviz << "cost = " << e->GetSolutionCost(*result);
        }
      } else {
        graphviz << "cost = 0";
      }
      if (show_slacks) {
        graphviz << ",\n";
        graphviz << "ϕ = " << result->GetSolution(e->phi()) << ",\n";
        if (result->get_decision_variable_index()->contains(
                e->y_[0].get_id())) {
          graphviz << "ϕ xᵤ = [" << e->GetSolutionPhiXu(*result).transpose()
                   << "],\n";
          graphviz << "ϕ xᵥ = [" << e->GetSolutionPhiXv(*result).transpose()
                   << "]";
        }
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
    VertexId source_id, VertexId target_id,
    const GraphOfConvexSetsOptions& options) const {
  if (vertices_.find(source_id) == vertices_.end()) {
    throw std::runtime_error(fmt::format(
        "Source vertex {} is not a vertex in this GraphOfConvexSets.",
        source_id));
  }
  if (vertices_.find(target_id) == vertices_.end()) {
    throw std::runtime_error(fmt::format(
        "Target vertex {} is not a vertex in this GraphOfConvexSets.",
        target_id));
  }

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
    if (unusable_edges.contains(edge_id)) {
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
    auto result = Solve(prog, options);
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
    MatrixXd A_cone =
        MatrixXd::Zero(l2c->get_sparse_A().rows() + 1, vars.size());
    A_cone(0, 1) = 1.0;  // z₀ = ℓ.
    A_cone.block(1, 0, l2c->get_sparse_A().rows(), 1) = l2c->b();  // bϕ.
    A_cone.block(1, 2, l2c->get_sparse_A().rows(), l2c->get_sparse_A().cols()) =
        l2c->get_sparse_A();  // Ax.
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
    const SparseMatrix<double>& A = lec->get_sparse_A();
    SparseMatrix<double> Aeq(A.rows(), A.cols() + 1);
    Aeq.col(0) = -lec->lower_bound().sparseView();
    Aeq.rightCols(A.cols()) = A;
    prog->AddConstraint(
        CreateBinding(std::make_shared<LinearEqualityConstraint>(
                          Aeq, VectorXd::Zero(A.rows())),
                      vars));
    // Note that LinearEqualityConstraint must come before LinearConstraint,
    // because LinearEqualityConstraint isa LinearConstraint.
  } else if (LinearConstraint* lc =
                 dynamic_cast<LinearConstraint*>(constraint)) {
    // lb <= A*x <= ub becomes
    // A*x <= phi*ub and phi*lb <= A*x, which can be spelled
    // [-ub, A][phi; x] <= 0, and 0 <= [-lb, A][phi; x].
    if (lc->upper_bound().array().isFinite().all()) {
      const SparseMatrix<double>& A = lc->get_sparse_A();
      SparseMatrix<double> Ac(A.rows(), A.cols() + 1);
      Ac.col(0) = -lc->upper_bound().sparseView();
      Ac.rightCols(A.cols()) = A;
      prog->AddConstraint(
          CreateBinding(std::make_shared<LinearConstraint>(
                            Ac, VectorXd::Constant(Ac.rows(), -inf),
                            VectorXd::Zero(Ac.rows())),
                        vars));
    } else if (lc->upper_bound().array().isInf().all()) {
      // Then do nothing.
    } else {
      // Need to go constraint by constraint.
      // TODO(Alexandre.Amice) make this only access the sparse matrix.
      const Eigen::MatrixXd& A = lc->GetDenseA();
      RowVectorXd a(vars.size());
      for (int i = 0; i < A.rows(); ++i) {
        if (std::isfinite(lc->upper_bound()[i])) {
          a[0] = -lc->upper_bound()[i];
          a.tail(A.cols()) = A.row(i);
          prog->AddLinearConstraint(a, -inf, 0, vars);
        }
      }
    }

    if (lc->lower_bound().array().isFinite().all()) {
      const SparseMatrix<double>& A = lc->get_sparse_A();
      SparseMatrix<double> Ac(A.rows(), A.cols() + 1);
      Ac.col(0) = -lc->lower_bound().sparseView();
      Ac.rightCols(A.cols()) = A;
      prog->AddConstraint(CreateBinding(std::make_shared<LinearConstraint>(
                                            Ac, VectorXd::Zero(Ac.rows()),
                                            VectorXd::Constant(Ac.rows(), inf)),
                                        vars));
    } else if (lc->lower_bound().array().isInf().all()) {
      // Then do nothing.
    } else {
      // Need to go constraint by constraint.
      const Eigen::MatrixXd& A = lc->GetDenseA();
      RowVectorXd a(vars.size());
      for (int i = 0; i < A.rows(); ++i) {
        if (std::isfinite(lc->lower_bound()[i])) {
          a[0] = -lc->lower_bound()[i];
          a.tail(A.cols()) = A.row(i);
          prog->AddLinearConstraint(a, 0, inf, vars);
        }
      }
    }
  } else if (LorentzConeConstraint* lcc =
                 dynamic_cast<LorentzConeConstraint*>(constraint)) {
    // z ∈ K for z = Ax + b becomes
    // z ∈ K for z = Ax + bϕ = [b A] [ϕ; x]
    // (Notice that this is the same as for a RotatedLorentzConeConstraint,
    // as it does not depend on whether the cone is rotated or not).
    MatrixXd A_cone = MatrixXd::Zero(lcc->A().rows(), vars.size());
    A_cone.block(0, 0, lcc->A().rows(), 1) = lcc->b();
    A_cone.block(0, 1, lcc->A().rows(), lcc->A().cols()) = lcc->A_dense();
    prog->AddLorentzConeConstraint(A_cone, VectorXd::Zero(lcc->A().rows()),
                                   vars);
  } else if (RotatedLorentzConeConstraint* rc =
                 dynamic_cast<RotatedLorentzConeConstraint*>(constraint)) {
    // z ∈ K for z = Ax + b becomes
    // z ∈ K for z = Ax + bϕ = [b A] [ϕ; x]
    // (Notice that this is the same as for a LorentzConeConstraint,
    // as it does not depend on whether the cone is rotated or not).
    MatrixXd A_cone = MatrixXd::Zero(rc->A().rows(), vars.size());
    A_cone.block(0, 0, rc->A().rows(), 1) = rc->b();
    A_cone.block(0, 1, rc->A().rows(), rc->A().cols()) = rc->A_dense();
    prog->AddRotatedLorentzConeConstraint(A_cone,
                                          VectorXd::Zero(rc->A().rows()), vars);
  } else if (dynamic_cast<PositiveSemidefiniteConstraint*>(constraint) !=
             nullptr) {
    // Since we have ϕ ≥ 0, we have S ≽ 0 ⇔ ϕS ≽ 0.
    // It is sufficient to add the original constraint to the program (with the
    // new variables).
    prog->AddConstraint(binding.evaluator(), vars.tail(vars.size() - 1));
  } else {
    throw std::runtime_error(
        fmt::format("ShortestPathProblem::Edge does not support this "
                    "binding type: {}",
                    binding.to_string()));
  }
}

MathematicalProgramResult GraphOfConvexSets::SolveShortestPath(
    const Vertex& source, const Vertex& target,
    const GraphOfConvexSetsOptions& specified_options) const {
  VertexId source_id = source.id();
  VertexId target_id = target.id();
  if (vertices_.find(source_id) == vertices_.end()) {
    throw std::runtime_error(fmt::format(
        "Source vertex {} is not a vertex in this GraphOfConvexSets.",
        source_id));
  }
  if (vertices_.find(target_id) == vertices_.end()) {
    throw std::runtime_error(fmt::format(
        "Target vertex {} is not a vertex in this GraphOfConvexSets.",
        target_id));
  }

  // Fill in default options. Note: if these options change, they must also be
  // updated in the method documentation.
  GraphOfConvexSetsOptions options = specified_options;
  if (!options.convex_relaxation) {
    options.convex_relaxation = false;
  }
  if (!options.preprocessing) {
    options.preprocessing = false;
  }
  if (!options.max_rounded_paths) {
    options.max_rounded_paths = 0;
  }

  std::set<EdgeId> unusable_edges;
  if (*options.preprocessing) {
    unusable_edges = PreprocessShortestPath(source_id, target_id, options);
  }

  MathematicalProgram prog;

  std::map<VertexId, std::vector<Edge*>> incoming_edges;
  std::map<VertexId, std::vector<Edge*>> outgoing_edges;
  std::map<VertexId, MatrixXDecisionVariable> vertex_edge_ell;
  std::vector<Edge*> excluded_edges;

  std::map<EdgeId, Variable> relaxed_phi;
  std::vector<Variable> excluded_phi;

  // The flow constraints below assume that we have some edge out of the source
  // and into the target, so we handle that case explicitly.
  bool has_edges_out_of_source = false;
  bool has_edges_into_target = false;
  for (const auto& [edge_id, e] : edges_) {
    // If an edge is turned off (ϕ = 0) or excluded by preprocessing, don't
    // include it in the optimization.
    if (!e->phi_value_.value_or(true) || unusable_edges.contains(edge_id)) {
      // Track excluded edges (ϕ = 0 and preprocessed) so that their variables
      // can be set in the optimization result.
      excluded_edges.emplace_back(e.get());
      if (*options.convex_relaxation) {
        Variable phi("phi_excluded");
        excluded_phi.push_back(phi);
      }
      continue;
    }
    if (e->u().id() == source_id) {
      has_edges_out_of_source = true;
    }
    if (e->v().id() == target_id) {
      has_edges_into_target = true;
    }
    outgoing_edges[e->u().id()].emplace_back(e.get());
    incoming_edges[e->v().id()].emplace_back(e.get());

    Variable phi;
    if (*options.convex_relaxation) {
      phi = prog.NewContinuousVariables<1>(e->name() + "phi")[0];
      prog.AddBoundingBoxConstraint(0, 1, phi);
      relaxed_phi.emplace(edge_id, phi);
    } else {
      phi = e->phi_;
      prog.AddDecisionVariables(Vector1<Variable>(phi));
    }
    if (e->phi_value_.has_value()) {
      DRAKE_DEMAND(*e->phi_value_);
      double phi_value = *e->phi_value_ ? 1.0 : 0.0;
      prog.AddLinearEqualityConstraint(Vector1d(1.0), phi_value,
                                       Vector1<Variable>(phi));
    }
    prog.AddDecisionVariables(e->y_);
    prog.AddDecisionVariables(e->z_);
    prog.AddDecisionVariables(e->ell_);
    prog.AddLinearCost(VectorXd::Ones(e->ell_.size()), e->ell_);

    // Spatial non-negativity: y ∈ ϕX, z ∈ ϕX.
    if (e->u().ambient_dimension() > 0) {
      e->u().set().AddPointInNonnegativeScalingConstraints(&prog, e->y_, phi);
    }
    if (e->v().ambient_dimension() > 0) {
      e->v().set().AddPointInNonnegativeScalingConstraints(&prog, e->z_, phi);
    }

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
    for (const auto& [b, transcriptions] : e->constraints_) {
      if ((*options.convex_relaxation &&
           transcriptions.contains(Transcription::kRelaxation)) ||
          (!*options.convex_relaxation &&
           transcriptions.contains(Transcription::kMIP))) {
        const VectorXDecisionVariable& old_vars = b.variables();
        VectorXDecisionVariable vars(old_vars.size() + 1);
        // vars = [phi; yz_vars]
        vars[0] = phi;
        for (int j = 0; j < old_vars.size(); ++j) {
          vars[j + 1] = e->x_to_yz_.at(old_vars[j]);
        }

        // Note: The use of perspective functions here does not check (nor
        // assume) that the constraints describe a bounded set.  The boundedness
        // is ensured by the intersection of these constraints with the convex
        // sets (on the vertices).
        AddPerspectiveConstraint(&prog, b, vars);
      }
    }
  }
  if (!has_edges_out_of_source) {
    MathematicalProgramResult result;
    log()->info("Source vertex {} ({}) has no outgoing edges.", source.name(),
                source_id);
    result.set_solution_result(SolutionResult::kInfeasibleConstraints);
    return result;
  }
  if (!has_edges_into_target) {
    MathematicalProgramResult result;
    log()->info("Target vertex {} ({}) has no incoming edges.", target.name(),
                target_id);
    result.set_solution_result(SolutionResult::kInfeasibleConstraints);
    return result;
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
            *options.convex_relaxation ? relaxed_phi.at(e->id()) : e->phi_;
      }
      for (const Edge* e : outgoing) {
        vars[count++] =
            *options.convex_relaxation ? relaxed_phi.at(e->id()) : e->phi_;
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
        phi_out[i] = *options.convex_relaxation
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
              phi_out[i] = *options.convex_relaxation
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
              phi_out[i] = *options.convex_relaxation
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
          if (*options.convex_relaxation) {
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
    for (const auto& [b, transcriptions] : v->constraints_) {
      if ((*options.convex_relaxation &&
           transcriptions.contains(Transcription::kRelaxation)) ||
          (!*options.convex_relaxation &&
           transcriptions.contains(Transcription::kMIP))) {
        const VectorXDecisionVariable& old_vars = b.variables();

        for (const Edge* e : cost_edges) {
          VectorXDecisionVariable vars(old_vars.size() + 1);
          // vars = [phi; yz_vars]
          if (*options.convex_relaxation) {
            vars[0] = relaxed_phi.at(e->id());
          } else {
            vars[0] = e->phi_;
          }
          for (int ii = 0; ii < old_vars.size(); ++ii) {
            vars[ii + 1] = e->x_to_yz_.at(old_vars[ii]);
          }

          // Note: The use of perspective functions here does not check (nor
          // assume) that the constraints describe a bounded set.  The
          // boundedness is ensured by the intersection of these constraints
          // with the convex sets (on the vertices).
          AddPerspectiveConstraint(&prog, b, vars);
        }
      }
    }
  }

  MathematicalProgramResult result = Solve(prog, options);
  log()->info(
      "Solved GCS shortest path using {} with convex_relaxation={} and "
      "preprocessing={}{}.",
      result.get_solver_id().name(), *options.convex_relaxation,
      *options.preprocessing,
      *options.convex_relaxation && *options.max_rounded_paths > 0
          ? " and rounding"
          : " and no rounding");

  bool found_rounded_result = false;
  // Implements the rounding scheme put forth in Section 4.2 of
  // "Motion Planning around Obstacles with Convex Optimization":
  // https://arxiv.org/abs/2205.04422
  if (*options.convex_relaxation && *options.max_rounded_paths > 0 &&
      result.is_success()) {
    DRAKE_THROW_UNLESS(options.max_rounding_trials > 0);

    RandomGenerator generator(options.rounding_seed);
    std::uniform_real_distribution<double> uniform;
    std::vector<std::vector<const Edge*>> paths;
    std::map<EdgeId, double> flows;
    for (const auto& [edge_id, e] : edges_) {
      if (!e->phi_value_.value_or(true) || unusable_edges.contains(edge_id)) {
        flows.emplace(edge_id, 0);
      } else {
        flows.emplace(edge_id, result.GetSolution(relaxed_phi[edge_id]));
      }
    }
    int num_trials = 0;
    MathematicalProgramResult best_rounded_result;
    while (static_cast<int>(paths.size()) < *options.max_rounded_paths &&
           num_trials < options.max_rounding_trials) {
      ++num_trials;

      // Find candidate path by traversing the graph with a depth first search
      // where edges are taken with probability proportional to their flow.
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
          // Since this code requires result.is_success() to be true, we should
          // always have a path. We assert that..
          DRAKE_ASSERT(path_vertex_ids.size() > 0);
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
      MathematicalProgramResult rounded_result =
          SolveConvexRestriction(new_path, options);

      // Check path quality.
      if (rounded_result.is_success() &&
          (!best_rounded_result.is_success() ||
           rounded_result.get_optimal_cost() <
               best_rounded_result.get_optimal_cost())) {
        best_rounded_result = rounded_result;
      } else {
        // In the event that all rounded results are infeasible, we still want
        // to propagate the solver id for logging.
        best_rounded_result.set_solver_id(rounded_result.get_solver_id());
      }
    }
    if (best_rounded_result.is_success()) {
      result = best_rounded_result;
      found_rounded_result = true;
    } else {
      result.set_solution_result(SolutionResult::kIterationLimit);
      result.set_solver_id(best_rounded_result.get_solver_id());
    }
    log()->info("Finished {} rounding trials with {}.", num_trials,
                result.get_solver_id().name());
  }
  if (!found_rounded_result) {
    // Push the placeholder variables and excluded edge variables into the
    // result, so that they can be accessed as if they were variables included
    // in the optimization.
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
              *options.convex_relaxation ? relaxed_phi.at(e->id()) : e->phi_);
        }
      }
      // In the convex relaxation, sum_relaxed_phi may not be one even for
      // vertices in the shortest path. We undo yₑ = ϕₑ xᵤ here to ensure that
      // xᵤ is in v->set(). If ∑ ϕₑ is small enough that numerical errors
      // prevent the projection back into the Xᵤ, then we prefer to return NaN.
      if (sum_phi < 100.0 * std::numeric_limits<double>::epsilon()) {
        x_v = VectorXd::Constant(v->ambient_dimension(),
                                 std::numeric_limits<double>::quiet_NaN());
      } else if (*options.convex_relaxation) {
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
    if (*options.convex_relaxation) {
      // Write the value of the relaxed phi into the phi placeholder.
      for (const auto& [edge_id, relaxed_phi_var] : relaxed_phi) {
        decision_variable_index.emplace(edges_.at(edge_id)->phi_.get_id(),
                                        count);
        x_val[count++] = result.GetSolution(relaxed_phi_var);
      }
    }
    result.set_decision_variable_index(decision_variable_index);
    result.set_x_val(x_val);
  }

  return result;
}

std::vector<std::vector<const Edge*>> GraphOfConvexSets::SamplePaths(
    const Vertex& source, const Vertex& target,
    const solvers::MathematicalProgramResult& result,
    const GraphOfConvexSetsOptions& options) const {
  if (!result.is_success()) {
    throw std::runtime_error(
        "Cannot sample paths when result.is_success() is false.");
  }
  if (vertices_.count(source.id()) == 0) {
    throw std::invalid_argument(fmt::format(
        "Source vertex {} is not a vertex in this GraphOfConvexSets.",
        source.name()));
  }
  if (vertices_.count(target.id()) == 0) {
    throw std::invalid_argument(fmt::format(
        "Target vertex {} is not a vertex in this GraphOfConvexSets.",
        target.name()));
  }

  VertexId source_id = source.id();
  VertexId target_id = target.id();

  std::set<EdgeId> unusable_edges;
  if (*options.preprocessing) {
    unusable_edges = PreprocessShortestPath(source_id, target_id, options);
  }

  if (*options.convex_relaxation && *options.max_rounded_paths > 0 &&
      result.is_success()) {
    DRAKE_THROW_UNLESS(options.max_rounding_trials > 0);

    RandomGenerator generator(options.rounding_seed);
    std::uniform_real_distribution<double> uniform;
    std::vector<std::vector<const Edge*>> paths;
    std::map<EdgeId, double> flows;
    for (const auto& [edge_id, e] : edges_) {
      if (!e->phi_value_.value_or(true) || unusable_edges.contains(edge_id)) {
        flows.emplace(edge_id, 0);
      } else {
        flows.emplace(edge_id, result.GetSolution(e->phi()));
      }
    }
    int num_trials = 0;
    MathematicalProgramResult best_rounded_result;
    while (static_cast<int>(paths.size()) < *options.max_rounded_paths &&
           num_trials < options.max_rounding_trials) {
      ++num_trials;

      // Find candidate path by traversing the graph with a depth first search
      // where edges are taken with probability proportional to their flow.
      std::vector<VertexId> visited_vertex_ids{source.id()};
      std::vector<const Vertex*> path_vertices{&source};
      std::vector<const Edge*> new_path;
      while (path_vertices.back()->id() != target_id) {
        std::vector<const Edge*> candidate_edges;
        for (const Edge* e : path_vertices.back()->outgoing_edges()) {
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
          path_vertices.pop_back();
          new_path.pop_back();
          // Since this code requires result.is_success() to be true, we should
          // always have a path. We assert that..
          DRAKE_ASSERT(path_vertices.size() > 0);
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
            path_vertices.push_back(&candidate_edges[ii]->v());
            new_path.emplace_back(candidate_edges[ii]);
            break;
          }
        }
      }

      if (std::find(paths.begin(), paths.end(), new_path) != paths.end()) {
        continue;
      }
      paths.push_back(new_path);
    }
    return paths;
  }
  throw std::runtime_error(
      fmt::format("Can not sample paths when convex relaxation is false or "
                  "max_rounded_paths is 0.",
                  target.name()));
}

std::vector<const Edge*> GraphOfConvexSets::GetSolutionPath(
    const Vertex& source, const Vertex& target,
    const solvers::MathematicalProgramResult& result, double tolerance) const {
  if (!result.is_success()) {
    throw std::runtime_error(
        "Cannot extract a solution path when result.is_success() is false.");
  }
  if (!vertices_.contains(source.id())) {
    throw std::invalid_argument(fmt::format(
        "Source vertex {} is not a vertex in this GraphOfConvexSets.",
        source.name()));
  }
  if (!vertices_.contains(target.id())) {
    throw std::invalid_argument(fmt::format(
        "Target vertex {} is not a vertex in this GraphOfConvexSets.",
        target.name()));
  }
  DRAKE_THROW_UNLESS(tolerance >= 0);
  std::vector<const Edge*> path_edges;

  // Extract the path by traversing the graph with a depth first search.
  std::unordered_set<const Vertex*> visited_vertices{&source};
  std::vector<const Vertex*> path_vertices{&source};
  while (path_vertices.back() != &target) {
    // Find the edge with the maximum flow from the current node.
    double maximum_flow = 0;
    const Vertex* max_flow_vertex{nullptr};
    const Edge* max_flow_edge = nullptr;
    for (const Edge* e : path_vertices.back()->outgoing_edges()) {
      const double flow = result.GetSolution(e->phi());
      // If the edge has not been visited and has a flow greater than the
      // current maximum, then this is our new maximum.
      if (flow >= 1 - tolerance && flow > maximum_flow &&
          !visited_vertices.contains(&e->v())) {
        maximum_flow = flow;
        max_flow_vertex = &e->v();
        max_flow_edge = e;
      }
    }

    if (max_flow_edge == nullptr) {
      // If no candidate edges are found, backtrack to the previous node and
      // continue the search.
      path_vertices.pop_back();
      if (path_vertices.empty()) {
        throw std::runtime_error(fmt::format("No path found from {} to {}.",
                                             source.name(), target.name()));
      }
      path_edges.pop_back();
      continue;
    } else {
      // If we have a maximum flow, then add the vertex/edge to the path and
      // continue the search.
      visited_vertices.insert(max_flow_vertex);
      path_vertices.push_back(max_flow_vertex);
      path_edges.push_back(max_flow_edge);
    }
  }
  return path_edges;
}

namespace {

// TODO(russt): Move this into the public API.

// TODO(russt): Return a map from the removed cost to the new slack variable
// and constraint.

/* Most convex solvers require only support linear and quadratic costs when
operating with nonlinear constraints. This removes costs and adds variables and
constraints as needed by the solvers. */
void RewriteForConvexSolver(MathematicalProgram* prog) {
  // Use Mosek's requirements to test the program attributes.
  solvers::MosekSolver mosek;
  if (mosek.AreProgramAttributesSatisfied(*prog)) {
    return;
  }

  const double kInf = std::numeric_limits<double>::infinity();

  // Loop through all unsupported costs and rewrite them into constraints.
  std::unordered_set<Binding<Cost>> to_remove;

  for (const auto& binding : prog->l2norm_costs()) {
    // N.B. This code is currently only unit tested in
    // gcs_trajectory_optimization_test, not graph_of_convex_sets_test.
    const int m = binding.evaluator()->get_sparse_A().rows();
    const int n = binding.evaluator()->get_sparse_A().cols();
    auto slack = prog->NewContinuousVariables<1>("slack");
    prog->AddLinearCost(Vector1d::Ones(), slack);
    // |Ax+b|² ≤ slack, written as a Lorentz cone with z = [slack; Ax+b].
    MatrixXd A = MatrixXd::Zero(m + 1, n + 1);
    A(0, 0) = 1;
    A.bottomRightCorner(m, n) = binding.evaluator()->get_sparse_A();
    VectorXd b(m + 1);
    b << 0, binding.evaluator()->b();
    prog->AddLorentzConeConstraint(A, b, {slack, binding.variables()});
    to_remove.insert(binding);
  }

  for (const auto& binding : prog->generic_costs()) {
    const Cost* cost = binding.evaluator().get();
    if (const auto* l1c = dynamic_cast<const L1NormCost*>(cost)) {
      const int m = l1c->A().rows();
      const int n = l1c->A().cols();
      auto slack = prog->NewContinuousVariables(m, "slack");
      prog->AddLinearCost(VectorXd::Ones(m), slack);
      // Ax + b ≤ slack, written as [A,-I][x;slack] ≤ -b.
      MatrixXd A = MatrixXd::Zero(m, m + n);
      A << l1c->A(), -MatrixXd::Identity(m, m);
      prog->AddLinearConstraint(A, VectorXd::Constant(m, -kInf), -l1c->b(),
                                {binding.variables(), slack});
      // -(Ax + b) ≤ slack, written as [A,I][x;slack] ≥ -b.
      A.rightCols(m) = MatrixXd::Identity(m, m);
      prog->AddLinearConstraint(A, -l1c->b(), VectorXd::Constant(m, kInf),
                                {binding.variables(), slack});
      to_remove.insert(binding);
    } else if (const auto* linfc = dynamic_cast<const LInfNormCost*>(cost)) {
      const int m = linfc->A().rows();
      const int n = linfc->A().cols();
      auto slack = prog->NewContinuousVariables<1>("slack");
      prog->AddLinearCost(Vector1d::Ones(), slack);
      // ∀i, aᵢᵀx + bᵢ ≤ slack, written as [A,-1][x;slack] ≤ -b.
      MatrixXd A = MatrixXd::Zero(m, n + 1);
      A << linfc->A(), VectorXd::Constant(m, -1);
      prog->AddLinearConstraint(A, VectorXd::Constant(m, -kInf), -linfc->b(),
                                {binding.variables(), slack});
      // ∀i, -(aᵢᵀx + bᵢ) ≤ slack, written as [A,1][x;slack] ≥ -b.
      A.col(A.cols() - 1) = VectorXd::Ones(m);
      prog->AddLinearConstraint(A, -linfc->b(), VectorXd::Constant(m, kInf),
                                {binding.variables(), slack});
      to_remove.insert(binding);
    } else if (const auto* pqc =
                   dynamic_cast<const PerspectiveQuadraticCost*>(cost)) {
      const int m = pqc->A().rows();
      const int n = pqc->A().cols();
      auto slack = prog->NewContinuousVariables<1>("slack");
      prog->AddLinearCost(Vector1d::Ones(), slack);
      // Written as rotated Lorentz cone with z = [slack; Ax+b].
      MatrixXd A = MatrixXd::Zero(m + 1, n + 1);
      A(0, 0) = 1;
      A.bottomRightCorner(m, n) = pqc->A();
      VectorXd b(m + 1);
      b << 0, pqc->b();
      prog->AddRotatedLorentzConeConstraint(A, b, {slack, binding.variables()});
      to_remove.insert(binding);
    }
  }
  for (const auto& b : to_remove) {
    prog->RemoveCost(b);
  }
}

}  // namespace

MathematicalProgramResult GraphOfConvexSets::SolveConvexRestriction(
    const std::vector<const Edge*>& active_edges,
    const GraphOfConvexSetsOptions& options) const {
  // Use the restriction solver and options if they are provided.
  GraphOfConvexSetsOptions restriction_options = options;
  if (restriction_options.restriction_solver) {
    restriction_options.solver = restriction_options.restriction_solver;
  }
  if (restriction_options.restriction_solver_options) {
    restriction_options.solver_options =
        *restriction_options.restriction_solver_options;
  }
  MathematicalProgram prog;

  std::set<const Vertex*, VertexIdComparator> vertices;
  for (const auto* e : active_edges) {
    if (!edges_.contains(e->id())) {
      throw std::runtime_error(
          fmt::format("Edge {} is not in the graph.", e->name()));
    }
    vertices.emplace(&e->u());
    vertices.emplace(&e->v());
  }

  for (const auto* v : vertices) {
    if (v->set().ambient_dimension() == 0) {
      continue;
    }
    prog.AddDecisionVariables(v->x());
    v->set().AddPointInSetConstraints(&prog, v->x());

    // Vertex costs.
    for (const Binding<Cost>& b : v->costs_) {
      prog.AddCost(b);
    }
    // Vertex constraints.
    for (const auto& [b, transcriptions] : v->constraints_) {
      if (transcriptions.contains(Transcription::kRestriction)) {
        prog.AddConstraint(b);
      }
    }
  }

  for (const auto* e : active_edges) {
    // Edge costs.
    for (const Binding<Cost>& b : e->costs_) {
      prog.AddCost(b);
    }
    // Edge constraints.
    for (const auto& [b, transcriptions] : e->constraints_) {
      if (transcriptions.contains(Transcription::kRestriction)) {
        prog.AddConstraint(b);
      }
    }
  }

  RewriteForConvexSolver(&prog);
  MathematicalProgramResult result = Solve(prog, restriction_options);

  // TODO(russt): Add the dual variables back in for the rewritten costs.

  // Add phi vars.
  int num_excluded_vars = edges_.size();
  // Add any excluded vertices to the result.
  std::vector<const Vertex*> excluded_vertices;
  for (const auto& pair : vertices_) {
    const Vertex* v = pair.second.get();
    if (!vertices.contains(v)) {
      num_excluded_vars += v->x().size();
      excluded_vertices.emplace_back(v);
    }
  }
  int count = result.get_x_val().size();
  Eigen::VectorXd x_val(count + num_excluded_vars);
  x_val.head(count) = result.get_x_val();
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index =
      prog.decision_variable_index();
  for (const auto& pair : edges_) {
    const Edge* e = pair.second.get();
    decision_variable_index.emplace(e->phi_.get_id(), count);
    x_val[count++] = std::find(active_edges.begin(), active_edges.end(), e) !=
                             active_edges.end()
                         ? 1.0
                         : 0.0;
  }
  for (const Vertex* v : excluded_vertices) {
    for (int i = 0; i < v->x().size(); ++i) {
      decision_variable_index.emplace(v->x()[i].get_id(), count);
      x_val[count++] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  result.set_decision_variable_index(decision_variable_index);
  result.set_x_val(x_val);

  return result;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
