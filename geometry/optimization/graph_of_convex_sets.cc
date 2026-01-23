// TODO(jwnimmer-tri) Port GetGraphvizString to fmt, once we have sufficient
// options there to control precision and scientific formatting.
#undef EIGEN_NO_IO

#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/parallelism.h"
#include "drake/common/text_logging.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/get_program_type.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

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
using solvers::SolveInParallel;
using solvers::VariableRefList;
using solvers::VectorXDecisionVariable;
using solvers::internal::CreateBinding;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

namespace {
MathematicalProgramResult SolveMainProgram(
    const MathematicalProgram& prog, const GraphOfConvexSetsOptions& options) {
  MathematicalProgramResult result;
  // Solving the main GCS program.
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

      // TODO(russt): Consider calling MixedIntegerBranchAndBound
      // automatically here. The small trick is that we need to pass the
      // SolverId into that constructor manually, and ChooseBestSolver doesn't
      // make it easy to figure out what the best solver would be if we
      // removed the integer variables.

      throw std::runtime_error(
          "GraphOfConvexSets: There is no solver available that can solve "
          "the "
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

Binding<Cost> Vertex::AddCost(
    const symbolic::Expression& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  return AddCost(solvers::internal::ParseCost(e), use_in_transcription);
}

Binding<Cost> Vertex::AddCost(
    const Binding<Cost>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DRAKE_THROW_UNLESS(
      Variables(binding.variables()).IsSubsetOf(Variables(placeholder_x_)));
  DRAKE_THROW_UNLESS(use_in_transcription.size() > 0);
  const int n = ell_.size();
  ell_.conservativeResize(n + 1);
  ell_[n] = Variable(fmt::format("v_ell{}", n), Variable::Type::CONTINUOUS);
  costs_.push_back({binding, use_in_transcription});
  // Note: The ell_ variable is a slack variable used e.g. SolveShortestPath,
  // but not e.g. SolveConvexRestriction. It is an implementation detail, and
  // should not leak into the public interface, otherwise people could expect
  // to call e.g. prog.GetSolution(ell_), and get different answers based on
  // which solution method they used.
  return binding;
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

std::vector<solvers::Binding<solvers::Cost>> Vertex::GetCosts(
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        used_in_transcription) const {
  DRAKE_THROW_UNLESS(used_in_transcription.size() > 0);
  std::vector<solvers::Binding<solvers::Cost>> costs;
  // Add all costs that are used in the transcription.
  for (const auto& [binding, transcriptions] : costs_) {
    if (std::any_of(transcriptions.begin(), transcriptions.end(),
                    [&used_in_transcription](const auto& elem) {
                      return used_in_transcription.contains(elem);
                    })) {
      costs.push_back(binding);
    }
  }
  return costs;
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

std::optional<double> Vertex::GetSolutionCost(
    const MathematicalProgramResult& result) const {
  if (!result.get_decision_variable_index()) {
    // Then there were no results.
    return std::nullopt;
  }
  double sum = 0.0;
  for (int i = 0; i < ssize(ell_); ++i) {
    if (result.get_decision_variable_index()->contains(ell_[i].get_id())) {
      sum += result.GetSolution(ell_[i]);
    }
  }
  return sum;
}

std::optional<double> Vertex::GetSolutionCost(
    const MathematicalProgramResult& result,
    const solvers::Binding<solvers::Cost>& cost) const {
  if (!result.get_decision_variable_index()) {
    // Then there were no results.
    return std::nullopt;
  }
  for (int i = 0; i < ssize(costs_); ++i) {
    if (costs_[i].first == cost) {
      if (result.get_decision_variable_index()->contains(ell_[i].get_id())) {
        return result.GetSolution(ell_[i]);
      } else {
        return 0.0;
      }
    }
  }
  throw std::runtime_error(fmt::format(
      "Vertex::GetSolutionCost: cost {} was not registered with this vertex.",
      cost.to_string()));
}

std::optional<VectorXd> Vertex::GetSolution(
    const MathematicalProgramResult& result) const {
  if (result.get_decision_variable_index() &&
      result.get_decision_variable_index()->contains(
          placeholder_x_[0].get_id())) {
    VectorXd x = result.GetSolution(placeholder_x_);
    // SolveShortestPath writes NaN into the solution if Phi is approximately
    // zero, so we handle that case here.
    if (x.array().isNaN().any()) {
      return std::nullopt;
    }
    return x;
  } else {
    return std::nullopt;
  }
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

VectorXDecisionVariable Edge::NewSlackVariables(int rows,
                                                const std::string& name = "s") {
  auto s = symbolic::MakeVectorContinuousVariable(rows, name_ + "_" + name);

  const int n = slacks_.size();
  slacks_.conservativeResize(n + rows);
  slacks_.tail(rows) = s;

  for (int i = 0; i < rows; ++i) {
    allowed_vars_.insert(s[i]);
  }

  // Add this slack variable to the x_to_yz map, so that it can
  // be looked up like any other variable in the vertices.
  for (int i = 0; i < rows; ++i) {
    x_to_yz_.emplace(s[i], s[i]);
  }

  return s;
}

Binding<Cost> Edge::AddCost(
    const symbolic::Expression& e,
    const std::unordered_set<Transcription>& use_in_transcription) {
  return AddCost(solvers::internal::ParseCost(e), use_in_transcription);
}

Binding<Cost> Edge::AddCost(
    const Binding<Cost>& binding,
    const std::unordered_set<Transcription>& use_in_transcription) {
  DRAKE_THROW_UNLESS(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
  DRAKE_THROW_UNLESS(use_in_transcription.size() > 0);
  const int n = ell_.size();
  ell_.conservativeResize(n + 1);
  ell_[n] =
      Variable(fmt::format("{}ell{}", name_, n), Variable::Type::CONTINUOUS);
  costs_.push_back({binding, use_in_transcription});
  // Note: The ell_ variable is a slack variable used e.g. SolveShortestPath,
  // but not e.g. SolveConvexRestriction. It is an implementation detail, and
  // should not leak into the public interface, otherwise people could expect
  // to call e.g. prog.GetSolution(ell_), and get different answers based on
  // which solution method they used.
  return binding;
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

std::vector<solvers::Binding<solvers::Cost>> Edge::GetCosts(
    const std::unordered_set<GraphOfConvexSets::Transcription>&
        used_in_transcription) const {
  DRAKE_THROW_UNLESS(used_in_transcription.size() > 0);
  std::vector<solvers::Binding<solvers::Cost>> costs;
  // Add all costs that are used in the transcription.
  for (const auto& [binding, transcriptions] : costs_) {
    if (std::any_of(transcriptions.begin(), transcriptions.end(),
                    [&used_in_transcription](const auto& elem) {
                      return used_in_transcription.contains(elem);
                    })) {
      costs.push_back(binding);
    }
  }
  return costs;
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

std::optional<double> Edge::GetSolutionCost(
    const MathematicalProgramResult& result) const {
  if (!result.get_decision_variable_index()) {
    // Then there were no results.
    return std::nullopt;
  }
  double sum = 0.0;
  for (int i = 0; i < ssize(ell_); ++i) {
    if (result.get_decision_variable_index()->contains(ell_[i].get_id())) {
      sum += result.GetSolution(ell_[i]);
    }
  }
  return sum;
}

std::optional<double> Edge::GetSolutionCost(
    const MathematicalProgramResult& result,
    const solvers::Binding<solvers::Cost>& cost) const {
  if (!result.get_decision_variable_index()) {
    // Then there were no results.
    return std::nullopt;
  }
  for (int i = 0; i < ssize(costs_); ++i) {
    if (costs_[i].first == cost) {
      if (result.get_decision_variable_index()->contains(ell_[i].get_id())) {
        return result.GetSolution(ell_[i]);
      } else {
        return 0.0;
      }
    }
  }
  throw std::runtime_error(fmt::format(
      "Edge::GetSolutionCost: cost {} was not registered with this edge.",
      cost.to_string()));
}

std::optional<Eigen::VectorXd> Edge::GetSolutionPhiXu(
    const solvers::MathematicalProgramResult& result) const {
  if (result.get_decision_variable_index() &&
      result.get_decision_variable_index()->contains(y_[0].get_id())) {
    return result.GetSolution(y_);
  } else {
    return std::nullopt;
  }
}

std::optional<Eigen::VectorXd> Edge::GetSolutionPhiXv(
    const solvers::MathematicalProgramResult& result) const {
  if (result.get_decision_variable_index() &&
      result.get_decision_variable_index()->contains(z_[0].get_id())) {
    return result.GetSolution(z_);
  } else {
    return std::nullopt;
  }
}

std::unique_ptr<GraphOfConvexSets> GraphOfConvexSets::Clone() const {
  auto clone = std::make_unique<GraphOfConvexSets>();
  // Map from original vertices to cloned vertices.
  std::unordered_map<const Vertex*, Vertex*> original_to_cloned;

  for (const auto& [v_id, v] : vertices_) {
    original_to_cloned.emplace(v.get(), clone->AddVertexFromTemplate(*v));
  }
  for (const auto& [e_id, e] : edges_) {
    clone->AddEdgeFromTemplate(original_to_cloned.at(&e->u()),
                               original_to_cloned.at(&e->v()), *e);
  }
  return clone;
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

namespace {

template <typename C>
std::vector<std::pair<solvers::Binding<C>, std::unordered_set<Transcription>>>
ReplaceVariables(
    const std::vector<std::pair<Binding<C>, std::unordered_set<Transcription>>>&
        bindings,
    const std::unordered_map<Variable::Id, Variable>& subs) {
  std::vector<std::pair<Binding<C>, std::unordered_set<Transcription>>>
      new_bindings;
  for (const auto& pair : bindings) {
    const Binding<C>& binding = pair.first;
    const std::unordered_set<Transcription>& transcriptions = pair.second;
    VectorX<Variable> new_vars(ssize(binding.variables()));
    for (int i = 0; i < ssize(binding.variables()); ++i) {
      auto it = subs.find(binding.variables()[i].get_id());
      if (it == subs.end()) {
        throw std::runtime_error(fmt::format(
            "ReplaceVariables: variable {} (id: {}) was not found in the "
            "substitution map. Probably the Drake implementation of "
            "AddVertexFromTemplate or AddEdgeFromTemplate needs to be updated.",
            binding.variables()[i].get_name(),
            binding.variables()[i].get_id()));
      }
      new_vars[i] = it->second;
    }
    new_bindings.push_back(std::make_pair(
        Binding<C>(binding.evaluator(), new_vars), transcriptions));
  }
  return new_bindings;
}

}  // namespace

Vertex* GraphOfConvexSets::AddVertexFromTemplate(const Vertex& v) {
  Vertex* v_new = AddVertex(v.set(), v.name());
  v_new->ell_ = symbolic::MakeVectorContinuousVariable(
      v.ell_.size(), "v_ell");  // These would normally be created by AddCost.
  std::unordered_map<Variable::Id, Variable> subs;
  for (int i = 0; i < ssize(v.ell_); ++i) {
    subs.emplace(v.ell_[i].get_id(), v_new->ell_[i]);
  }
  for (int i = 0; i < ssize(v.placeholder_x_); ++i) {
    subs.emplace(v.placeholder_x_[i].get_id(), v_new->placeholder_x_[i]);
  }
  v_new->costs_ = ReplaceVariables(v.costs_, subs);
  v_new->constraints_ = ReplaceVariables(v.constraints_, subs);
  return v_new;
}

Edge* GraphOfConvexSets::AddEdge(Vertex* u, Vertex* v, std::string name) {
  DRAKE_DEMAND(u != nullptr && IsValid(*u));
  DRAKE_DEMAND(v != nullptr && IsValid(*v));
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

Edge* GraphOfConvexSets::AddEdgeFromTemplate(Vertex* u, Vertex* v,
                                             const Edge& e) {
  DRAKE_DEMAND(u != nullptr && IsValid(*u));
  DRAKE_DEMAND(v != nullptr && IsValid(*v));

  Edge* e_new = AddEdge(u, v, e.name());
  e_new->ell_ = symbolic::MakeVectorContinuousVariable(
      e.ell_.size(),
      e.name() + "ell");  // These would normally be created by AddCost.
  std::unordered_map<Variable::Id, Variable> subs;
  subs.emplace(e.phi_.get_id(), e_new->phi_);
  for (int i = 0; i < ssize(e.ell_); ++i) {
    subs.emplace(e.ell_[i].get_id(), e_new->ell_[i]);
  }
  for (int i = 0; i < ssize(e.xu()); ++i) {
    subs.emplace(e.xu()[i].get_id(), e_new->xu()[i]);
  }
  for (int i = 0; i < ssize(e.xv()); ++i) {
    subs.emplace(e.xv()[i].get_id(), e_new->xv()[i]);
  }
  if (e.slacks_.size() > 0) {
    // Slacks get created and inserted into the other variable lists in a way
    // that's slightly annoying to deal with. We can add this once it's needed.
    throw std::runtime_error(
        "AddEdgeFromTemplate: templates with slack variables are not supported "
        "yet (but could be).");
  }
  e_new->costs_ = ReplaceVariables(e.costs_, subs);
  e_new->constraints_ = ReplaceVariables(e.constraints_, subs);
  e_new->phi_value_ = e.phi_value_;
  return e_new;
}

const Vertex* GraphOfConvexSets::GetVertexByName(
    const std::string& name) const {
  for (const auto& [v_id, v] : vertices_) {
    if (v->name() == name) {
      return v.get();
    }
  }
  return nullptr;
}

Vertex* GraphOfConvexSets::GetMutableVertexByName(const std::string& name) {
  for (auto& [v_id, v] : vertices_) {
    if (v->name() == name) {
      return v.get();
    }
  }
  return nullptr;
}

const Edge* GraphOfConvexSets::GetEdgeByName(const std::string& name) const {
  for (const auto& [e_id, e] : edges_) {
    if (e->name() == name) {
      return e.get();
    }
  }
  return nullptr;
}

Edge* GraphOfConvexSets::GetMutableEdgeByName(const std::string& name) {
  for (auto& [e_id, e] : edges_) {
    if (e->name() == name) {
      return e.get();
    }
  }
  return nullptr;
}

void GraphOfConvexSets::RemoveVertex(Vertex* vertex) {
  DRAKE_THROW_UNLESS(vertex != nullptr);
  VertexId vertex_id = vertex->id();
  DRAKE_THROW_UNLESS(vertices_.contains(vertex_id));
  for (const auto uv : vertex->incoming_edges_) {
    uv->u().RemoveOutgoingEdge(uv);
    edges_.erase(uv->id());
  }
  for (const auto vw : vertex->outgoing_edges_) {
    vw->v().RemoveIncomingEdge(vw);
    edges_.erase(vw->id());
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

bool GraphOfConvexSets::IsValid(const Vertex& v) const {
  if (vertices_.contains(v.id())) {
    DRAKE_DEMAND(vertices_.at(v.id()).get() == &v);
    return true;
  }
  return false;
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

bool GraphOfConvexSets::IsValid(const Edge& e) const {
  if (edges_.contains(e.id())) {
    DRAKE_DEMAND(edges_.at(e.id()).get() == &e);
    return true;
  }
  return false;
}

void GraphOfConvexSets::ClearAllPhiConstraints() {
  for (const auto& e : edges_) {
    e.second->ClearPhiConstraints();
  }
}

std::string GraphOfConvexSets::GetGraphvizString(
    const solvers::MathematicalProgramResult* result,
    const GcsGraphvizOptions& options,
    const std::vector<const Edge*>* active_path) const {
  // This function converts the range (0.0, 1.0) to Hex strings in the range
  // (20, FF).
  const int kMaxHexValue = 255;
  const int kMinHexValue = 32;
  auto floatToHex = [](float value) -> std::string {
    if (value < 0.0f || value > 1.0f) return "Out of range";
    std::ostringstream ss;
    ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
       << static_cast<int>(value * (kMaxHexValue - kMinHexValue) +
                           kMinHexValue);
    return ss.str();
  };

  // Note: We use stringstream instead of fmt in order to control the
  // formatting of the Eigen output and double output in a consistent way.
  std::stringstream graphviz;
  graphviz.precision(options.precision);
  if (!options.scientific) graphviz << std::fixed;
  graphviz << "digraph GraphOfConvexSets {\n";
  graphviz << "labelloc=t;\n";
  for (const auto& [v_id, v] : vertices_) {
    graphviz << "v" << to_string(v_id) << " [label=\"" << v->name();
    if (result) {
      if (options.show_vars) {
        std::optional<VectorXd> x = v->GetSolution(*result);
        if (x.has_value()) {
          graphviz << "\nx = [" << x->transpose() << "]";
        }
      }
      if (options.show_costs) {
        std::optional<double> cost = v->GetSolutionCost(*result);
        if (cost.has_value()) {
          graphviz << "\ncost = " << *cost;
        }
      }
    }
    graphviz << "\"]\n";
  }
  for (const auto& [e_id, e] : edges_) {
    unused(e_id);
    graphviz << "v" << to_string(e->u().id()) << " -> v"
             << to_string(e->v().id());
    graphviz << " [label=\"" << e->name();
    if (result) {
      if (options.show_costs) {
        std::optional<double> cost = e->GetSolutionCost(*result);
        if (cost.has_value()) {
          graphviz << "\ncost = " << *cost;
        }
      }
      if (options.show_slacks && result->get_decision_variable_index()) {
        graphviz << "\n";
        std::optional<VectorXd> phixu = e->GetSolutionPhiXu(*result);
        if (phixu.has_value()) {
          graphviz << "ϕ xᵤ = [" << phixu->transpose() << "],\n";
        }
        std::optional<VectorXd> phixv = e->GetSolutionPhiXv(*result);
        if (phixv.has_value()) {
          graphviz << "ϕ xᵥ = [" << phixv->transpose() << "]";
        }
      }
      if (options.show_flows) {
        graphviz << "\n";
        graphviz << "ϕ = " << result->GetSolution(e->phi());
        graphviz << "\"";
        // Note: This must be last, because it also sets the color parameter
        // of the edge (and hence must close the name within quote-marks)
        graphviz << ", color=" << "\"#000000"
                 << floatToHex(result->GetSolution(e->phi()));
      }
    }
    graphviz << "\"];\n";
  }

  if (active_path) {
    for (const auto& e : *active_path) {
      graphviz << "v" << to_string(e->u().id()) << " -> v"
               << to_string(e->v().id());
      graphviz << " [label=\"" << e->name() << " = active\"";
      graphviz << ", color="
               << "\"#ff0000\"";
      graphviz << ", style=\"dashed\"";
      graphviz << "];\n";
    }
  }
  graphviz << "}\n";
  return graphviz.str();
}

std::unique_ptr<MathematicalProgram>
GraphOfConvexSets::ConstructPreprocessingProgram(
    EdgeId edge_id, const std::map<VertexId, std::vector<int>>& incoming_edges,
    const std::map<VertexId, std::vector<int>>& outgoing_edges,
    VertexId source_id, VertexId target_id) const {
  const auto& e = edges_.at(edge_id);
  int nE = edges_.size();
  std::unique_ptr<MathematicalProgram> prog =
      std::make_unique<MathematicalProgram>();

  // Flow for each edge is between 0 and 1 for both paths.
  VectorXDecisionVariable f = prog->NewContinuousVariables(nE, "flow_su");
  Binding<solvers::BoundingBoxConstraint> f_limits =
      prog->AddBoundingBoxConstraint(0, 1, f);
  VectorXDecisionVariable g = prog->NewContinuousVariables(nE, "flow_vt");
  Binding<solvers::BoundingBoxConstraint> g_limits =
      prog->AddBoundingBoxConstraint(0, 1, g);

  std::map<VertexId, Binding<LinearEqualityConstraint>> conservation_f;
  std::map<VertexId, Binding<LinearEqualityConstraint>> conservation_g;
  std::map<VertexId, Binding<LinearConstraint>> degree;
  for (const auto& [vertex_id, v] : vertices_) {
    auto maybe_Ev_in = incoming_edges.find(vertex_id);
    std::vector<int> Ev_in = maybe_Ev_in != incoming_edges.end()
                                 ? maybe_Ev_in->second
                                 : std::vector<int>{};
    auto maybe_Ev_out = outgoing_edges.find(vertex_id);
    std::vector<int> Ev_out = maybe_Ev_out != outgoing_edges.end()
                                  ? maybe_Ev_out->second
                                  : std::vector<int>{};
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
            {vertex_id, prog->AddLinearEqualityConstraint(A_flow, -1, fv)});
      } else {
        conservation_f.insert(
            {vertex_id, prog->AddLinearEqualityConstraint(A_flow, 0, fv)});
      }

      // Conservation of flow for g: ∑ g_in - ∑ g_out = δ(is_target).
      if (vertex_id == target_id) {
        conservation_g.insert(
            {vertex_id, prog->AddLinearEqualityConstraint(A_flow, 1, gv)});
      } else {
        conservation_g.insert(
            {vertex_id, prog->AddLinearEqualityConstraint(A_flow, 0, gv)});
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
          {vertex_id, prog->AddLinearConstraint(A_degree, 0, 1, fgin)});
    }
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
  if (degree.contains(e->v().id())) {
    degree.at(e->v().id()).evaluator()->set_bounds(Vector1d(0), Vector1d(0));
  }

  return prog;
}

// Implements the preprocessing scheme put forth in Appendix A.2 of
// "Motion Planning around Obstacles with Convex Optimization":
// https://arxiv.org/abs/2205.04422
std::set<EdgeId> GraphOfConvexSets::PreprocessShortestPath(
    VertexId source_id, VertexId target_id,
    const GraphOfConvexSetsOptions& options) const {
  if (!vertices_.contains(source_id)) {
    throw std::runtime_error(fmt::format(
        "Source vertex {} is not a vertex in this GraphOfConvexSets.",
        source_id));
  }
  if (!vertices_.contains(target_id)) {
    throw std::runtime_error(fmt::format(
        "Target vertex {} is not a vertex in this GraphOfConvexSets.",
        target_id));
  }

  std::vector<EdgeId> edge_id_list;
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

      // Note: we only add the edge id to this list if we still need to check
      // it.
      edge_id_list.push_back(edge_id);
    }

    edge_count++;
  }

  // edge_id_list is now the canonical list of edges we will check in the
  // remainder of the function.
  int nE = edge_id_list.size();

  // TODO(cohnt): Rewrite with a parallel for loop where each thread creates and
  // solves the preprocessing program, to avoid having to use batching together
  // with SolveInParallel.

  // Given an edge (u,v) check if a path from source to u and another from v to
  // target exist without sharing edges.

  int preprocessing_parallel_batch_size = 1000;
  int num_batches = 1 + (nE / preprocessing_parallel_batch_size);
  for (int batch_idx = 0; batch_idx < num_batches; ++batch_idx) {
    int this_batch_start = batch_idx * preprocessing_parallel_batch_size;
    int this_batch_end =
        std::min((batch_idx + 1) * preprocessing_parallel_batch_size, nE);
    std::vector<std::unique_ptr<MathematicalProgram>> progs;
    std::vector<EdgeId> idx_to_edge_id;

    int this_batch_nE = this_batch_end - this_batch_start;
    progs.reserve(this_batch_nE);
    idx_to_edge_id.reserve(this_batch_nE);

    for (int i = this_batch_start; i < this_batch_end; ++i) {
      idx_to_edge_id.push_back(edge_id_list.at(i));
    }

    for (int i = 0; i < this_batch_nE; ++i) {
      EdgeId edge_id = idx_to_edge_id.at(i);
      progs.emplace_back(ConstructPreprocessingProgram(
          edge_id, incoming_edges, outgoing_edges, source_id, target_id));
      DRAKE_ASSERT(progs.back()->IsThreadSafe());
    }

    // Check if edge e = (u,v) could be on a path from start to goal.
    std::vector<const MathematicalProgram*> prog_ptrs(progs.size());
    for (int i = 0; i < ssize(progs); ++i) {
      prog_ptrs[i] = progs[i].get();
    }
    std::optional<solvers::SolverId> maybe_solver_id;
    solvers::SolverOptions preprocessing_solver_options =
        options.preprocessing_solver_options.value_or(options.solver_options);
    if (options.preprocessing_solver) {
      maybe_solver_id = options.preprocessing_solver->solver_id();
    } else if (options.solver) {
      maybe_solver_id = options.solver->solver_id();
    } else {
      maybe_solver_id = std::nullopt;
    }

    std::vector<MathematicalProgramResult> results =
        SolveInParallel(prog_ptrs, nullptr, &preprocessing_solver_options,
                        maybe_solver_id, options.parallelism, false);

    for (int i = 0; i < this_batch_nE; ++i) {
      const auto& result = results.at(i);
      if (!result.is_success()) {
        unusable_edges.insert(idx_to_edge_id.at(i));
      }
    }
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
    const SparseMatrix<double>& A = l2c->get_sparse_A();
    MatrixXd A_cone = MatrixXd::Zero(A.rows() + 1, vars.size());
    A_cone(0, 1) = 1.0;                          // z₀ = ℓ.
    A_cone.block(1, 0, A.rows(), 1) = l2c->b();  // bϕ.
    A_cone.block(1, 2, A.rows(), A.cols()) = A;  // Ax.
    prog->AddLorentzConeConstraint(A_cone, VectorXd::Zero(A_cone.rows()), vars);
  } else if (LInfNormCost* linfc = dynamic_cast<LInfNormCost*>(cost)) {
    // |Ax + b|∞ becomes ℓ ≥ |Aᵢx+bᵢϕ| ∀ i.
    int A_rows = linfc->A().rows();
    MatrixXd A_linear(2 * A_rows, vars.size());
    A_linear.block(0, 0, A_rows, 1) = linfc->b();                        // bϕ.
    A_linear.block(0, 1, A_rows, 1) = -VectorXd::Ones(A_rows);           // -ℓ.
    A_linear.block(0, 2, A_rows, linfc->A().cols()) = linfc->A();        // Ax.
    A_linear.block(A_rows, 0, A_rows, 1) = -linfc->b();                  // -bϕ.
    A_linear.block(A_rows, 1, A_rows, 1) = -VectorXd::Ones(A_rows);      // -ℓ.
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
    } else if (lc->upper_bound().array().isInf().all() &&
               (lc->upper_bound().array() > 0).all()) {
      // If every upper bound is +inf, do nothing.
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
        } else if (lc->upper_bound()[i] < 0) {
          // If the upper bound is -inf, we cannot take the perspective of such
          // a constraint, so we throw an error.
          throw std::runtime_error(
              "Cannot take the perspective of a trivially-infeasible linear "
              "constraint of the form x <= -inf.");
        } else {
          // Do nothing for the constraint x <= inf.
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
    } else if (lc->lower_bound().array().isInf().all() &&
               (lc->lower_bound().array() < 0).all()) {
      // If every lower bound is -inf, do nothing.
    } else {
      // Need to go constraint by constraint.
      const Eigen::MatrixXd& A = lc->GetDenseA();
      RowVectorXd a(vars.size());
      for (int i = 0; i < A.rows(); ++i) {
        if (std::isfinite(lc->lower_bound()[i])) {
          a[0] = -lc->lower_bound()[i];
          a.tail(A.cols()) = A.row(i);
          prog->AddLinearConstraint(a, 0, inf, vars);
        } else if (lc->lower_bound()[i] > 0) {
          // If the lower bound is +inf, we cannot take the perspective of such
          // a constraint, so we throw an error.
          throw std::runtime_error(
              "Cannot take the perspective of a trivially-infeasible linear "
              "constraint of the form x >= +inf.");
        } else {
          // Do nothing for the constraint x >= -inf.
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
  } else if (dynamic_cast<solvers::QuadraticConstraint*>(constraint) !=
             nullptr) {
    // TODO(Alexandre.Amice) Handle convex Quadratic Constraints for the users.
    // Special case the error message for QuadraticConstraint to suggest an easy
    // adjustment to downstream code for users.
    throw std::runtime_error(fmt::format(
        "ShortestPathProblem::Edge does not support this "
        "binding type: {}. Please add convex quadratic constraints in conic "
        "form by using the function AddQuadraticAsRotatedLorentzConeConstraint",
        binding.to_string()));
  } else {
    throw std::runtime_error(
        fmt::format("ShortestPathProblem::Edge does not support this "
                    "binding type: {}",
                    binding.to_string()));
  }
}

namespace {

bool GcsIsThreadsafe(const GraphOfConvexSets& gcs) {
  // Iterate over vertices to look for non-thread-safe costs and constraints.
  for (const auto& vertex_ptr : gcs.Vertices()) {
    DRAKE_ASSERT(vertex_ptr != nullptr);
    std::vector<Binding<Cost>> restriction_costs =
        vertex_ptr->GetCosts({Transcription::kRestriction});
    for (const auto& binding : restriction_costs) {
      if (!binding.evaluator()->is_thread_safe()) {
        return false;
      }
    }
    std::vector<Binding<Constraint>> restriction_constraints =
        vertex_ptr->GetConstraints({Transcription::kRestriction});
    for (const auto& binding : restriction_constraints) {
      if (!binding.evaluator()->is_thread_safe()) {
        return false;
      }
    }
  }
  // Iterate over edges to look for non-thread-safe costs and constraints.
  for (const auto& edge_ptr : gcs.Edges()) {
    DRAKE_ASSERT(edge_ptr != nullptr);
    std::vector<Binding<Cost>> restriction_costs =
        edge_ptr->GetCosts({Transcription::kRestriction});
    for (const auto& binding : restriction_costs) {
      if (!binding.evaluator()->is_thread_safe()) {
        return false;
      }
    }
    std::vector<Binding<Constraint>> restriction_constraints =
        edge_ptr->GetConstraints({Transcription::kRestriction});
    for (const auto& binding : restriction_constraints) {
      if (!binding.evaluator()->is_thread_safe()) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

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

  auto IncludesCurrentTranscription =
      [&options](
          const std::unordered_set<Transcription>& transcriptions) -> bool {
    return ((*options.convex_relaxation &&
             transcriptions.contains(Transcription::kRelaxation)) ||
            (!*options.convex_relaxation &&
             transcriptions.contains(Transcription::kMIP)));
  };

  std::set<EdgeId> unusable_edges;
  if (*options.preprocessing) {
    unusable_edges = PreprocessShortestPath(source_id, target_id, options);
  }

  MathematicalProgram prog;

  std::map<VertexId, std::vector<Edge*>> incoming_edges;
  std::map<VertexId, std::vector<Edge*>> outgoing_edges;
  std::map<VertexId, std::vector<VectorXDecisionVariable>> vertex_edge_ell;
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
    prog.AddDecisionVariables(e->slacks_);

    // Spatial non-negativity: y ∈ ϕX, z ∈ ϕX.
    if (e->u().ambient_dimension() > 0) {
      e->u().set().AddPointInNonnegativeScalingConstraints(&prog, e->y_, phi);
    }
    if (e->v().ambient_dimension() > 0) {
      e->v().set().AddPointInNonnegativeScalingConstraints(&prog, e->z_, phi);
    }

    // Edge costs.
    for (int i = 0; i < e->ell_.size(); ++i) {
      const auto& [b, transcriptions] = e->costs_[i];
      if (IncludesCurrentTranscription(transcriptions)) {
        prog.AddDecisionVariables(Vector1<Variable>{e->ell_[i]});
        prog.AddLinearCost(VectorXd::Ones(1), Vector1<Variable>{e->ell_[i]});
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
    }

    // Edge constraints.
    for (const auto& [b, transcriptions] : e->constraints_) {
      if (IncludesCurrentTranscription(transcriptions)) {
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
      for (int ii = 0; ii < v->ell_.size(); ++ii) {
        const auto& [b, transcriptions] = v->costs_[ii];
        if (IncludesCurrentTranscription(transcriptions)) {
          VectorXDecisionVariable vertex_ell =
              prog.NewContinuousVariables(cost_edges.size());
          vertex_edge_ell[v->id()].push_back(vertex_ell);
          const VectorXDecisionVariable& old_vars = b.variables();

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
    }

    // Vertex constraints.
    for (const auto& [b, transcriptions] : v->constraints_) {
      if (IncludesCurrentTranscription(transcriptions)) {
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

  MathematicalProgramResult result = SolveMainProgram(prog, options);
  log()->info(
      "Solved GCS shortest path using {} with convex_relaxation={} and "
      "preprocessing={}{}.",
      result.get_solver_id().name(), *options.convex_relaxation,
      *options.preprocessing,
      *options.convex_relaxation && *options.max_rounded_paths > 0
          ? " and rounding"
          : " and no rounding");

  {  // Push the placeholder variables and excluded edge variables into the
    // result, so that they can be accessed as if they were variables included
    // in the optimization.
    int num_placeholder_vars = relaxed_phi.size();
    for (const std::pair<const VertexId, std::unique_ptr<Vertex>>& vpair :
         vertices_) {
      const Vertex* v = vpair.second.get();
      num_placeholder_vars += v->ambient_dimension();
      for (int i = 0; i < v->ell_.size(); ++i) {
        const auto& [b, transcriptions] = v->costs_[i];
        if (IncludesCurrentTranscription(transcriptions)) {
          num_placeholder_vars += 1;
        }
      }
    }
    for (const Edge* e : excluded_edges) {
      num_placeholder_vars += e->y_.size() + e->z_.size() + 1;
    }
    num_placeholder_vars += excluded_phi.size();
    std::unordered_map<symbolic::Variable::Id, int> decision_variable_index =
        prog.decision_variable_index();
    int count = result.get_x_val().size();
    Eigen::VectorXd x_val(count + num_placeholder_vars);
    x_val.head(count) = result.get_x_val();
    for (const Edge* e : excluded_edges) {
      // TODO(russt): Consider not adding y_ and z_ for the excluded edges;
      // GetSolutionPhiXu() and GetSolutionPhiXv() should handle this.
      for (int i = 0; i < e->y_.size(); ++i) {
        decision_variable_index.emplace(e->y_[i].get_id(), count);
        x_val[count++] = 0;
      }
      for (int i = 0; i < e->z_.size(); ++i) {
        decision_variable_index.emplace(e->z_[i].get_id(), count);
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
      int active_ell = 0;
      for (int i = 0; i < v->ell_.size(); ++i) {
        const auto& [b, transcriptions] = v->costs_[i];
        if (IncludesCurrentTranscription(transcriptions)) {
          decision_variable_index.emplace(v->ell_[i].get_id(), count);
          x_val[count++] =
              result.GetSolution(vertex_edge_ell.at(v->id())[active_ell++])
                  .sum();
        }
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
    DRAKE_DEMAND(count == x_val.size());
    result.set_decision_variable_index(decision_variable_index);
    result.set_x_val(x_val);
  }

  // Implements the rounding scheme put forth in Section 4.2 of
  // "Motion Planning around Obstacles with Convex Optimization":
  // https://arxiv.org/abs/2205.04422
  if (*options.convex_relaxation && *options.max_rounded_paths > 0 &&
      result.is_success()) {
    std::unordered_map<const Edge*, double> flows;
    for (const auto& [edge_id, e] : edges_) {
      if (!e->phi_value_.value_or(true) || unusable_edges.contains(edge_id)) {
        flows.emplace(e.get(), 0.0);
      } else {
        flows.emplace(e.get(), result.GetSolution(relaxed_phi[edge_id]));
      }
    }

    std::vector<std::vector<const Edge*>> candidate_paths =
        SamplePaths(source, target, flows, options);

    if (candidate_paths.size() == 0) {
      throw std::runtime_error(
          "GCS rounding failed. The convex relaxation returned a "
          "feasible solution, but there is (definitely) no path from the "
          "source to the target using only edges with flows > "
          "options.flow_tolerance. You can set "
          "options.max_rounded_paths=0 to disable rounding and return "
          "the solution to the relaxation instead.");
    }

    // If any costs or constraints aren't thread-safe, we can't parallelize.
    bool is_thread_safe = GcsIsThreadsafe(*this);
    if (!is_thread_safe) {
      static const logging::Warn log_once(
          "GCS restriction has costs or constraints which are not declared "
          "thread-safe. Any paths being considered in the rounding process "
          "that contain these constraints will be solved sequentially.");
    }

    std::vector<std::unique_ptr<MathematicalProgram>> progs;
    std::vector<const MathematicalProgram*> prog_ptrs;
    for (int i = 0; i < ssize(candidate_paths); ++i) {
      progs.push_back(ConstructRestrictionProgram(candidate_paths[i], &result));
      prog_ptrs.push_back(progs[i].get());
    }

    std::optional<solvers::SolverId> maybe_solver_id = std::nullopt;
    if (options.restriction_solver) {
      maybe_solver_id = options.restriction_solver->solver_id();
    } else if (options.solver) {
      maybe_solver_id = options.solver->solver_id();
    }

    const solvers::SolverOptions* maybe_options =
        options.restriction_solver_options
            ? &(options.restriction_solver_options.value())
            : &(options.solver_options);

    // We use nullptr for the initial guesses, since ConstructRestrictionProgram
    // prepopulates the initial guesses. We use dynamic scheduling, since
    // individual restriction solves may vary in the number of variables and
    // constraints.
    std::vector<MathematicalProgramResult> rounded_results =
        SolveInParallel(prog_ptrs, nullptr /* initial_guesses */,
                        maybe_options /* solver_options */, maybe_solver_id,
                        options.parallelism, true /* dynamic_schedule */);

    constexpr double kInf = std::numeric_limits<double>::infinity();
    double best_cost = kInf;
    int best_result_idx = -1;
    for (int i = 0; i < ssize(rounded_results); ++i) {
      if (rounded_results[i].is_success() &&
          rounded_results[i].get_optimal_cost() < best_cost) {
        best_result_idx = i;
        best_cost = rounded_results[i].get_optimal_cost();
      }
    }

    if (best_cost < kInf) {
      // We found at least one valid result.
      result = rounded_results[best_result_idx];
      MakeRestrictionResultLookLikeMixedInteger(
          *(prog_ptrs[best_result_idx]), &result,
          candidate_paths[best_result_idx]);
    } else {
      // In the event that all rounded results are infeasible, we still want
      // to propagate the solver id for logging.
      result.set_solution_result(SolutionResult::kIterationLimit);
      result.set_solver_id(rounded_results.back().get_solver_id());
    }

    log()->info("Finished {} rounding solutions with {}.",
                candidate_paths.size(), result.get_solver_id().name());
  }

  return result;
}

std::vector<std::vector<const Edge*>> GraphOfConvexSets::SamplePaths(
    const Vertex& source, const Vertex& target,
    const solvers::MathematicalProgramResult& result,
    const GraphOfConvexSetsOptions& options) const {
  std::unordered_map<const Edge*, double> flows;
  for (const auto& [edge_id, e] : edges_) {
    double flow = result.GetSolution(e->phi_);
    if (flow >= options.flow_tolerance) {
      flows.emplace(e.get(), flow);
    }
  }
  return SamplePaths(source, target, flows, options);
}

std::vector<std::vector<const Edge*>> GraphOfConvexSets::SamplePaths(
    const Vertex& source, const Vertex& target,
    const std::unordered_map<const Edge*, double>& flows,
    const GraphOfConvexSetsOptions& options) const {
  DRAKE_THROW_UNLESS(options.max_rounded_paths.value_or(0) > 0);

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

  for (const auto& [edge, flow] : flows) {
    if (edges_.count(edge->id()) == 0) {
      throw std::invalid_argument(fmt::format(
          "Edge with id {} does not exist in this GraphOfConvexSets.",
          edge->id()));
    }
  }

  auto flow_exists_and_above_threshold = [&](const Edge* edge) -> bool {
    auto it = flows.find(edge);
    return it != flows.end() && it->second > options.flow_tolerance;
  };

  RandomGenerator generator(options.rounding_seed);
  std::uniform_real_distribution<double> uniform;
  std::vector<std::vector<const Edge*>> paths;

  int num_trials = 0;
  bool no_feasible_paths = false;
  while (static_cast<int>(paths.size()) < *options.max_rounded_paths &&
         num_trials < options.max_rounding_trials) {
    if (no_feasible_paths) {
      break;
    }

    ++num_trials;

    // Find candidate path by traversing the graph with a depth first search
    // where edges are taken with probability proportional to their flow.
    std::vector<VertexId> visited_vertex_ids{source.id()};
    std::vector<const Edge*> new_path_edges;
    std::vector<const Vertex*> new_path_vertices{&source};
    while (new_path_vertices.back()->id() != target.id()) {
      std::vector<const Edge*> candidate_edges;
      for (const Edge* e : new_path_vertices.back()->outgoing_edges()) {
        if (std::find(visited_vertex_ids.begin(), visited_vertex_ids.end(),
                      e->v().id()) == visited_vertex_ids.end() &&
            flow_exists_and_above_threshold(e)) {
          candidate_edges.emplace_back(e);
        }
      }
      // If the depth first search finds itself at a node with no candidate
      // outbound edges, backtrack to the previous node and continue the
      // search.
      if (candidate_edges.size() == 0) {
        // Remove the vertex with no candidate outgoing edges from the path.
        new_path_vertices.pop_back();

        // There may be no feasible paths
        // (This can for instance happen if the convex relaxation was
        // infeasible, or the provided flows do not correspond to any paths from
        // the source to the target).
        if (new_path_vertices.size() == 0) {
          no_feasible_paths = true;
          break;
        }

        // If there still are vertices in the path, we remove the edge to the
        // vertex with no candidate outgoing vertices and try again.
        new_path_edges.pop_back();
        continue;
      }
      Eigen::VectorXd candidate_flows(candidate_edges.size());
      for (size_t ii = 0; ii < candidate_edges.size(); ++ii) {
        candidate_flows(ii) = flows.at(candidate_edges[ii]);
      }
      // Sample the next edge with probability corresponding to the edge flow
      // (normalized by the sum of all the current outgoing candidate edge
      // flows).
      double edge_sample = uniform(generator) * candidate_flows.sum();
      for (size_t ii = 0; ii < candidate_edges.size(); ++ii) {
        if (edge_sample >= candidate_flows(ii)) {
          edge_sample -= candidate_flows(ii);
        } else {
          visited_vertex_ids.push_back(candidate_edges[ii]->v().id());
          new_path_vertices.push_back(&candidate_edges[ii]->v());
          new_path_edges.emplace_back(candidate_edges[ii]);
          break;
        }
      }
    }

    if (new_path_vertices.size() == 0) {
      continue;
    }

    if (std::find(paths.begin(), paths.end(), new_path_edges) != paths.end()) {
      continue;
    }

    paths.push_back(new_path_edges);
  }

  if (no_feasible_paths) {
    log()->info("Could not find any feasible paths using flow tolerance {}.",
                options.flow_tolerance);
  } else {
    log()->info("Found {} unique paths, discarded {} duplicate paths.",
                paths.size(), num_trials - paths.size());
  }

  return paths;
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

std::unique_ptr<MathematicalProgram>
GraphOfConvexSets::ConstructRestrictionProgram(
    const std::vector<const Edge*>& active_edges,
    const MathematicalProgramResult* initial_guess) const {
  std::unique_ptr<MathematicalProgram> prog =
      std::make_unique<MathematicalProgram>();

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
    prog->AddDecisionVariables(v->x());
    if (initial_guess) {
      prog->SetInitialGuess(v->x(), initial_guess->GetSolution(v->x()));
    }
    v->set().AddPointInSetConstraints(prog.get(), v->x());

    // Vertex costs.
    for (const auto& [b, transcriptions] : v->costs_) {
      if (transcriptions.contains(Transcription::kRestriction)) {
        prog->AddCost(b);
      }
    }
    // Vertex constraints.
    for (const auto& [b, transcriptions] : v->constraints_) {
      if (transcriptions.contains(Transcription::kRestriction)) {
        prog->AddConstraint(b);
      }
    }
  }

  for (const auto* e : active_edges) {
    prog->AddDecisionVariables(e->slacks_);

    // Edge costs.
    for (const auto& [b, transcriptions] : e->costs_) {
      if (transcriptions.contains(Transcription::kRestriction)) {
        prog->AddCost(b);
      }
    }
    // Edge constraints.
    for (const auto& [b, transcriptions] : e->constraints_) {
      if (transcriptions.contains(Transcription::kRestriction)) {
        prog->AddConstraint(b);
      }
    }
  }

  RewriteForConvexSolver(prog.get());

  return prog;
}

void GraphOfConvexSets::MakeRestrictionResultLookLikeMixedInteger(
    const MathematicalProgram& prog, MathematicalProgramResult* result,
    const std::vector<const Edge*>& active_edges) const {
  DRAKE_DEMAND(result != nullptr);
  // TODO(russt): Add the dual variables back in for the rewritten costs.

  // In order to access to the results that is comparable with the other GCS
  // transcriptions, we add a few extra values to the result:
  // - flow variables (phi) for all of the edges,
  // - slack variables corresponding the (active) vertex and edge costs: they
  //   are not used in the restriction, but are the only way to retrieve the
  //   cost from the MIP and/or its convex relaxation.

  std::set<const Vertex*, VertexIdComparator> vertices;
  for (const auto* e : active_edges) {
    if (!edges_.contains(e->id())) {
      throw std::runtime_error(
          fmt::format("Edge {} is not in the graph.", e->name()));
    }
    vertices.emplace(&e->u());
    vertices.emplace(&e->v());
  }

  // Add phi vars for all edges.
  int num_excluded_vars = edges_.size();
  // Add edge cost slack variables for active_edges.
  for (const Edge* e : active_edges) {
    for (int i = 0; i < e->ell_.size(); ++i) {
      const auto& [b, transcriptions] = e->costs_[i];
      if (transcriptions.contains(Transcription::kRestriction)) {
        num_excluded_vars += 1;
      }
    }
  }
  for (const auto* v : vertices) {
    // Add the vertex cost slack variables.
    for (int i = 0; i < v->ell_.size(); ++i) {
      const auto& [b, transcriptions] = v->costs_[i];
      if (transcriptions.contains(Transcription::kRestriction)) {
        num_excluded_vars += 1;
      }
    }
  }
  int count = result->get_x_val().size();
  Eigen::VectorXd x_val(count + num_excluded_vars);
  x_val.head(count) = result->get_x_val();
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index =
      prog.decision_variable_index();
  for (const auto& pair : edges_) {
    const Edge* e = pair.second.get();
    if (std::find(active_edges.begin(), active_edges.end(), e) !=
        active_edges.end()) {
      // phi.
      decision_variable_index.emplace(e->phi_.get_id(), count);
      x_val[count++] = 1.0;
      // edge cost slack variables.
      for (int i = 0; i < e->ell_.size(); ++i) {
        const auto& [b, transcriptions] = e->costs_[i];
        if (transcriptions.contains(Transcription::kRestriction)) {
          decision_variable_index.emplace(e->ell_[i].get_id(), count);
          x_val[count++] = result->EvalBinding(b)[0];
        }
      }
    } else {
      // phi.
      decision_variable_index.emplace(e->phi_.get_id(), count);
      x_val[count++] = 0.0;
    }
  }
  for (const auto* v : vertices) {
    // vertex cost slack variables.
    for (int i = 0; i < v->ell_.size(); ++i) {
      const auto& [b, transcriptions] = v->costs_[i];
      if (transcriptions.contains(Transcription::kRestriction)) {
        decision_variable_index.emplace(v->ell_[i].get_id(), count);
        x_val[count++] = result->EvalBinding(b)[0];
      }
    }
  }
  result->set_decision_variable_index(decision_variable_index);
  result->set_x_val(x_val);
}

MathematicalProgramResult GraphOfConvexSets::SolveConvexRestriction(
    const std::vector<const Edge*>& active_edges,
    const GraphOfConvexSetsOptions& options,
    const MathematicalProgramResult* initial_guess) const {
  std::unique_ptr<MathematicalProgram> prog =
      ConstructRestrictionProgram(active_edges, initial_guess);
  DRAKE_ASSERT(prog != nullptr);

  // Use the restriction solver and options if they are provided.
  const solvers::SolverInterface* solver;
  std::unique_ptr<solvers::SolverInterface> default_solver;
  if (options.restriction_solver) {
    solver = options.restriction_solver;
  } else {
    default_solver = solvers::MakeSolver(solvers::ChooseBestSolver(*prog));
    solver = default_solver.get();
  }
  solvers::SolverOptions solver_options =
      options.restriction_solver_options.value_or(options.solver_options);

  MathematicalProgramResult result;
  solver->Solve(*prog, {}, solver_options, &result);
  MakeRestrictionResultLookLikeMixedInteger(*prog, &result, active_edges);

  return result;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
