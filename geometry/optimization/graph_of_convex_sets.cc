#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/solvers/create_cost.h"

namespace drake {
namespace geometry {
namespace optimization {

using Edge = GraphOfConvexSets::Edge;
using Vertex = GraphOfConvexSets::Vertex;
using VertexId = GraphOfConvexSets::VertexId;

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::Cost;
using solvers::L2NormCost;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::QuadraticCost;
using solvers::VariableRefList;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

GraphOfConvexSets::~GraphOfConvexSets() = default;

Vertex::Vertex(const VertexId& id, const ConvexSet& set, std::string name)
    : id_(id),
      set_(set.Clone()),
      name_(std::move(name)),
      placeholder_x_(symbolic::MakeVectorContinuousVariable(
          set_->ambient_dimension(), name_)) {}

Vertex::~Vertex() = default;

VectorXd Vertex::GetSolution(const MathematicalProgramResult& result) const {
  return result.GetSolution(placeholder_x_);
}

Edge::Edge(const Vertex* u, const Vertex* v, std::string name)
    : u_{u},
      v_{v},
      allowed_vars_{u_->x()},
      phi_{"phi", symbolic::Variable::Type::BINARY},
      name_(std::move(name)),
      y_{symbolic::MakeVectorContinuousVariable(u_->ambient_dimension(), "y")},
      z_{symbolic::MakeVectorContinuousVariable(v_->ambient_dimension(), "z")},
      x_to_yz_(u_->ambient_dimension() + v_->ambient_dimension()) {
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
  auto [iter, inserted] = constraints_.emplace(binding);
  unused(inserted);
  return *iter;
}

double Edge::GetSolutionCost(const MathematicalProgramResult& result) const {
  return result.GetSolution(ell_).sum();
}

Vertex* GraphOfConvexSets::AddVertex(const ConvexSet& set, std::string name) {
  if (name.empty()) {
    name = fmt::format("v{}", vertices_.size());
  }
  VertexId id = VertexId::get_new_id();
  auto [iter, success] =
      vertices_.emplace(id, std::unique_ptr<Vertex>(new Vertex(id, set, name)));
  DRAKE_DEMAND(success);
  return iter->second.get();
}

Edge* GraphOfConvexSets::AddEdge(const VertexId& u_id, const VertexId& v_id,
                                 std::string name) {
  auto u_iter = vertices_.find(u_id);
  DRAKE_DEMAND(u_iter != vertices_.end());
  auto v_iter = vertices_.find(v_id);
  DRAKE_DEMAND(v_iter != vertices_.end());

  if (name.empty()) {
    name = fmt::format("e{}", edges_.size());
  }
  auto [iter, success] = edges_.emplace(std::unique_ptr<Edge>(
      new Edge(u_iter->second.get(), v_iter->second.get(), name)));
  DRAKE_DEMAND(success);
  return iter->get();
}

Edge* GraphOfConvexSets::AddEdge(const Vertex& u, const Vertex& v,
                                 std::string name) {
  return AddEdge(u.id(), v.id(), std::move(name));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
