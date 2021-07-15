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

typedef GraphOfConvexSets::Vertex Vertex;
typedef GraphOfConvexSets::Edge Edge;

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

Vertex::Vertex(const ConvexSet& set, std::string name)
    : set_(set),
      name_(name),
      placeholder_x_(symbolic::MakeVectorContinuousVariable(
          set_.ambient_dimension(), name)) {}

Vertex::~Vertex() = default;

VectorXd Vertex::GetSolution(const MathematicalProgramResult& result) const {
  return result.GetSolution(placeholder_x_);
}

Edge::Edge(const Vertex& u, const Vertex& v, std::string name)
    : u_{u},
      v_{v},
      allowed_vars_{u_.x()},
      phi_{"phi", symbolic::Variable::Type::BINARY},
      name_{name},
      y_{symbolic::MakeVectorContinuousVariable(u_.ambient_dimension(), "y")},
      z_{symbolic::MakeVectorContinuousVariable(v_.ambient_dimension(), "z")},
      x_to_yz_(u_.ambient_dimension() + v_.ambient_dimension()) {
  allowed_vars_.insert(Variables(v_.x()));
  for (int i = 0; i < u_.ambient_dimension(); ++i) {
    x_to_yz_.emplace(u_.x()[i], y_[i]);
  }
  for (int i = 0; i < v_.ambient_dimension(); ++i) {
    x_to_yz_.emplace(v_.x()[i], z_[i]);
  }
}

Edge::~Edge() = default;

std::pair<Variable, Binding<Cost>> Edge::AddCost(
    const symbolic::Expression& e) {
  return AddCost(solvers::internal::ParseCost(e));
}

std::pair<Variable, Binding<Cost>> Edge::AddCost(const Binding<Cost>& binding) {
  DRAKE_DEMAND(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
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
  DRAKE_DEMAND(Variables(binding.variables()).IsSubsetOf(allowed_vars_));
  auto [iter, inserted] = constraints_.emplace(binding);
  return *iter;
}

double Edge::GetSolutionCost(const MathematicalProgramResult& result) const {
  return result.GetSolution(ell_).sum();
}

Vertex* GraphOfConvexSets::AddVertex(std::unique_ptr<ConvexSet> set,
                                       std::string name) {
  if (name.empty()) {
    name = fmt::format("v{}", vertices_.size());
  }
  sets_.emplace_back(std::move(set));
  auto [iter, inserted] =
      vertices_.emplace(std::make_unique<Vertex>(*sets_.back(), name));
  return iter->get();
}

Vertex* GraphOfConvexSets::AddVertex(const ConvexSet& set, std::string name) {
  return AddVertex(set.Clone(), name);
}

Edge* GraphOfConvexSets::AddEdge(const Vertex& u, const Vertex& v,
                                   std::string name) {
  if (name.empty()) {
    name = fmt::format("e{}", edges_.size());
  }
  auto [iter, inserted] = edges_.emplace(std::make_unique<Edge>(u, v, name));
  return iter->get();
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
