#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
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

std::unordered_set<VertexId> GraphOfConvexSets::VertexIds() const {
  std::unordered_set<VertexId> ids(vertices_.size());
  for (const auto& v : vertices_) {
    ids.emplace(v.first);
  }
  return ids;
}

std::unordered_set<Edge*> GraphOfConvexSets::Edges() const {
  std::unordered_set<Edge*> edges(edges_.size());
  for (auto& e : edges_) {
    edges.emplace(e.get());
  }
  return edges;
}

MathematicalProgramResult GraphOfConvexSets::SolveShortestPath(
    const VertexId& source_id, const VertexId& target_id,
    bool convex_relaxation) const {
  DRAKE_DEMAND(vertices_.find(source_id) != vertices_.end());
  DRAKE_DEMAND(vertices_.find(target_id) != vertices_.end());

  MathematicalProgram prog;

  std::unordered_map<VertexId, std::vector<Edge*>> incoming_edges(
      vertices_.size());
  std::unordered_map<VertexId, std::vector<Edge*>> outgoing_edges(
      vertices_.size());
  const double inf = std::numeric_limits<double>::infinity();

  std::unordered_map<const Edge*, Variable> relaxed_phi(
      convex_relaxation ? edges_.size() : 0);

  for (const auto& e : edges_) {
    outgoing_edges[e->u().id()].emplace_back(e.get());
    incoming_edges[e->v().id()].emplace_back(e.get());

    Variable phi;
    if (convex_relaxation) {
      phi = prog.NewContinuousVariables<1>("phi")[0];
      prog.AddBoundingBoxConstraint(0, 1, phi);
      relaxed_phi.emplace(e.get(), phi);
    } else {
      phi = e->phi_;
      prog.AddDecisionVariables(Vector1<Variable>(phi));
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

      // TODO(russt): Avoid this use of RTTI, which mirrors the current pattern
      // in MathematicalProgram::AddCost.
      Cost* cost = b.evaluator().get();
      if (LinearCost* lc = dynamic_cast<LinearCost*>(cost)) {
        // Linear terms can lead to negative costs, which cannot be allowed.  We
        // only support constant terms here.
        // TODO(russt): Consider setting a precision here (and exposing it to
        // the user) instead of using Eigen's dummy_precision.
        if (!lc->a().isZero()) {
          throw std::runtime_error(
              fmt::format("In order to prevent negative edge lengths, linear "
                          "costs cannot have linear terms (only constant): {}",
                          b.to_string()));
        }
        if (lc->b() < 0.0) {
          throw std::runtime_error(
              fmt::format("Costs must be non-negative: {}", b.to_string()));
        }
        // phi*b <= ell or [b, -1.0][phi; ell] <= 0
        RowVector2d a{lc->b(), -1.0};
        prog.AddLinearConstraint(a, -inf, 0.0, vars.head<2>());
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
              b.to_string(), minimum[0]));
        }
        MatrixXd A_cone = MatrixXd::Zero(R.rows() + 2, vars.size());
        A_cone(0, 0) = 1.0;  // z₀ = ϕ.
        // z₁ = ℓ - b'x - ϕ c.
        A_cone(1, 1) = 1.0;
        A_cone.block(1, 2, 1, qc->b().rows()) = -qc->b().transpose();
        A_cone(1, 0) = -qc->c();
        // z₂ ... z_{n+1} = R x.
        A_cone.block(2, 2, R.rows(), R.cols()) = R;
        prog.AddRotatedLorentzConeConstraint(
            A_cone, VectorXd::Zero(A_cone.rows()), vars);
      } else if (L2NormCost* l2c = dynamic_cast<L2NormCost*>(cost)) {
        // |Ax + b|₂ becomes ℓ ≥ |Ax+bϕ|₂.
        MatrixXd A_cone = MatrixXd::Zero(l2c->A().rows() + 2, vars.size());
        A_cone(0, 1) = 1.0;                                 // z₀ = ℓ.
        A_cone.block(1, 0, l2c->A().rows(), 1) = l2c->b();  // bϕ.
        A_cone.block(1, 2, l2c->A().rows(), l2c->A().cols()) = l2c->A();  // Ax.
        prog.AddLorentzConeConstraint(A_cone, VectorXd::Zero(A_cone.rows()),
                                      vars);
      } else {
        throw std::runtime_error(fmt::format(
            "GraphOfConvexSets::Edge does not support this binding type: {}",
            b.to_string()));
      }
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
      Constraint* constraint = b.evaluator().get();
      if (LinearEqualityConstraint* lec =
              dynamic_cast<LinearEqualityConstraint*>(constraint)) {
        // A*x = b becomes A*x = phi*b.
        MatrixXd Aeq(lec->A().rows(), lec->A().cols() + 1);
        Aeq.col(0) = -lec->lower_bound();
        Aeq.rightCols(lec->A().cols()) = lec->A();
        prog.AddLinearEqualityConstraint(Aeq, VectorXd::Zero(lec->A().rows()),
                                         vars);
        // Note that LinearEqualityConstraint must come before LinearConstraint,
        // because LinearEqualityConstraint isa LinearConstraint.
      } else if (LinearConstraint* lc =
                     dynamic_cast<LinearConstraint*>(constraint)) {
        // lb <= A*x <= ub becomes
        // A*x <= phi*ub and phi*lb <= A*x, which can be spelled
        // [-ub, A][phi; x] <= 0, and 0 <= [-lb, A][phi; x].
        RowVectorXd a(vars.size());
        for (int i = 0; i < lc->A().rows(); ++i) {
          if (std::isfinite(lc->upper_bound()[i])) {
            a[0] = -lc->upper_bound()[i];
            a.tail(lc->A().cols()) = lc->A().row(i);
            prog.AddLinearConstraint(a, -inf, 0, vars);
          }
          if (std::isfinite(lc->lower_bound()[i])) {
            a[0] = -lc->lower_bound()[i];
            a.tail(lc->A().cols()) = lc->A().row(i);
            prog.AddLinearConstraint(a, 0, inf, vars);
          }
        }
      } else {
        throw std::runtime_error(
            fmt::format("ShortestPathProblem::Edge does not support this "
                        "binding type: {}",
                        b.to_string()));
      }
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
      for (const auto* e : incoming) {
        vars[count++] = convex_relaxation ? relaxed_phi.at(e) : e->phi_;
      }
      for (const auto* e : outgoing) {
        vars[count++] = convex_relaxation ? relaxed_phi.at(e) : e->phi_;
      }
      prog.AddLinearEqualityConstraint(
          a, (is_source ? 1.0 : 0.0) - (is_target ? 1.0 : 0.0), vars);

      // Spatial conservation of flow: ∑ z_in = ∑ y_out.
      if (!is_source && !is_target) {
        for (int i = 0; i < v->ambient_dimension(); ++i) {
          count = 0;
          for (const auto* e : incoming) {
            vars[count++] = e->z_[i];
          }
          for (const auto* e : outgoing) {
            vars[count++] = e->y_[i];
          }
          prog.AddLinearEqualityConstraint(a, 0, vars);
        }
      }
    }

    // Degree constraint: ∑ ϕ_out <= 1- δ(is_target).
    if (outgoing.size() > 0) {
      VectorXDecisionVariable phi_out(outgoing.size());
      for (int i = 0; i < static_cast<int>(outgoing.size()); ++i) {
        phi_out[i] =
            convex_relaxation ? relaxed_phi.at(outgoing[i]) : outgoing[i]->phi_;
      }
      prog.AddLinearConstraint(RowVectorXd::Ones(outgoing.size()), 0.0,
                               is_target ? 0.0 : 1.0, phi_out);
    }
  }

  MathematicalProgramResult result = solvers::Solve(prog);

  // Push the placeholder variables into the result, so that they can be
  // accessed as if they were real variables.
  int num_placeholder_vars = relaxed_phi.size();
  for (const std::pair<const VertexId, std::unique_ptr<Vertex>>& vpair :
       vertices_) {
    num_placeholder_vars += vpair.second->ambient_dimension();
  }
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index =
      prog.decision_variable_index();
  int count = result.get_x_val().size();
  Eigen::VectorXd x_val(count + num_placeholder_vars);
  x_val.head(count) = result.get_x_val();
  for (const std::pair<const VertexId, std::unique_ptr<Vertex>>& vpair :
       vertices_) {
    const Vertex* v = vpair.second.get();
    const bool is_target = (target_id == v->id());
    VectorXd x_v = VectorXd::Zero(v->ambient_dimension());
    if (is_target) {
      for (const auto& e : incoming_edges[v->id()]) {
        x_v += result.GetSolution(e->z_);
      }
    } else {
      for (const auto& e : outgoing_edges[v->id()]) {
        x_v += result.GetSolution(e->y_);
      }
    }
    for (int i = 0; i < v->ambient_dimension(); ++i) {
      decision_variable_index.emplace(v->x()[i].get_id(), count);
      x_val[count++] = x_v[i];
    }
  }
  if (convex_relaxation) {
    // Write the value of the relaxed phi into the phi placeholder.
    for (const std::pair<const Edge* const, Variable>& phipair : relaxed_phi) {
      decision_variable_index.emplace(phipair.first->phi_.get_id(), count);
      x_val[count++] = result.GetSolution(phipair.second);
    }
  }
  result.set_decision_variable_index(decision_variable_index);
  result.set_x_val(x_val);
  return result;
}

MathematicalProgramResult GraphOfConvexSets::SolveShortestPath(
    const Vertex& source, const Vertex& target,
    bool convex_relaxation) const {
  return SolveShortestPath(source.id(), target.id(), convex_relaxation);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
