#include "drake/geometry/optimization/minkowski_sum.h"

#include <memory>

#include <fmt/format.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::Solve;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

namespace {

int GetAmbientDimension(const ConvexSets& sets) {
  if (sets.empty()) {
    return 0;
  }
  const int ambient_dimension = sets[0]->ambient_dimension();
  for (const copyable_unique_ptr<ConvexSet>& set : sets) {
    DRAKE_THROW_UNLESS(set != nullptr);
    DRAKE_THROW_UNLESS(set->ambient_dimension() == ambient_dimension);
  }
  return ambient_dimension;
}

}  // namespace

MinkowskiSum::MinkowskiSum() : MinkowskiSum(ConvexSets{}) {}

MinkowskiSum::MinkowskiSum(const ConvexSets& sets)
    : ConvexSet(GetAmbientDimension(sets), false), sets_(sets) {}

MinkowskiSum::MinkowskiSum(const ConvexSet& setA, const ConvexSet& setB)
    : ConvexSet(setA.ambient_dimension(), false) {
  DRAKE_THROW_UNLESS(setB.ambient_dimension() == setA.ambient_dimension());
  sets_.emplace_back(setA.Clone());
  sets_.emplace_back(setB.Clone());
}

MinkowskiSum::MinkowskiSum(const QueryObject<double>& query_object,
                           GeometryId geometry_id,
                           std::optional<FrameId> reference_frame)
    : ConvexSet(3, false) {
  const Shape& shape = query_object.inspector().GetShape(geometry_id);
  if (shape.type_name() != "Capsule") {
    throw std::logic_error(fmt::format(
        "MinkowskiSum(geometry_id={}, ...) cannot convert a {}, only a Capsule",
        geometry_id, shape));
  }
  const Capsule& capsule = dynamic_cast<const Capsule&>(shape);

  // Sphere at zero.
  sets_.emplace_back(
      Hyperellipsoid::MakeHypersphere(capsule.radius(), Vector3d::Zero())
          .Clone());

  const RigidTransformd X_WF =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GF = X_WG.InvertAndCompose(X_WF);

  // Line segment as a HPolyhedron (the VPolytope would be easier here, but
  // HPolyhedron is nicer for most of the computations).
  HPolyhedron H_G =
      HPolyhedron::MakeBox(Vector3d{0, 0, -capsule.length() / 2.0},
                           Vector3d{0, 0, capsule.length() / 2.0});
  // A_G*(p_GF + R_GF*p_FF_var) ≤ b_G
  sets_.emplace_back(
      std::make_unique<HPolyhedron>(H_G.A() * X_GF.rotation().matrix(),
                                    H_G.b() - H_G.A() * X_GF.translation()));
}

MinkowskiSum::~MinkowskiSum() = default;

const ConvexSet& MinkowskiSum::term(int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < ssize(sets_));
  return *sets_[index];
}

std::unique_ptr<ConvexSet> MinkowskiSum::DoClone() const {
  return std::make_unique<MinkowskiSum>(*this);
}

std::optional<bool> MinkowskiSum::DoIsBoundedShortcutParallel(
    Parallelism parallelism) const {
  if (IsEmpty()) {
    return true;
  }
  for (const auto& s : sets_) {
    if (!s->IsBounded(parallelism)) {
      return false;
    }
  }
  return true;
}

bool MinkowskiSum::DoIsEmpty() const {
  if (sets_.size() == 0) {
    return false;
  }
  // The empty set is annihilatory in Minkowski addition.
  for (const auto& s : sets_) {
    if (s->IsEmpty()) {
      return true;
    }
  }
  return false;
}

std::optional<VectorXd> MinkowskiSum::DoMaybeGetPoint() const {
  std::optional<VectorXd> result;
  for (const auto& s : sets_) {
    if (std::optional<VectorXd> point = s->MaybeGetPoint()) {
      if (result.has_value()) {
        *result += *point;
      } else {
        result = std::move(point);
      }
    } else {
      return std::nullopt;
    }
  }
  return result;
}

std::optional<VectorXd> MinkowskiSum::DoMaybeGetFeasiblePoint() const {
  std::optional<VectorXd> result;
  for (const auto& s : sets_) {
    if (std::optional<VectorXd> point = s->MaybeGetFeasiblePoint()) {
      if (result.has_value()) {
        *result += *point;
      } else {
        result = std::move(point);
      }
    } else {
      return std::nullopt;
    }
  }
  return result;
}

bool MinkowskiSum::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                                double) const {
  // TODO(russt): Figure out if there is a general way to communicate tol
  // to/through the solver.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables(ambient_dimension(), num_terms(), "x");
  const VectorXd ones = VectorXd::Ones(num_terms());
  for (int i = 0; i < ambient_dimension(); ++i) {
    // ∑ⱼ xⱼ[i] = x[i]
    prog.AddLinearEqualityConstraint(ones, x[i], X.row(i).transpose());
  }
  for (int j = 0; j < num_terms(); ++j) {
    sets_[j]->AddPointInSetConstraints(&prog, X.col(j));
  }
  auto result = Solve(prog);
  return result.is_success();
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
MinkowskiSum::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  std::vector<Variable> new_vars;
  std::vector<Binding<Constraint>> new_constraints;
  auto X = prog->NewContinuousVariables(ambient_dimension(), num_terms(), "x");
  new_vars.reserve(X.size());
  for (int j = 0; j < X.cols(); ++j) {
    for (int i = 0; i < X.rows(); ++i) {
      new_vars.push_back(X(i, j));
    }
  }
  RowVectorXd a = RowVectorXd::Ones(num_terms() + 1);
  a[0] = -1;
  for (int i = 0; i < ambient_dimension(); ++i) {
    // ∑ⱼ xⱼ[i] = x[i]
    new_constraints.push_back(prog->AddLinearEqualityConstraint(
        a, 0.0, {Vector1<Variable>(x[i]), X.row(i).transpose()}));
  }
  for (int j = 0; j < num_terms(); ++j) {
    if (sets_[j]->ambient_dimension() == 0) {
      std::optional<Variable> new_var =
          ConvexSet::HandleZeroAmbientDimensionConstraints(prog, *sets_[j],
                                                           &new_constraints);
      if (new_var.has_value()) {
        new_vars.push_back(new_var.value());
      }
      continue;
    }
    const auto [new_vars_in_sets_j, new_constraints_in_sets_j] =
        sets_[j]->AddPointInSetConstraints(prog, X.col(j));
    for (int k = 0; k < new_vars_in_sets_j.rows(); ++k) {
      new_vars.push_back(new_vars_in_sets_j(k));
    }
    new_constraints.insert(new_constraints.end(),
                           new_constraints_in_sets_j.begin(),
                           new_constraints_in_sets_j.end());
  }
  VectorX<Variable> new_vars_vec =
      Eigen::Map<VectorX<Variable>>(new_vars.data(), new_vars.size());
  return {std::move(new_vars_vec), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
MinkowskiSum::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  // We add the constraint
  //   x in t (S1 ⨁ ... ⨁ Sn)
  // by enforcing
  //   x in t S1 ⨁ ... ⨁ t Sn.
  // This can be done because t is nonnegative and S1,..., Sn are convex.
  std::vector<Binding<Constraint>> constraints;
  auto X = prog->NewContinuousVariables(ambient_dimension(), num_terms(), "x");
  RowVectorXd a = RowVectorXd::Ones(num_terms() + 1);
  a[0] = -1;
  for (int i = 0; i < ambient_dimension(); ++i) {
    // ∑ⱼ xⱼ[i] = x[i]
    constraints.emplace_back(prog->AddLinearEqualityConstraint(
        a, 0.0, {Vector1<Variable>(x[i]), X.row(i).transpose()}));
  }
  for (int j = 0; j < num_terms(); ++j) {
    if (sets_[j]->ambient_dimension() == 0) {
      ConvexSet::HandleZeroAmbientDimensionConstraints(prog, *sets_[j],
                                                       &constraints);
      continue;
    }
    auto new_constraints =
        sets_[j]->AddPointInNonnegativeScalingConstraints(prog, X.col(j), t);
    constraints.insert(constraints.end(),
                       std::make_move_iterator(new_constraints.begin()),
                       std::make_move_iterator(new_constraints.end()));
  }
  return constraints;
}

std::vector<Binding<Constraint>>
MinkowskiSum::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  // We add the constraint
  //   A*x+b in (c't+d) (S1 ⨁ ... ⨁ Sn)
  // by enforcing
  //   A*x+b in (c't+d) S1 ⨁ ... ⨁ (c't+d) Sn.
  // This can be done because c't+d is nonnegative and S1,..., Sn are convex.
  std::vector<Binding<Constraint>> constraints;
  auto X = prog->NewContinuousVariables(x.size(), num_terms(), "x");
  RowVectorXd a = RowVectorXd::Ones(num_terms() + 1);
  a[0] = -1;
  for (int i = 0; i < x.size(); ++i) {
    // ∑ⱼ xⱼ[i] = x[i]
    constraints.emplace_back(prog->AddLinearEqualityConstraint(
        a, 0.0, {Vector1<Variable>(x[i]), X.row(i).transpose()}));
  }
  for (int j = 0; j < num_terms(); ++j) {
    if (sets_[j]->ambient_dimension() == 0) {
      ConvexSet::HandleZeroAmbientDimensionConstraints(prog, *sets_[j],
                                                       &constraints);
      continue;
    }
    auto new_constraints = sets_[j]->AddPointInNonnegativeScalingConstraints(
        prog, A, b, c, d, X.col(j), t);
    constraints.insert(constraints.end(),
                       std::make_move_iterator(new_constraints.begin()),
                       std::make_move_iterator(new_constraints.end()));
  }
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
MinkowskiSum::DoToShapeWithPose() const {
  // TODO(russt): Consider handling Capsule as a (very) special case.
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for MinkowskiSum.");
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
