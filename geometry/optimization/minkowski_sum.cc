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

MinkowskiSum::MinkowskiSum(const ConvexSets& sets)
    : ConvexSet(&ConvexSetCloner<MinkowskiSum>,
                sets.size() > 0 ? sets[0]->ambient_dimension() : 0),
      sets_(sets) {
  for (int i = 1; i < static_cast<int>(sets_.size()); ++i) {
    DRAKE_DEMAND(sets_[i]->ambient_dimension() ==
                 sets_[0]->ambient_dimension());
  }
}

MinkowskiSum::MinkowskiSum(const ConvexSet& setA, const ConvexSet& setB)
    : ConvexSet(&ConvexSetCloner<MinkowskiSum>, setA.ambient_dimension()) {
  DRAKE_DEMAND(setB.ambient_dimension() == setA.ambient_dimension());
  sets_.emplace_back(setA.Clone());
  sets_.emplace_back(setB.Clone());
}

MinkowskiSum::MinkowskiSum(const QueryObject<double>& query_object,
                           GeometryId geometry_id,
                           std::optional<FrameId> reference_frame)
    : ConvexSet(&ConvexSetCloner<MinkowskiSum>, 3) {
  Capsule capsule(1., 1.);
  query_object.inspector().GetShape(geometry_id).Reify(this, &capsule);

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
  DRAKE_DEMAND(0 <= index && index < static_cast<int>(sets_.size()));
  return *sets_[index];
}

bool MinkowskiSum::DoIsBounded() const {
  for (const auto& s : sets_) {
    if (!s->IsBounded()) {
      return false;
    }
  }
  return true;
}

bool MinkowskiSum::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
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

void MinkowskiSum::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  auto X = prog->NewContinuousVariables(ambient_dimension(), num_terms(), "x");
  RowVectorXd a = RowVectorXd::Ones(num_terms() + 1);
  a[0] = -1;
  for (int i = 0; i < ambient_dimension(); ++i) {
    // ∑ⱼ xⱼ[i] = x[i]
    prog->AddLinearEqualityConstraint(
        a, 0.0, {Vector1<Variable>(x[i]), X.row(i).transpose()});
  }
  for (int j = 0; j < num_terms(); ++j) {
    sets_[j]->AddPointInSetConstraints(prog, X.col(j));
  }
}

std::vector<Binding<Constraint>>
MinkowskiSum::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
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
    auto new_constraints =
        sets_[j]->AddPointInNonnegativeScalingConstraints(prog, X.col(j), t);
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

void MinkowskiSum::ImplementGeometry(const Capsule& capsule, void* data) {
  Capsule* c = static_cast<Capsule*>(data);
  *c = capsule;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
