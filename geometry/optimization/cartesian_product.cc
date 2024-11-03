#include "drake/geometry/optimization/cartesian_product.h"

#include <memory>

#include <fmt/format.h>

#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
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

int SumAmbientDimensions(const ConvexSets& sets) {
  int dim = 0;
  for (const copyable_unique_ptr<ConvexSet>& set : sets) {
    DRAKE_THROW_UNLESS(set != nullptr);
    dim += set->ambient_dimension();
  }
  return dim;
}

bool HasExactVolume(const std::vector<const ConvexSet*>& sets) {
  for (const auto* set : sets) {
    if (!set->has_exact_volume()) {
      return false;
    }
  }
  return true;
}

bool HasExactVolume(const ConvexSets& sets) {
  for (const auto& set : sets) {
    if (!set->has_exact_volume()) {
      return false;
    }
  }
  return true;
}

}  // namespace

CartesianProduct::CartesianProduct() : CartesianProduct(ConvexSets{}) {}

CartesianProduct::CartesianProduct(const ConvexSets& sets)
    : ConvexSet(SumAmbientDimensions(sets), HasExactVolume(sets)),
      sets_(sets) {}

CartesianProduct::CartesianProduct(const ConvexSet& setA, const ConvexSet& setB)
    : ConvexSet(setA.ambient_dimension() + setB.ambient_dimension(),
                HasExactVolume(std::vector{&setA, &setB})) {
  sets_.emplace_back(setA.Clone());
  sets_.emplace_back(setB.Clone());
}

CartesianProduct::CartesianProduct(const ConvexSets& sets,
                                   const Eigen::Ref<const MatrixXd>& A,
                                   const Eigen::Ref<const VectorXd>& b)
    : ConvexSet(A.cols(), HasExactVolume(sets)),
      sets_(sets),
      A_(A),
      b_(b),
      A_decomp_(Eigen::ColPivHouseholderQR<Eigen::MatrixXd>(*A_)) {
  const int y_ambient_dimension = SumAmbientDimensions(sets);
  const int x_ambient_dimension = ambient_dimension();
  DRAKE_THROW_UNLESS(A_->rows() == y_ambient_dimension);
  DRAKE_THROW_UNLESS(b_->rows() == y_ambient_dimension);
  DRAKE_THROW_UNLESS(A_->cols() == x_ambient_dimension);
  // Ensure that A is injective.
  DRAKE_THROW_UNLESS(A_decomp_->rank() == A_->cols());
}

CartesianProduct::CartesianProduct(const QueryObject<double>& query_object,
                                   GeometryId geometry_id,
                                   std::optional<FrameId> reference_frame)
    : ConvexSet(3, false) {
  const Shape& shape = query_object.inspector().GetShape(geometry_id);
  if (shape.type_name() != "Cylinder") {
    throw std::logic_error(fmt::format(
        "CartesianProduct(geometry_id={}, ...) cannot convert a {}, only a "
        "Cylinder",
        geometry_id, shape));
  }
  const Cylinder& cylinder = dynamic_cast<const Cylinder&>(shape);

  // Make the cylinder out of a circle (2D sphere) and a line segment (1D box).
  sets_.emplace_back(
      Hyperellipsoid::MakeHypersphere(cylinder.radius(), Vector2d::Zero())
          .Clone());
  // ToDo(@sadraddini) use MakeBox out of a future zonotope class instead so
  // that we can compute the volume exactly.
  sets_.emplace_back(HPolyhedron::MakeBox(Vector1d{-cylinder.length() / 2.0},
                                          Vector1d{cylinder.length() / 2.0})
                         .Clone());

  const RigidTransformd X_WF =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GF = X_WG.InvertAndCompose(X_WF);

  A_ = X_GF.rotation().matrix();
  b_ = X_GF.translation();
  A_decomp_ = Eigen::ColPivHouseholderQR<Eigen::MatrixXd>(*A_);
}

CartesianProduct::~CartesianProduct() = default;

const ConvexSet& CartesianProduct::factor(int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < ssize(sets_));
  return *sets_[index];
}

std::unique_ptr<ConvexSet> CartesianProduct::DoClone() const {
  return std::make_unique<CartesianProduct>(*this);
}

std::optional<bool> CartesianProduct::DoIsBoundedShortcutParallel(
    Parallelism parallelism) const {
  // Note: The constructor enforces that A_ is full column rank.
  for (const auto& s : sets_) {
    if (!s->IsBounded(parallelism)) {
      return false;
    }
  }
  return true;
}

bool CartesianProduct::DoIsEmpty() const {
  if (sets_.size() == 0) {
    return false;
  }
  for (const auto& s : sets_) {
    if (s->IsEmpty()) {
      return true;
    }
  }
  return false;
}

std::optional<VectorXd> CartesianProduct::DoMaybeGetPoint() const {
  // Check if all sets are points.
  std::vector<VectorXd> points;
  for (const auto& s : sets_) {
    if (std::optional<VectorXd> point = s->MaybeGetPoint()) {
      points.push_back(std::move(*point));
    } else {
      return std::nullopt;
    }
  }
  return CartesianProduct::StackAndMaybeTransform(points);
}

std::optional<Eigen::VectorXd> CartesianProduct::DoMaybeGetFeasiblePoint()
    const {
  std::vector<VectorXd> points;
  for (const auto& s : sets_) {
    if (std::optional<VectorXd> point = s->MaybeGetFeasiblePoint()) {
      points.push_back(std::move(*point));
    } else {
      return std::nullopt;
    }
  }
  return CartesianProduct::StackAndMaybeTransform(points);
}

VectorXd CartesianProduct::StackAndMaybeTransform(
    const std::vector<Eigen::VectorXd>& points) const {
  // Stack the points.
  const int y_ambient_dimension = A_ ? A_->rows() : ambient_dimension();
  int start = 0;
  VectorXd y(y_ambient_dimension);
  for (const VectorXd& point : points) {
    const int point_size = point.size();
    y.segment(start, point_size) = point;
    start += point_size;
  }
  DRAKE_DEMAND(start == y.size());

  // When A and b are NOT in use, x is just y.
  if (!A_.has_value()) {
    return y;
  }

  // When A_ is in use, either A was passed to the constructor or we denote a
  // cylinder. In the former case, we have the QR decomp already (set in the
  // constructor). In the latter case, we can't possibly have cleared the "all
  // sets are points" checks earlier in this function.
  DRAKE_DEMAND(A_decomp_.has_value());

  // Solve for x.
  return A_decomp_->solve(y - *b_);
}

bool CartesianProduct::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                                    double tol) const {
  int index = 0;
  VectorXd y = A_ ? (*A_) * x + (*b_) : x;
  for (const auto& s : sets_) {
    if (!s->PointInSet(y.segment(index, s->ambient_dimension()), tol)) {
      return false;
    }
    index += s->ambient_dimension();
  }
  return true;
}

std::pair<VectorX<symbolic::Variable>, std::vector<Binding<Constraint>>>
CartesianProduct::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  // Use std::vector which allocates heap memory logarithmically instead of
  // linearly.
  std::vector<Variable> new_vars;
  VectorXDecisionVariable y;
  std::vector<Binding<Constraint>> new_constraints;
  if (A_) {
    // Note: The constructor enforces that A_ is full column rank.
    y = prog->NewContinuousVariables(A_->rows(), "y");
    new_vars = std::vector<Variable>(y.data(), y.data() + y.rows() * y.cols());
    // y = Ax + b, or [I,-A]*[y;x] = b.
    MatrixXd Aeq = MatrixXd::Identity(A_->rows(), A_->rows() + A_->cols());
    Aeq.rightCols(A_->cols()) = -(*A_);
    new_constraints.push_back(
        prog->AddLinearEqualityConstraint(Aeq, (*b_), {y, x}));
  } else {
    y = x;
  }
  int index = 0;
  for (const auto& s : sets_) {
    if (s->ambient_dimension() == 0) {
      std::optional<Variable> new_var =
          ConvexSet::HandleZeroAmbientDimensionConstraints(prog, *s,
                                                           &new_constraints);
      if (new_var.has_value()) {
        new_vars.push_back(new_var.value());
      }
      continue;
    }
    const auto [new_var_in_s, new_constraints_in_s] =
        s->AddPointInSetConstraints(prog,
                                    y.segment(index, s->ambient_dimension()));
    for (int i = 0; i < new_var_in_s.rows(); ++i) {
      new_vars.push_back(new_var_in_s(i));
    }
    new_constraints.insert(new_constraints.end(), new_constraints_in_s.begin(),
                           new_constraints_in_s.end());
    index += s->ambient_dimension();
  }
  VectorX<Variable> new_vars_vec =
      Eigen::Map<VectorX<Variable>>(new_vars.data(), new_vars.size());
  return {std::move(new_vars_vec), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
CartesianProduct::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  VectorXDecisionVariable y;
  if (A_) {
    y = prog->NewContinuousVariables(A_->rows(), "y");
    // y = Ax + t*b, or [I,-b,-A]*[y;t;x] = 0.
    MatrixXd Aeq = MatrixXd::Identity(A_->rows(), A_->rows() + A_->cols() + 1);
    Aeq.col(A_->rows()) = -(*b_);
    Aeq.rightCols(A_->cols()) = -(*A_);
    constraints.emplace_back(prog->AddLinearEqualityConstraint(
        Aeq, VectorXd::Zero(A_->rows()), {y, Vector1<Variable>{t}, x}));
  } else {
    y = x;
  }
  int index = 0;
  for (const auto& s : sets_) {
    if (s->ambient_dimension() == 0) {
      ConvexSet::HandleZeroAmbientDimensionConstraints(prog, *s, &constraints);
      continue;
    }
    auto new_constraints = s->AddPointInNonnegativeScalingConstraints(
        prog, y.segment(index, s->ambient_dimension()), t);
    index += s->ambient_dimension();
    constraints.insert(constraints.end(),
                       std::make_move_iterator(new_constraints.begin()),
                       std::make_move_iterator(new_constraints.end()));
  }
  return constraints;
}

std::vector<Binding<Constraint>>
CartesianProduct::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A_x,
    const Eigen::Ref<const VectorXd>& b_x, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  if (A_) {
    VectorXDecisionVariable y = prog->NewContinuousVariables(A_->rows(), "y");
    // y = A (A_x * x + b_x) + (c' * t + d) b,
    // or [I, -A*A_x, - b*c']*[y; x; t] = A * b_x + d * b.
    MatrixXd Aeq =
        MatrixXd::Identity(A_->rows(), A_->rows() + x.size() + t.size());
    Aeq.middleCols(A_->rows(), x.size()) = -(*A_) * A_x;
    Aeq.rightCols(t.size()) = -(*b_) * c.transpose();
    VectorXd beq = (*A_) * b_x + (*b_) * d;
    constraints.emplace_back(
        prog->AddLinearEqualityConstraint(Aeq, beq, {y, x, t}));
    int index = 0;
    for (const auto& s : sets_) {
      if (s->ambient_dimension() == 0) {
        ConvexSet::HandleZeroAmbientDimensionConstraints(prog, *s,
                                                         &constraints);
        continue;
      }
      int set_dim = s->ambient_dimension();
      auto new_constraints = s->AddPointInNonnegativeScalingConstraints(
          prog, MatrixXd::Identity(set_dim, set_dim), VectorXd::Zero(set_dim),
          c, d, y.segment(index, set_dim), t);
      index += set_dim;
      constraints.insert(constraints.end(),
                         std::make_move_iterator(new_constraints.begin()),
                         std::make_move_iterator(new_constraints.end()));
    }
  } else {
    int index = 0;
    for (const auto& s : sets_) {
      if (s->ambient_dimension() == 0) {
        ConvexSet::HandleZeroAmbientDimensionConstraints(prog, *s,
                                                         &constraints);
        continue;
      }
      auto new_constraints = s->AddPointInNonnegativeScalingConstraints(
          prog, A_x.middleRows(index, s->ambient_dimension()),
          b_x.segment(index, s->ambient_dimension()), c, d, x, t);
      index += s->ambient_dimension();
      constraints.insert(constraints.end(),
                         std::make_move_iterator(new_constraints.begin()),
                         std::make_move_iterator(new_constraints.end()));
    }
  }
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
CartesianProduct::DoToShapeWithPose() const {
  // TODO(russt): Consider handling Cylinder as a (very) special case.
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for CartesianProduct.");
}

std::unique_ptr<ConvexSet> CartesianProduct::DoAffineHullShortcut(
    std::optional<double> tol) const {
  // TODO(cohnt): Support affine transformations of Cartesian products. For now,
  // we just return std::nullopt and use the generic affine hull computation.
  if (A_ != std::nullopt || b_ != std::nullopt) {
    return nullptr;
  }

  // The basis will be a block diagonal matrix, whose blocks correspond to the
  // bases of the affine subspace of each factor. Not all blocks will be square,
  // and some of the columns on the right will be skipped, since the affine hull
  // may be a proper subspace.
  MatrixXd basis = MatrixXd::Zero(ambient_dimension(), ambient_dimension());
  // The translation will be a vector, concatenating all of the translations of
  // each factor. Zero-initialization is not needed, since all entries will be
  // overwritten in the following loop.
  VectorXd translation(ambient_dimension());
  int current_dimension = 0;
  int num_basis_vectors = 0;
  for (int i = 0; i < num_factors(); ++i) {
    AffineSubspace a(factor(i), tol);
    basis.block(current_dimension, num_basis_vectors, a.ambient_dimension(),
                a.AffineDimension()) = a.basis();
    translation.segment(current_dimension, a.ambient_dimension()) =
        a.translation();
    current_dimension += a.ambient_dimension();
    num_basis_vectors += a.AffineDimension();
  }
  return std::make_unique<AffineSubspace>(basis.leftCols(num_basis_vectors),
                                          std::move(translation));
}

double CartesianProduct::DoCalcVolume() const {
  DRAKE_DEMAND(sets_.size() > 0);
  double volume = 1.0;
  for (const auto& set : sets_) {
    volume *= set->CalcVolume();
  }
  if (A_.has_value()) {
    // Note: The constructor enforces that A_ is full column rank.
    if (A_decomp_->rank() < A_->rows()) {
      volume *= 0.0;
    } else {
      // the determinant of a triangular matrix is the product of the diagonal
      volume *= std::abs(A_decomp_->matrixQR().diagonal().prod());
    }
  }
  return volume;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
