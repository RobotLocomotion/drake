#include "drake/common/schema/rotation.h"

#include "drake/common/drake_throw.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace schema {

using symbolic::Expression;

namespace {
// Boilerplate for std::visit.
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}  // namespace

Rotation::Rotation(const math::RotationMatrix<double>& arg)
    : Rotation(math::RollPitchYaw<double>(arg)) {}

Rotation::Rotation(const math::RollPitchYaw<double>& arg) {
  const Eigen::Vector3d rpy_rad = arg.vector();
  set_rpy_deg(rpy_rad * 180 / M_PI);
}

bool Rotation::IsDeterministic() const {
  using Result = bool;
  return std::visit(overloaded{
    [](const Identity&) -> Result {
      return true;
    },
    [](const Rpy& rpy) -> Result {
      return schema::IsDeterministic(rpy.deg);
    },
    [](const AngleAxis& aa) -> Result {
      return schema::IsDeterministic(aa.angle_deg) &&
             schema::IsDeterministic(aa.axis);
    },
    [](const Uniform&) -> Result {
      return false;
    },
  }, value);
}

math::RotationMatrixd Rotation::GetDeterministicValue() const {
  DRAKE_THROW_UNLESS(this->IsDeterministic());
  const Matrix3<Expression> symbolic = this->ToSymbolic().matrix();
  const Eigen::Matrix3d result = symbolic.unaryExpr([](const auto& e) {
    return ExtractDoubleOrThrow(e);
  });
  return math::RotationMatrixd(result);
}

namespace {
// Converts a degree distribution to radians.  Our symbolic representations do
// not yet handle gaussian angles correctly, so we forbid them for now.
Expression deg2rad(const DistributionVariant& deg_var) {
  DRAKE_THROW_UNLESS(!std::holds_alternative<Gaussian>(deg_var));
  const Expression deg_sym = schema::ToSymbolic(deg_var);
  return deg_sym * (M_PI / 180.0);
}
template <int Size>
Vector<Expression, Size> deg2rad(
    const DistributionVectorVariant<Size>& deg_var) {
  DRAKE_THROW_UNLESS(!std::holds_alternative<GaussianVector<Size>>(deg_var));
  const Vector<Expression, Size> deg_sym =
      schema::ToDistributionVector(deg_var)->ToSymbolic();
  return deg_sym * (M_PI / 180.0);
}
}  // namespace

math::RotationMatrix<Expression> Rotation::ToSymbolic() const {
  using Result = math::RotationMatrix<Expression>;
  return std::visit(overloaded{
    [](const Identity&) -> Result {
      return Result{};
    },
    [](const Rpy& rpy) -> Result {
      const Vector3<Expression> rpy_rad = deg2rad(rpy.deg);
      return Result{math::RollPitchYaw<Expression>(rpy_rad)};
    },
    [](const AngleAxis& aa) -> Result {
      const Expression angle_rad = deg2rad(aa.angle_deg);
      const Vector3<Expression> axis =
          schema::ToDistributionVector(aa.axis)->ToSymbolic().normalized();
      const Eigen::AngleAxis<Expression> theta_lambda(angle_rad, axis);
      return Result{theta_lambda};
    },
    [](const Uniform&) -> Result {
      RandomGenerator generator;
      return math::UniformlyRandomRotationMatrix<Expression>(&generator);
    },
  }, value);
}

}  // namespace schema
}  // namespace drake
