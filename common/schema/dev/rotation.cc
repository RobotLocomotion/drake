#include "common/schema/rotation.h"

#include "drake/common/drake_throw.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"

namespace anzu {
namespace common {
namespace schema {

using drake::symbolic::Expression;

namespace {
// Boilerplate for std::visit.
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}  // namespace

Rotation::Rotation(const drake::math::RotationMatrix<double>& arg)
    : Rotation(drake::math::RollPitchYaw<double>(arg)) {}

Rotation::Rotation(const drake::math::RollPitchYaw<double>& arg) {
  const Eigen::Vector3d rpy_rad = arg.vector();
  set_rpy_deg(rpy_rad * 180 / M_PI);
}

bool Rotation::IsDeterministic() const {
  using Result = bool;
  return std::visit(overloaded {
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

drake::math::RotationMatrixd Rotation::GetDeterministicValue() const {
  DRAKE_THROW_UNLESS(this->IsDeterministic());
  const drake::Matrix3<Expression> symbolic = this->ToSymbolic().matrix();
  const Eigen::Matrix3d result = symbolic.unaryExpr([](const auto& e) {
    return drake::ExtractDoubleOrThrow(e);
  });
  return drake::math::RotationMatrixd(result);
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
drake::Vector<Expression, Size> deg2rad(
    const DistributionVectorVariant<Size>& deg_var) {
  DRAKE_THROW_UNLESS(!std::holds_alternative<GaussianVector<Size>>(deg_var));
  const drake::Vector<Expression, Size> deg_sym =
      schema::ToDistributionVector(deg_var)->ToSymbolic();
  return deg_sym * (M_PI / 180.0);
}
}  // namespace

drake::math::RotationMatrix<Expression> Rotation::ToSymbolic() const {
  using Result = drake::math::RotationMatrix<Expression>;
  return std::visit(overloaded {
    [](const Identity&) -> Result {
      return Result{};
    },
    [](const Rpy& rpy) -> Result {
      const drake::Vector3<Expression> rpy_rad = deg2rad(rpy.deg);
      return Result{drake::math::RollPitchYaw<Expression>(rpy_rad)};
    },
    [](const AngleAxis& aa) -> Result {
      const Expression angle_rad = deg2rad(aa.angle_deg);
      const drake::Vector3<Expression> axis =
          schema::ToDistributionVector(aa.axis)->ToSymbolic().normalized();
      const Eigen::AngleAxis<Expression> theta_lambda(angle_rad, axis);
      return Result{theta_lambda};
    },
    [](const Uniform&) -> Result {
      drake::RandomGenerator generator;
      return drake::math::UniformlyRandomRotationMatrix<Expression>(&generator);
    },
  }, value);
}

}  // namespace schema
}  // namespace common
}  // namespace anzu
