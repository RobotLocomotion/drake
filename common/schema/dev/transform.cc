#include "common/schema/transform.h"

#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/math/rotation_matrix.h"

namespace anzu {
namespace common {
namespace schema {

using drake::symbolic::Expression;

Transform::Transform(const drake::math::RigidTransformd& x) {
  translation = x.translation();
  rotation = schema::Rotation{x.rotation()};
}

bool Transform::IsDeterministic() const {
  return schema::IsDeterministic(translation) && rotation.IsDeterministic();
}

drake::math::RigidTransformd Transform::GetDeterministicValue() const {
  DRAKE_THROW_UNLESS(this->IsDeterministic());
  return {
    rotation.GetDeterministicValue(),
    schema::GetDeterministicValue(translation),
  };
}

drake::math::RigidTransform<Expression> Transform::ToSymbolic() const {
  auto sym_translate = schema::ToDistributionVector(translation)->ToSymbolic();
  auto sym_rotate = rotation.ToSymbolic();
  return drake::math::RigidTransform<Expression>(sym_rotate, sym_translate);
}

drake::math::RigidTransformd Transform::Mean() const {
  using drake::symbolic::Environment;
  using drake::symbolic::Variables;
  using VariableType = drake::symbolic::Variable::Type;

  // Obtain the symbolic form of this Transform; this is an easy way to
  // collapse the variants into simple arithmetic expressions.
  const drake::math::RigidTransform<Expression> symbolic = ToSymbolic();

  // The symbolic representation of the transform uses uniform random
  // variables (or else has no variables).  Set them all to the mean.
  Environment env;
  for (const auto& var : GetDistinctVariables(symbolic.GetAsMatrix34())) {
    DRAKE_DEMAND(var.get_type() == VariableType::RANDOM_UNIFORM);
    env.insert(var, 0.5);
  }

  // Extract the underlying matrix of the transform, subsitute the env so
  // that the expressions are now all constants, and then re-create the
  // RigidTransform wrapper around the matrix.
  const auto to_double = [&env](const auto& x) { return x.Evaluate(env); };
  return drake::math::RigidTransformd(
      symbolic.GetAsMatrix34().unaryExpr(to_double));
}

drake::math::RigidTransformd Transform::Sample(
    drake::RandomGenerator* random) const {
  // It is somewhat yak-shavey to get an actual materialized transform here.
  // We convert to symbolic, convert the symbolic to a vector and matrix of
  // symbolic, `Evaluate` those, convert the result back to a
  // `RigidTransform<double>`, and build the resulting values into a new
  // fully determinstic `Transform`.
  //
  // This is *much* prettier written with `auto` but please do not be
  // tempted to use it here: I have left the long type names in because it
  // is impossible to debug Eigen `enable_if` error messages without them.
  const drake::math::RigidTransform<Expression> symbolic_transform =
      ToSymbolic();

  const drake::Vector3<Expression> symbolic_translation =
      symbolic_transform.translation();
  const Eigen::Vector3d concrete_translation =
      drake::symbolic::Evaluate(symbolic_translation, {}, random);

  const drake::math::RotationMatrix<Expression> symbolic_rotation =
      symbolic_transform.rotation();
  const drake::math::RotationMatrixd concrete_rotation(
      drake::symbolic::Evaluate(symbolic_rotation.matrix(), {}, random));

  return drake::math::RigidTransformd{concrete_rotation, concrete_translation};
}

}  // namespace schema
}  // namespace common
}  // namespace anzu
