#include "drake/systems/primitives/transfer_function.h"

#include <drake/common/never_destroyed.h>

namespace drake {
namespace systems {

using symbolic::RationalFunction;
using symbolic::Variable;
using symbolic::Variables;

TransferFunction::TransferFunction()
    : TransferFunction(MatrixX<RationalFunction>(0, 0), 0.0) {}

TransferFunction::TransferFunction(MatrixX<RationalFunction> H,
                                   double time_step)
    : H_{H}, time_step_{time_step} {
  DRAKE_THROW_UNLESS(time_step >= 0.0);
  Variables s_or_z{time_step > 0 ? z_var() : s_var()};
  for (int i = 0; i < H_.rows(); ++i) {
    for (int j = 0; j < H_.cols(); ++j) {
      if (!H(i, j).numerator().indeterminates().IsSubsetOf(s_or_z) ||
          !H(i, j).denominator().indeterminates().IsSubsetOf(s_or_z)) {
        throw std::runtime_error(fmt::format(
            "H must only be a function of {}, because time-step = {}. H({},{}) "
            "= {}). Note that you must use the static methods of this class to "
            "obtain {}; variables with  the same name will not be recognized.",
            time_step > 0.0 ? "z_var()" : "s_var()", time_step, i, j,
            H(i, j).ToExpression(), time_step > 0.0 ? "z_var()" : "s_var()"));
      }
    }
  }
}

TransferFunction::TransferFunction(RationalFunction H, double time_step)
    : TransferFunction(Vector1<RationalFunction>(H), time_step) {}

Variable TransferFunction::s_var() {
  static const never_destroyed<Variable> s_var{"s"};
  return s_var.access();
}

Variable TransferFunction::z_var() {
  static const never_destroyed<Variable> z_var{"z"};
  return z_var.access();
}

}  // namespace systems
}  // namespace drake
