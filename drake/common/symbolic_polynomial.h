#pragma once

#include <Eigen/Core>

#include "drake/common/monomial.h"

namespace drake {
namespace symbolic {
/** Represents a polynomial expressed with symbolic variables.*/
class Polynomial {
 private:
  std::map<internal::Monomial, double, internal::GradedReverseLexOrder<std::less<Variable::Id>>> monomial_to_coeff_map_;
  Variables vars_;
};
}  // namespace symbolic
}  // namespace drake