#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace test {

// Helper to set the derivatives to the desired n-dimensional values, resizing
// its derivatives vector according to the size of `desired`.
inline static void SetDerivatives(AutoDiffXd* variable,
                                  const Eigen::VectorXd& desired) {
  variable->derivatives() = desired;
}

inline static void SetDerivatives(double*, const Eigen::VectorXd&) {}

// Helper to check that the derivatives match, to within the given tolerance,
// the provided expected values.
inline static void CheckDerivatives(const AutoDiffXd& variable,
                                    const Eigen::VectorXd& expected,
                                    const double tol = 1e-9) {
  EXPECT_TRUE(CompareMatrices(expected, variable.derivatives(), tol));
}

inline static void CheckDerivatives(const double&, const Eigen::VectorXd&) {}

// Helper to check positivity of the i-th derivative of the variable.
inline static void CheckDerivativePositivity(int i,
                                             const AutoDiffXd& variable) {
  DRAKE_DEMAND(i >= 0);
  DRAKE_DEMAND(i < variable.derivatives().size());
  EXPECT_LT(0., variable.derivatives()(i));
}

inline static void CheckDerivativePositivity(int, const double&) {}

// Helper to check negativity of the i-th derivative of the variable.
inline static void CheckDerivativeNegativity(int i,
                                             const AutoDiffXd& variable) {
  DRAKE_DEMAND(i >= 0);
  DRAKE_DEMAND(i < variable.derivatives().size());
  EXPECT_GT(0., variable.derivatives()(i));
}

inline static void CheckDerivativeNegativity(int, const double&) {}

}  // namespace test
}  // namespace automotive
}  // namespace drake
