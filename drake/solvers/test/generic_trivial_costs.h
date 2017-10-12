#pragma once

#include <cstddef>
#include <limits>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/number_traits.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/function.h"

namespace drake {
namespace solvers {
namespace test {

// A generic cost derived from Constraint class. This is meant for testing
// adding a cost to optimization program, and the cost is in the form of a
// derived class of Constraint.
class GenericTrivialCost1 : public Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GenericTrivialCost1)

  GenericTrivialCost1() : Cost(3), private_val_(2) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              Eigen::VectorXd& y) const override {
    y.resize(1);
    y(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              AutoDiffVecXd& y) const override {
    y.resize(1);
    y(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

 private:
  // Add a private data member to make sure no slicing on this class, derived
  // from Constraint.
  double private_val_{0};
};

// A generic cost. This class is meant for testing adding a cost to the
// optimization program, by calling `MathematicalProgram::MakeCost` to
// convert this class to a ConstraintImpl object.
class GenericTrivialCost2 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericTrivialCost2)

  GenericTrivialCost2() = default;

  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(detail::VecIn<ScalarType> const& x,
            // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
            detail::VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = x(0) * x(0) - x(1) * x(1) + 2;
  }
};

}  // namespace test
}  // namespace solvers
}  // namespace drake
