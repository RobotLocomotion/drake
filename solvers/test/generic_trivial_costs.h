#pragma once

#include <cstddef>
#include <limits>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/solvers/cost.h"

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
              Eigen::VectorXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    DoEvalGeneric(x, y);
  }

 private:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const {
    y->resize(1);
    (*y)(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

  // Add a private data member to make sure no slicing on this class, derived
  // from Constraint.
  double private_val_{0};
};


}  // namespace test
}  // namespace solvers
}  // namespace drake
