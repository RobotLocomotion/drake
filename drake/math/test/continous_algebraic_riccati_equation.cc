#include "drake/math/continuous_algebraic_riccati_equation.h"
#include <gtest/gtest.h>
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"

using Eigen::MatrixXd;

namespace drake {
namespace math {
namespace {
  
  void TestCare(const Eigen::Ref<const MatrixXd>& A, // passing matricies and values
                const Eigen::Ref<const MatrixXd>& B,
                const Eigen::Ref<const MatrixXd>& Q,
                const Eigen::Ref<const MatrixXd>& R){
  MatrixXd X = ContinousAlgebraicRicattiEqation(A,B,Q,R); // setting matrix X as the return from CARE
  
  EXPECT_TRUE(CompareMatrices(X, X.transpose(), 1E-10, MatrixCompareType::absolute));
    int n = X.rows();
    Eigen::SelfAdjointEigenSolver<MatrixXd> es(X);
    for (int i = 0; i < n; i++) {
      EXPECT_GE(es.eigenvalues()[i], 0);
  }
  
  MatrixXd Y = (A.transpose*X)+
                     (X*A)-(X*B*R.inverse*B.transpose)+
                     (C.transpose*C); // Solving the Continous-Time AR Equation using A^T*X+X*A-XBR^-1+C^T*c = 0 

  
  EXPECT_TRUE(CompareMatrices(Y, MatrixXd::Zero(n, n), 1E-10,
                              MatrixCompareType::absolute));
}
  

    }
  }
}
