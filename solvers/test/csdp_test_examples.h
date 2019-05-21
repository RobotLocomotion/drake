#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * min 2 * x0 + x2
 * s.t [x0 x1] is psd
 *     [x1 x0]

 *     [x0 x2] is psd
 *     [x2 x0]
 *     x1 == 1
 * the optimal solution is x = (1, 1, -1).
 */
class SDPwithOverlappingVariables : public ::testing::Test {
 public:
  SDPwithOverlappingVariables();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * This is the example in CSDP 6.2.0 User's Guide
 * max tr(C1 * X1) + tr(C2 * X2)
 * s.t tr(A1 * X1) + y(0) = 1
 *     tr(A2 * X2) + y(1) = 2
 *     X1, X2 are psd.
 *     y(0), y(1) >= 0
 * where C1 = [2 1]
 *            [1 2]
 *       C2 = [3 0 1]
 *            [0 2 0]
 *            [1 0 3]
 *       A1 = [3 1]
 *            [1 3]
 *       A2 = [3 0 1]
 *            [0 4 0]
 *            [1 0 5]
 */
class CsdpDocExample : public ::testing::Test {
 public:
  CsdpDocExample();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Matrix2<symbolic::Variable> X1_;
  Matrix3<symbolic::Variable> X2_;
  Vector2<symbolic::Variable> y_;
};

/**
 * A simple linear program withou only bounding box constraint.
 * max -x(0) + x(1) - 2 *x(2) + 3 * x(3) + x(4) + 1
 * 0 <= x(0)
 * 0 <= x(1) <= 5;
 * -1 <= x(2)
 *       x(3) <= 10
 * -2 <= x(4) <= 5
 *  0 <= x(5) <= 0
 *  1 <= x(6) <= 1
 */
class LinearProgram1 : public ::testing::Test {
 public:
  LinearProgram1();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 7, 1> x_;
};

/**
 * A simple linear program.
 * min x(0) + 2 * x(1) +  3 * x(2)
 * s.t 2 * x(0) + 3 * x(1) + x(2) = 1
 *     -2 * x(2) + x(0) <= -1
 *     2 * x(1) + x(0) >= -2
 *     -2 <= -x(0) + 3 * x(2) <= 3
 *     x(0) + x(1) + 4 * x(2) = 3
 */
class LinearProgram2 : public ::testing::Test {
 public:
  LinearProgram2();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * A trivial SDP
 * max X1(0, 1) + X1(1, 2)
 * s.t X1 ∈ ℝ³ˣ³ is psd
 *     X1(0, 0) + X1(1, 1) + X1(2, 2) = 1
 *     X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) <= 0
 */
class TrivialSDP1 : public ::testing::Test {
 public:
  TrivialSDP1();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Matrix3<symbolic::Variable> X1_;
};

/**
 * max y
 * X1 ∈ ℝ²ˣ² is psd
 * I + F1 * y + F2 * X1(0, 0) is psd.
 * X1(0, 0) + 2 * X1(1, 1) + 3 * y = 1
 * where F1 = [1 2; 2 3], F2 = [2 0; 0 4]
 */
class TrivialSDP2 : public ::testing::Test {
 public:
  TrivialSDP2();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Matrix2<symbolic::Variable> X1_;
  symbolic::Variable y_;
};
}  // namespace solvers
}  // namespace drake
