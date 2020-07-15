#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * min 2x0 + x2
 * s.t ⎡x0 x1⎤ is psd,
 *     ⎣x1 x0⎦
 *     ⎡x0 x2⎤ is psd, and
 *     ⎣x2 x0⎦
 *     x1 == 1.
 *     x0 >= 0.5
 *     x2 <= 2
 * The optimal solution is x = (1, 1, -1).
 */
class SDPwithOverlappingVariables1 : public ::testing::Test {
 public:
  SDPwithOverlappingVariables1();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * min 2x0 + x1
 * s.t ⎡x0 x1⎤ is psd,
 *     ⎣x1 x0⎦
 *     2 <= x0 <= 3
 *     1 <= x1
 * The optimal solution is x = (2, 1).
 */
class SDPwithOverlappingVariables2 : public ::testing::Test {
 public:
  SDPwithOverlappingVariables2();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector2<symbolic::Variable> x_;
};

/**
 * This is the example in CSDP 6.2.0 User's Guide
 * max tr(C1 * X1) + tr(C2 * X2)
 * s.t tr(A1 * X1) + y(0) = 1
 *     tr(A2 * X2) + y(1) = 2
 *     X1, X2 are psd.
 *     y(0), y(1) ≥ 0
 * where C1 = ⎡2 1⎤
 *            ⎣1 2⎦
 *       C2 = ⎡3 0 1⎤
 *            ⎢0 2 0⎥
 *            ⎣1 0 3⎦
 *       A1 = ⎡3 1⎤
 *            ⎣1 3⎦
 *       A2 = ⎡3 0 1⎤
 *            ⎢0 4 0⎥
 *            ⎣1 0 5⎦
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
 * 0 ≤ x(0)
 * 0 ≤ x(1) ≤ 5;
 * -1 ≤ x(2)
 *       x(3) ≤ 10
 * -2 ≤ x(4) ≤ 5
 *  0 ≤ x(5) ≤ 0
 *  1 ≤ x(6) ≤ 1
 * -inf≤x(7) ≤ inf
 */
class LinearProgramBoundingBox1 : public ::testing::Test {
 public:
  LinearProgramBoundingBox1();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 8, 1> x_;
};

/**
 * A simple linear program.
 * min x(0) + 2x(1) +  3x(2)
 * s.t 2x(0) + 3x(1) + x(2) = 1
 *     -2x(2) + x(0) ≤ -1
 *     2x(1) + x(0) ≥ -2
 *     -2 ≤ -x(0) + 3x(2) ≤ 3
 *     x(0) + x(1) + 4x(2) = 3
 * The optimal solution is x = (5 / 13, -2 / 13, 9 / 13). The optimal cost is
 * (28 / 13).
 */
class CsdpLinearProgram2 : public ::testing::Test {
 public:
  CsdpLinearProgram2();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * A linear program with both linear (in)equality constraints and bounding box
 * constraint.
 * max 2x(0) + 3x(1) + 4x(2) + 3
 * s.t x(0) + 2x(1) + 3x(2) = 3
 *     2x(0) - x(2) ≥ -1
 *     x(1) - 3x(2) ≤ 5
 *     -4 ≤ x(0) + x(2) ≤ 9
 *     -1 ≤ x(0) ≤ 10
 *     x(1) ≤ 8
 * The optimal solution is (10, -2/3, -17/9), the optimal cost is 121 / 9
 */
class CsdpLinearProgram3 : public ::testing::Test {
 public:
  CsdpLinearProgram3();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * A trivial SDP
 * max X1(0, 1) + X1(1, 2)
 * s.t X1 ∈ ℝ³ˣ³ is psd
 *     X1(0, 0) + X1(1, 1) + X1(2, 2) = 1
 *     X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) ≤ 0
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
 * The optimal solution is X1 = [0 0; 0 0], y = 1 / 3, The optimal cost is 1/3.
 */
class TrivialSDP2 : public ::testing::Test {
 public:
  TrivialSDP2();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Matrix2<symbolic::Variable> X1_;
  symbolic::Variable y_;
};

/**
 * Test a problem with LorentzConeConstraint
 * max x(0)
 * s.t 2x(0) + 1 >= sqrt((3x(1)+2)² + (x(2)+x(0)+3)²)
 *     x(0) + x(1) + x(2) = 10
 *     x(1) >= 0
 *     x(2) >= 0
 * The optimal solution is (10, 0, 0).
 */
class TrivialSOCP1 : public ::testing::Test {
 public:
  TrivialSOCP1();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * max x(1)
 * s.t x(0) + 2 >= sqrt((x(0) + x(1) + 1)² + (x(0) - x(1) + 1)²)
 * The optimal solution is (0, 1)
 */
class TrivialSOCP2 : public ::testing::Test {
 public:
  TrivialSOCP2();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector2<symbolic::Variable> x_;
};

/**
 * max -x(1)
 * s.t (2x(0) + 2)(3x(1) + 4) >= sqrt((x(0) + 2)² + (3x(0) + x(1) + 1)²)
 *      2x(0) + 2 >= 0
 *      3x(1) + 4 >= 0
 * The optimal solution is at (-0.1, 2 - sqrt(7.1))
 */
class TrivialSOCP3 : public ::testing::Test {
 public:
  TrivialSOCP3();

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector2<symbolic::Variable> x_;
};
}  // namespace solvers
}  // namespace drake
