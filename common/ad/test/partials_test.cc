#include <limits>

#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

// An empty fixture for later expansion.
class PartialsTest : public ::testing::Test {};

TEST_F(PartialsTest, DefaultCtor) {
  const Partials dut;
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.make_const_xpr().size(), 0);
}

TEST_F(PartialsTest, UnitCtor2Arg) {
  const Partials dut{4, 2};
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(2)));
}

TEST_F(PartialsTest, UnitCtor3Arg) {
  const Partials dut{4, 2, -1.0};
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), -Vector4d::Unit(2)));
}

TEST_F(PartialsTest, UnitCtorZeroCoeff) {
  const Partials dut{4, 2, 0.0};
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

TEST_F(PartialsTest, UnitCtorInsanelyLarge) {
  const Eigen::Index terabyte = Eigen::Index{1} << 40;
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(terabyte, 0), ".*too large.*");
}

TEST_F(PartialsTest, UnitCtorMisuse) {
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(4, -1), ".*is negative.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(-1, 0), ".*is negative.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(2, 2), ".*strictly less.*");
}

TEST_F(PartialsTest, FullCtor) {
  const Partials dut{Vector4d::LinSpaced(10.0, 13.0)};
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(
      dut.make_const_xpr(), Vector4d::LinSpaced(10.0, 13.0)));
}

TEST_F(PartialsTest, FullCtorEmpty) {
  const Partials dut{VectorXd{}};
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.make_const_xpr().size(), 0);
}

TEST_F(PartialsTest, FullCtorZeros) {
  const Partials dut{Vector4d::Zero()};
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

TEST_F(PartialsTest, SetZeroFromDefaultCtor) {
  Partials dut;
  dut.SetZero();
  EXPECT_EQ(dut.size(), 0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), VectorXd{}));
}

TEST_F(PartialsTest, SetZeroFromUnitCtor) {
  Partials dut{4, 2};
  dut.SetZero();
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

TEST_F(PartialsTest, SetZeroFromFullCtor) {
  Partials dut{Vector4d::LinSpaced(10.0, 13.0)};
  dut.SetZero();
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

TEST_F(PartialsTest, Mul) {
  Partials dut{4, 2};

  dut.Mul(-2.0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), -2 * Vector4d::Unit(2)));

  dut.Mul(0.0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));

  dut = Partials{Vector2d{1.0, 2.0}};

  dut.Mul(-2.0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{-2.0, -4.0}));

  dut.Mul(0.0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Zero()));
}

TEST_F(PartialsTest, Div) {
  Partials dut{4, 2};
  dut.Div(-0.5);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), -2 * Vector4d::Unit(2)));
  dut.SetZero();
  dut.Div(-0.5);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), -Vector4d::Zero()));
}

TEST_F(PartialsTest, DivZero) {
  Partials dut{4, 2};
  dut.Div(0.0);
  const double kInf = std::numeric_limits<double>::infinity();
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(),
                              Vector4d(kNaN, kNaN, kInf, kNaN)));
}

TEST_F(PartialsTest, Add) {
  Partials dut{4, 0};
  dut.Add(Partials(4, 1));
  dut.Add(Partials(4, 2));
  dut.Add(Partials(4, 3));
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Ones()));
}

TEST_F(PartialsTest, AddScaled) {
  Partials dut{4, 0};
  dut.AddScaled(2.0, Partials(4, 1));
  dut.AddScaled(3.0, Partials(4, 2));
  dut.AddScaled(4.0, Partials(4, 3));
  EXPECT_TRUE(CompareMatrices(
      dut.make_const_xpr(), Vector4d::LinSpaced(1.0, 4.0)));
}

TEST_F(PartialsTest, AddRhsEmpty) {
  Partials dut{4, 2};
  dut.Add(Partials{});
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(2)));
}

TEST_F(PartialsTest, AddScaledRhsEmpty) {
  Partials dut{4, 2};
  dut.AddScaled(1.0, Partials{});
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(2)));
}

TEST_F(PartialsTest, AddLhsEmpty) {
  Partials dut;
  dut.Add(Partials(4, 2));
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(2)));
}

TEST_F(PartialsTest, AddScaledLhsEmpty) {
  Partials dut;
  dut.AddScaled(-1.0, Partials(4, 2));
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), -Vector4d::Unit(2)));
}

TEST_F(PartialsTest, AddBothEmpty) {
  Partials dut;
  dut.Add(dut);
  EXPECT_EQ(dut.size(), 0);
}

TEST_F(PartialsTest, AddScaledBothEmpty) {
  Partials dut;
  dut.AddScaled(1.0, dut);
  EXPECT_EQ(dut.size(), 0);
}

TEST_F(PartialsTest, AddDifferentSizes) {
  Partials dut{4, 2};
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Add(Partials(2, 0)),
      ".*different sizes.*");
}

TEST_F(PartialsTest, AddScaledDifferentSizes) {
  Partials dut{4, 2};
  DRAKE_EXPECT_THROWS_MESSAGE(dut.AddScaled(-1.0, Partials(2, 0)),
      ".*different sizes.*");
}

TEST_F(PartialsTest, DefaultCtorMutableGetter) {
  Partials dut;
  EXPECT_EQ(dut.get_raw_storage_mutable().size(), 0);
  dut.get_raw_storage_mutable().resize(4);
  EXPECT_EQ(dut.get_raw_storage_mutable().size(), 4);
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
