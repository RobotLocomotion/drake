#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

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
  EXPECT_TRUE(
      CompareMatrices(dut.make_const_xpr(), Vector4d::LinSpaced(10.0, 13.0)));
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

TEST_F(PartialsTest, MatchSizeOf) {
  Partials dut{4, 2};

  // No-op to match size == 0.
  dut.MatchSizeOf(Partials{});
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(2)));

  // No-op to match size == 4.
  dut.MatchSizeOf(Partials{4, 0});
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(2)));

  // Inherits size == 4 from dut.
  Partials foo;
  foo.MatchSizeOf(dut);
  EXPECT_TRUE(CompareMatrices(foo.make_const_xpr(), Vector4d::Zero()));

  // Mismatched sizes.
  const Partials wrong{5, 0};
  DRAKE_EXPECT_THROWS_MESSAGE(foo.MatchSizeOf(wrong), ".*different sizes.*");
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

// Multiply a unit vector by non-finite scale factor and check our sparsity
// promise -- that zeros are left alone.
TEST_F(PartialsTest, MulNonFinite) {
  Partials dut{2, 1};
  dut.Mul(kInf);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{0.0, kInf}));

  dut = Partials{2, 1};
  dut.Mul(kNaN);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{0.0, kNaN}));
}

// Check that non-finite numbers in `storage_` are correctly reset to zero. If
// they were multiplied by zero instead of reset to zero, we'd end up with NaNs.
TEST_F(PartialsTest, MulNonFiniteThenSetZero) {
  Partials dut{Vector4d{0.0, 1.0, 2.0, -3.0}};
  dut.Mul(kInf);
  dut.SetZero();
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

// When the partials contain a NaN, make sure that rescaling it doesn't cause
// trouble.
TEST_F(PartialsTest, MulDerivativeNonFinite) {
  Partials dut{Vector2d{1.0, kNaN}};
  dut.Mul(2.0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{2.0, kNaN}));
  dut.Mul(kInf);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{kInf, kNaN}));
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
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d(0, 0, kInf, 0)));
}

TEST_F(PartialsTest, DivNonFinite) {
  Partials dut{4, 2};
  dut.Div(kNaN);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d(0, 0, kNaN, 0)));

  dut = Partials{4, 2};
  dut.Div(kInf);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

// When the partials contain a NaN, make sure that rescaling it doesn't cause
// trouble.
TEST_F(PartialsTest, DivDerivativeNonFinite) {
  Partials dut{Vector2d{1.0, kNaN}};
  dut.Div(2.0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{0.5, kNaN}));
  dut.Div(0.0);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{kInf, kNaN}));
}

// Spot check a few additions manually. Mostly, we rely on our knowledge that
// Add(...) is implemented in terms of AddScaled(1.0, ...) to delegate almost
// all testing to AddScaled() instead.
TEST_F(PartialsTest, Add) {
  Partials dut{4, 0};
  dut.Add(Partials(4, 1));
  dut.Add(Partials(4, 2));
  dut.Add(Partials(4, 3));
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Ones()));
}

// Spot check a few additions manually. For the most part, we rely on the
// Eigen equivalence test below for coverage.
TEST_F(PartialsTest, AddScaled) {
  Partials dut{4, 0};
  dut.AddScaled(2.0, Partials(4, 1));
  dut.AddScaled(3.0, Partials(4, 2));
  dut.AddScaled(4.0, Partials(4, 3));
  EXPECT_TRUE(
      CompareMatrices(dut.make_const_xpr(), Vector4d::LinSpaced(1.0, 4.0)));
}

TEST_F(PartialsTest, AddScaledNonFinite) {
  Partials dut{4, 0};
  dut.AddScaled(kInf, Partials(4, 1));
  dut.AddScaled(kNaN, Partials(4, 2));
  EXPECT_TRUE(
      CompareMatrices(dut.make_const_xpr(), Vector4d(1.0, kInf, kNaN, 0.0)));
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

  dut = Partials{4, 2};
  dut.AddScaled(kInf, Partials());
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

  dut = Partials{};
  dut.AddScaled(kInf, Partials(4, 2));
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d(0, 0, kInf, 0)));
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

// Rather than writing out dozens of acute test inputs and outputs by hand, we
// can test for equivalence with a known-good implementation, i.e., VectorXd.
//
// We'll set up sample inputs of each type, perform the same operations, and
// check that Partials produces the results as VectorXd, while using kcov to
// ensure that all of the branches in the implementation have been covered.
TEST_F(PartialsTest, AddScaledEquivalenceToEigen) {
  constexpr int kSize = 4;
  struct Twins {
    // Constructs a unit vector.
    explicit Twins(int unit_offset)
        : partials{kSize, unit_offset}, twin{Vector4d::Unit(unit_offset)} {}

    // Constructs a new value: this + scale*other.
    Twins AddScaled(double scale, const Twins& other, bool prescale) const {
      Twins result(*this);
      if (prescale) {
        // Force the partials into canonical form.
        result.partials.MakeMutableXpr();
      }
      result.partials.AddScaled(scale, other.partials);
      result.twin.noalias() += scale * other.twin;
      return result;
    }

    // The value under test.
    Partials partials;

    // Its expected value from the reference implementation.
    Eigen::Vector4d twin;

    // For debugging:
    // - The index of this data within the `values` vector below.
    int index{};
    // - The inputs that formed this value (this = lhs + scale * rhs).
    std::optional<int> lhs_index;
    std::optional<int> rhs_index;
    std::optional<double> rhs_scale;
  };

  std::vector<Twins> values;

  // Initialize the partials to the full set of unit vectors.
  // Pre-allocate the vector avoid extra moves during debugging.
  constexpr int kNumRandomCases = 100;
  values.reserve(1 + kSize + kNumRandomCases);
  // Add a zero and then all of the unit vectors.
  values.emplace_back(0);
  values.at(0).partials.SetZero();
  values.at(0).twin.setZero();
  for (int i = 0; i < kSize; ++i) {
    values.emplace_back(i);
    values.at(i).index = values.size() - 1;
  }

  // Set up our randomness.
  RandomGenerator generator(22);
  // Functor that chooses a random index within the `values.size()`.
  auto random_index = [&generator, &values]() -> int {
    // Bias towards the unit vectors at the start.
    if (generator() % 2 == 0) {
      return generator() % (kSize + 1);
    } else {
      return generator() % values.size();
    }
  };
  // Functor that chooses a random scale factor.
  std::uniform_real_distribution<double> random_real(-2.0, 2.0);
  auto random_scale = [&generator, &random_real]() -> double {
    return random_real(generator);
  };

  // Do lots of scaled additions.
  for (int i = 0; i < kNumRandomCases; ++i) {
    static_assert(RandomGenerator::min() == 0);
    const int a_index = random_index();
    const int b_index = random_index();
    const double scale = random_scale();
    const bool prescale = i % 2 == 1;

    const Twins& a = values.at(a_index);
    const Twins& b = values.at(b_index);
    values.emplace_back(a.AddScaled(scale, b, prescale));

    values.back().index = values.size() - 1;
    values.back().lhs_index = a_index;
    values.back().rhs_index = b_index;
    values.back().rhs_scale = scale;
  }

  // Check that all twins are identical. We need a somewhat non-zero tolerance
  // because our implementation multiplies together all scale factors before
  // broadcasting it onto the partials, whereas the VectorXd reference data
  // always is broadcast immediately, and the roundoff errors can be different
  // in those two cases.
  const double tolerance = 4 * std::numeric_limits<double>::epsilon();
  for (const Twins& value : values) {
    const std::string description = fmt::format(
        "...\n"
        "  for values[{}] = {}\n"
        "  when calculating a + s*b for\n"
        "  a = values[{}] = {}\n"
        "  b = values[{}] = {}\n"
        "  s = {}\n",
        value.index, fmt_eigen(value.twin.transpose()),
        value.lhs_index.value_or(-1),
        fmt_eigen(values.at(value.lhs_index.value_or(0)).twin.transpose()),
        value.rhs_index.value_or(-1),
        fmt_eigen(values.at(value.rhs_index.value_or(0)).twin.transpose()),
        value.rhs_scale.value_or(std::numeric_limits<double>::quiet_NaN()));
    VectorXd actual = value.partials.make_const_xpr();
    if (actual.size() == 0) {
      actual = VectorXd::Zero(kSize);
    }
    EXPECT_TRUE(CompareMatrices(actual, value.twin, tolerance)) << description;
  }
}

TEST_F(PartialsTest, AddDifferentSizes) {
  Partials dut{4, 2};
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Add(Partials(2, 0)), ".*different sizes.*");
}

TEST_F(PartialsTest, AddScaledDifferentSizes) {
  Partials dut{4, 2};
  DRAKE_EXPECT_THROWS_MESSAGE(dut.AddScaled(-1.0, Partials(2, 0)),
                              ".*different sizes.*");
}

TEST_F(PartialsTest, Resizing) {
  Partials dut;
  EXPECT_EQ(dut.make_const_xpr().size(), 0);

  // Assign a larger vector.
  dut.MakeMutableXpr() = Vector4d::Unit(1);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(1)));

  // Resize smaller, preserving the existing data.
  dut.MakeMutableXpr().conservativeResize(2);
  EXPECT_EQ(dut.size(), 2);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Unit(1)));

  // Resize larger, preserving the existing data.
  dut.MakeMutableXpr().conservativeResize(4);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(1)));

  // Resize smaller, not preserving any existing data.
  dut.MakeMutableXpr().resize(1);
  EXPECT_EQ(dut.size(), 1);
}

// We need to indirect self-move-assign through this function; doing it
// directly in the test code generates a compiler warning.
void MoveAssign(Partials* target, Partials* donor) {
  *target = std::move(*donor);
}

TEST_F(PartialsTest, MutableAssignmentSelf) {
  Partials dut{2, 1};
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Unit(1)));

  MoveAssign(&dut, &dut);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Unit(1)));
}

TEST_F(PartialsTest, MutableAssignmentEmpty) {
  Partials dut{2, 1};
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Unit(1)));

  Partials empty;
  dut = empty;
  EXPECT_EQ(dut.make_const_xpr().size(), 0);

  dut = Partials{2, 1};
  dut = std::move(empty);
  EXPECT_EQ(dut.make_const_xpr().size(), 0);
}

TEST_F(PartialsTest, MutableAssignmentEqualSize) {
  Partials dut{2, 1};
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Unit(1)));

  Partials other{2, 0};
  dut = other;
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Unit(0)));
}

TEST_F(PartialsTest, MutableAssignmentFromDefaultCtor) {
  Partials dut;
  EXPECT_EQ(dut.make_const_xpr().size(), 0);

  dut.MakeMutableXpr() = Vector4d::Unit(1);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(1)));
}

TEST_F(PartialsTest, MutableAssignmentFromUnitCtor) {
  Partials dut{2, 1};
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d::Unit(1)));

  dut.MakeMutableXpr() = Vector4d::Unit(1);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(1)));
}

TEST_F(PartialsTest, MutableAssignmentFromFullCtor) {
  Partials dut{Vector2d{1.0, 2.0}};
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{1.0, 2.0}));

  dut.MakeMutableXpr() = Vector4d::Unit(1);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(1)));
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
