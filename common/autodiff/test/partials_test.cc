#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/autodiff/internal/static_unit_vector.h"
#include "drake/common/random.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace internal {
namespace {

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

class PartialsTest : public ::testing::Test {
 protected:
};

// TODO(jwnimmer-tri) Add a tranche of 1d test cases; the logic is magical.
// TODO(jwnimmer-tri) Test that Add(known_zero) still propagtes the size().

TEST_F(PartialsTest, DefaultCtor) {
  const Partials dut;
  EXPECT_TRUE(dut.is_known_zero());
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.make_const_xpr().size(), 0);
}

TEST_F(PartialsTest, UnitCtor2Arg) {
  const Partials dut{4, 2};
  EXPECT_FALSE(dut.is_known_zero());
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(2)));
}

TEST_F(PartialsTest, UnitCtor3Arg) {
  const Partials dut{4, 2, -1.0};
  EXPECT_FALSE(dut.is_known_zero());
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), -Vector4d::Unit(2)));
}

TEST_F(PartialsTest, UnitCtorZeroCoeff) {
  const Partials dut{4, 2, 0.0};
  EXPECT_TRUE(dut.is_known_zero());
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

TEST_F(PartialsTest, UnitCtorLarge) {
  constexpr int big = 1200;
  static_assert(big > kMaxStaticVectorSize, "Test case needs updating");
  const Partials dut(big, 3);
  EXPECT_EQ(dut.size(), big);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), VectorXd::Unit(big, 3)));
}

TEST_F(PartialsTest, UnitCtorInsanelyLarge) {
  const Eigen::Index terabyte = Eigen::Index{1} << 40;
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(terabyte, 0), ".*too large.*");
}

// Check the constructor's special cases when value.size() == 1.
TEST_F(PartialsTest, UnitCtorSize1Value) {
  const VectorXd basis1d = VectorXd::Constant(1, 2.0);
  const Partials unit(basis1d);
  EXPECT_TRUE(CompareMatrices(unit.make_const_xpr(), basis1d));

  const VectorXd zero1d = VectorXd::Zero(1);
  const Partials empty(zero1d);
  EXPECT_TRUE(CompareMatrices(empty.make_const_xpr(), zero1d));
}

TEST_F(PartialsTest, UnitCtorMisuse) {
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(4, -1), ".*is negative.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(-1, 0), ".*is negative.*");
  DRAKE_EXPECT_THROWS_MESSAGE(Partials(2, 2), ".*strictly less.*");
}

TEST_F(PartialsTest, FullCtor) {
  const Partials dut{Vector4d::LinSpaced(10.0, 13.0)};
  EXPECT_FALSE(dut.is_known_zero());
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(
      dut.make_const_xpr(), Vector4d::LinSpaced(10.0, 13.0)));
}

TEST_F(PartialsTest, FullCtorEmpty) {
  const Partials dut{VectorXd{}};
  EXPECT_TRUE(dut.is_known_zero());
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.make_const_xpr().size(), 0);
}

TEST_F(PartialsTest, FullCtorZeros) {
  const Partials dut{Vector4d::Zero()};
  // It would be plausible for the dut to notice the all-zero value and set
  // itself to empty in response. At the moment, it doesn't check for that
  // because it doesn't seem like a worthwhile optimization.
  EXPECT_FALSE(dut.is_known_zero());
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

TEST_F(PartialsTest, SetZeroFromFullCtorLarge) {
  constexpr int big = 1200;
  static_assert(big > kMaxStaticVectorSize, "Test case needs updating");
  const VectorXd basis = VectorXd::LinSpaced(big, 0.0, 1.0);
  Partials dut(basis);
  dut.SetZero();
  EXPECT_EQ(dut.size(), big);
  EXPECT_TRUE(dut.make_const_xpr().isZero(0 /* precision */));
}

TEST_F(PartialsTest, MutableXprFromDefaultCtor) {
  Partials dut;
  EXPECT_EQ(dut.MakeMutableXpr().size(), 0);
  dut.MakeMutableXpr().resize(4) = Vector4d::Unit(3);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(3)));
}

TEST_F(PartialsTest, MutableXprFromUnitCtor) {
  Partials dut{2, 1, -1.0};
  EXPECT_TRUE(CompareMatrices(dut.MakeMutableXpr(), -Vector2d::Unit(1)));
  dut.MakeMutableXpr().resize(4) = Vector4d::Unit(3);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(3)));
}

TEST_F(PartialsTest, MutableXprFromFullCtor) {
  Partials dut{Vector2d{1.0, 2.0}};
  EXPECT_TRUE(CompareMatrices(dut.MakeMutableXpr(), Vector2d{1.0, 2.0}));
  dut.MakeMutableXpr().resize(4) = Vector4d::Unit(3);
  EXPECT_EQ(dut.size(), 4);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Unit(3)));
}

TEST_F(PartialsTest, Mul) {
  Partials dut{4, 2};

  dut.Mul(-2.0);
  EXPECT_FALSE(dut.is_known_zero());
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), -2 * Vector4d::Unit(2)));

  dut.Mul(0.0);
  EXPECT_TRUE(dut.is_known_zero());
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));

  dut = Partials{Vector2d{1.0, 2.0}};

  dut.Mul(-2.0);
  EXPECT_FALSE(dut.is_known_zero());
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector2d{-2.0, -4.0}));

  dut.Mul(0.0);
  EXPECT_TRUE(dut.is_known_zero());
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
  EXPECT_TRUE(CompareMatrices(
      dut.make_const_xpr(), Vector4d::LinSpaced(1.0, 4.0)));
}

// These special cases are too rare to fall under the Eigen equivalence test.
TEST_F(PartialsTest, AddScaledUnitVectors) {
  Partials dut{4, 0};
  dut.AddScaled(2.0, dut);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), 3 * Vector4d::Unit(0)));
  dut.AddScaled(-1.0, dut);
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
}

// These special cases are too rare to fall under the Eigen equivalence test.
TEST_F(PartialsTest, AddScaledFullVectors) {
  const Vector4d basis = Vector4d::LinSpaced(10.0, 13.0);
  Partials dut{basis};
  dut.AddScaled(2.0, Partials(dut));
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), 3 * basis));
  dut.AddScaled(-1.0, Partials(dut));
  EXPECT_TRUE(CompareMatrices(dut.make_const_xpr(), Vector4d::Zero()));
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
    explicit Twins(int unit_offset, double coeff = 1.0)
        : partials{kSize, unit_offset, coeff},
          twin{coeff * Vector4d::Unit(unit_offset)} {}

    // Constructs a new value: this + scale*other.
    Twins AddScaled(double scale, const Twins& other, bool inplace) const {
      Twins result(*this);
      if (inplace) {
        // Force the result to work within exclusively-owned storage.
        result.partials.MakeMutableXpr();
      }
      result.partials.AddScaled(scale, other.partials);
      result.twin.noalias() += scale * other.twin;
      return result;
    }

    // The value under test.
    Partials partials;

    // It's expected value from the reference implementation.
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
  values.reserve(1000);
  // Add a zero and then all of the unit vectors.
  values.emplace_back(0, 0.0);
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
  for (int i = 0; i < 100; ++i) {
    static_assert(RandomGenerator::min() == 0);
    const int a_index = random_index();
    const int b_index = random_index();
    const double scale = random_scale();
    const bool inplace = i % 2 == 1;

    const Twins& a = values.at(a_index);
    const Twins& b = values.at(b_index);
    values.emplace_back(a.AddScaled(scale, b, inplace));

    values.back().index = values.size() - 1;
    values.back().lhs_index = a_index;
    values.back().rhs_index = b_index;
    values.back().rhs_scale = scale;
  }

  // Check that all twins are identical. We need a somewhat non-zero tolerance
  // because our implementation multiplies together all scale factors before
  // broadcasting it onto the partials, whereas the VectorXd reference data
  // always is brodcast immediately, and the roundoff errors can be different
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
        value.index, value.twin.transpose(),
        value.lhs_index.value_or(-1),
        values.at(value.lhs_index.value_or(0)).twin.transpose(),
        value.rhs_index.value_or(-1),
        values.at(value.rhs_index.value_or(0)).twin.transpose(),
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
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Add(Partials(2, 0)),
      ".*different sizes.*");
}

TEST_F(PartialsTest, AddScaledDifferentSizes) {
  Partials dut{4, 2};
  DRAKE_EXPECT_THROWS_MESSAGE(dut.AddScaled(-1.0, Partials(2, 0)),
      ".*different sizes.*");
}

}  // namespace
}  // namespace internal
}  // namespace drake
