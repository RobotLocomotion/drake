#include "drake/common/sorted_pair.h"

#include <sstream>
#include <unordered_map>
#include <vector>

#include <fmt/ranges.h>
#include <gtest/gtest.h>

namespace drake {
namespace {

struct NoDefaultCtor {
  explicit NoDefaultCtor(int value_in) : value(value_in) {}

  bool operator<(const NoDefaultCtor& other) const {
    return value < other.value;
  }

  int value{};
};

// Verifies behavior of the default constructor.
GTEST_TEST(SortedPair, Default) {
  SortedPair<int> x;
  EXPECT_EQ(x.first(), 0);
  EXPECT_EQ(x.second(), 0);

  SortedPair<double> y;
  EXPECT_EQ(y.first(), 0.0);
  EXPECT_EQ(y.second(), 0.0);
}

GTEST_TEST(SortedPair, FromStdPair) {
  std::pair<int, int> p{1, 2};
  SortedPair<int> sp(p);
  EXPECT_EQ(sp.first(), 1);
  EXPECT_EQ(sp.second(), 2);

  std::pair<int, int> p2{2, 1};
  SortedPair<int> sp2(p2);
  EXPECT_EQ(sp2.first(), 1);
  EXPECT_EQ(sp2.second(), 2);
}

// Usable even with non-default-constructible types
GTEST_TEST(SortedPair, WithoutDefaultCtor) {
  NoDefaultCtor a(2);
  NoDefaultCtor b(1);
  SortedPair<NoDefaultCtor> x(a, b);
  EXPECT_EQ(x.first().value, 1);
  EXPECT_EQ(x.second().value, 2);
}

// Verifies sorting occurs.
GTEST_TEST(SortedPair, Values) {
  SortedPair<int> x(3, 2);
  EXPECT_EQ(x.first(), 2);
  EXPECT_EQ(x.second(), 3);
}

// Verifies that the type casting copy constructor works as desired.
GTEST_TEST(SortedPair, Casting) {
  SortedPair<double> x = SortedPair<int>(3, 2);
  EXPECT_EQ(x.first(), 2.0);
  EXPECT_EQ(x.second(), 3.0);
}

// Verifies that 'set()' works properly.
GTEST_TEST(SortedPair, Set) {
  SortedPair<double> x;
  x.set(1.0, 2.0);
  EXPECT_EQ(x.first(), 1.0);
  EXPECT_EQ(x.second(), 2.0);
  x.set(5.0, 3.0);
  EXPECT_EQ(x.first(), 3.0);
  EXPECT_EQ(x.second(), 5.0);
}

// Verifies that the move assignment operator and move constructor work.
GTEST_TEST(SortedPair, Move) {
  auto a = std::make_unique<int>(2);
  auto b = std::make_unique<int>(1);
  SortedPair<std::unique_ptr<int>> y(std::move(a), std::move(b));
  SortedPair<std::unique_ptr<int>> x;
  x = std::move(y);
  EXPECT_TRUE(x.first() < x.second());
  EXPECT_EQ(y.first().get(), nullptr);
  EXPECT_EQ(y.second().get(), nullptr);
  y = SortedPair<std::unique_ptr<int>>(std::move(x));
  EXPECT_TRUE(y.first() < y.second());
  EXPECT_EQ(x.first().get(), nullptr);
  EXPECT_EQ(x.second().get(), nullptr);
}

// Checks the assignment operator.
GTEST_TEST(SortedPair, Assignment) {
  SortedPair<int> x;
  SortedPair<int> y(3, 2);
  EXPECT_EQ(&(x = y), &x);
  EXPECT_EQ(x.first(), 2.0);
  EXPECT_EQ(x.second(), 3.0);
}

// Checks the equality operator.
GTEST_TEST(SortedPair, Equality) {
  SortedPair<int> x(1, 2), y(2, 1);
  EXPECT_EQ(x, y);
}

// Checks the comparison operators.
GTEST_TEST(SortedPair, Comparison) {
  SortedPair<int> x(1, 2), y(2, 2);
  EXPECT_FALSE(x < x);
  EXPECT_FALSE(x > x);
  EXPECT_TRUE(x <= x);
  EXPECT_TRUE(x >= x);
  EXPECT_TRUE(x < y);
}

// Checks the swap function.
GTEST_TEST(SortedPair, Swap) {
  SortedPair<int> x(1, 2), y(3, 4);
  std::swap(x, y);
  EXPECT_EQ(x.first(), 3);
  EXPECT_EQ(x.second(), 4);
  EXPECT_EQ(y.first(), 1);
  EXPECT_EQ(y.second(), 2);
}

// Checks hash keys.
GTEST_TEST(SortedPair, Hash) {
  SortedPair<int> x(1, 2), y(2, 4);
  std::unordered_map<SortedPair<int>, int> hash;
  hash[x] = 11;
  hash[y] = 13;
  EXPECT_EQ(hash[x], 11);
  EXPECT_EQ(hash[y], 13);
}

// Checks expansion with STL vector.
GTEST_TEST(SortedPair, VectorExp) {
  std::vector<std::unique_ptr<SortedPair<int>>> v;
  v.emplace_back(std::make_unique<SortedPair<int>>(1, 2));
  v.resize(v.capacity() + 1);
  EXPECT_EQ(v.front()->first(), 1);
  EXPECT_EQ(v.front()->second(), 2);
}

// Tests the MakeSortedPair operator.
GTEST_TEST(SortedPair, MakeSortedPair) {
  EXPECT_EQ(SortedPair<int>(1, 2), MakeSortedPair(1, 2));
}

// Tests that formatting support is reasonable.
GTEST_TEST(SortedPair, Format) {
  SortedPair<int> pair{8, 7};
  // We don't care exactly what fmt does here, just that it looks sensible.
  // It's OK to to update this goal string in case fmt changes a bit over time.
  EXPECT_EQ(fmt::to_string(pair), "(7, 8)");
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Tests the streaming support.
GTEST_TEST(SortedPair, DeprecatedWriteToStream) {
  SortedPair<int> pair{8, 7};
  std::stringstream ss;
  ss << pair;
  EXPECT_EQ(ss.str(), "(7, 8)");
}
#pragma GCC diagnostic pop

GTEST_TEST(SortedPair, StructuredBinding) {
  SortedPair<int> pair{8, 7};

  // Copy access.
  {
    auto [a, b] = pair;
    EXPECT_EQ(a, pair.first());
    EXPECT_EQ(b, pair.second());
  }

  // Const reference access.
  {
    auto& [a, b] = pair;
    EXPECT_EQ(&a, &pair.first());
    EXPECT_EQ(&b, &pair.second());
  }

  // Access via range iterators.
  {
    std::vector<SortedPair<int>> pairs({{1, 2}, {3, 6}});
    for (const auto& [a, b] : pairs) {
      EXPECT_EQ(2 * a, b);
    }
  }
}

}  // namespace
}  // namespace drake
