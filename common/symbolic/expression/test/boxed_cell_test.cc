/* clang-format off to disable clang-format-includes */
#include "drake/common/symbolic/expression/all.h"
/* clang-format on */

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#define DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER
#include "drake/common/symbolic/expression/expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER

namespace drake {
namespace symbolic {
namespace internal {
namespace {

class BoxedCellTest : public ::testing::Test {
 protected:
  // These helper functions return a bare pointer to a newly-allocated cell.
  // The test case is responsible for deleting the cell. Tests pass the cell to
  // BoxedCell::SetSharedCell which takes over ownership. When the BoxedCell is
  // destroyed (typically, at the end of the test case's lexical scope), the
  // use_count of the cell will reach zero and it will be deleted as part of the
  // BoxedCell destructor. If BoxedCell has any bugs with its reference counts,
  // that will show up as leaks in our memory checkers (e.g., LSan or Memcheck).
  const ExpressionCell* MakeVarCell() { return new ExpressionVar(x_); }
  const ExpressionCell* MakeSqrtCell() { return new ExpressionSqrt(x_); }
  const ExpressionCell* MakeNaNCell() { return new ExpressionNaN(); }

  const BoxedCell zero_;
  const Variable x_{"x"};
};

TEST_F(BoxedCellTest, DefaultCtor) {
  const BoxedCell dut;
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Constant);
  EXPECT_FALSE(dut.is_kind<ExpressionKind::Add>());
  EXPECT_TRUE(dut.is_constant());
  EXPECT_EQ(dut.constant(), 0.0);
  EXPECT_EQ(dut.constant_or_nan(), 0.0);

  const BoxedCell other;
  EXPECT_TRUE(dut.trivially_equals(other));
}

TEST_F(BoxedCellTest, DoubleCtor) {
  const BoxedCell dut{0.25};
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Constant);
  EXPECT_FALSE(dut.is_kind<ExpressionKind::Add>());
  EXPECT_TRUE(dut.is_constant());
  EXPECT_EQ(dut.constant(), 0.25);
  EXPECT_EQ(dut.constant_or_nan(), 0.25);
  EXPECT_TRUE(dut.trivially_equals(dut));
  EXPECT_FALSE(dut.trivially_equals(zero_));
}

TEST_F(BoxedCellTest, CopyCtorConstant) {
  const BoxedCell original{0.25};
  const BoxedCell dut{original};
  EXPECT_EQ(original.constant(), 0.25);
  EXPECT_EQ(dut.constant(), 0.25);
}

TEST_F(BoxedCellTest, MoveCtorConstant) {
  BoxedCell original{0.25};
  BoxedCell dut{std::move(original)};
  EXPECT_EQ(original.constant(), 0.0);
  EXPECT_EQ(dut.constant(), 0.25);
}

TEST_F(BoxedCellTest, CopyAssignConstant) {
  const BoxedCell original{0.25};
  BoxedCell dut{0.333};
  dut = original;
  EXPECT_EQ(original.constant(), 0.25);
  EXPECT_EQ(dut.constant(), 0.25);
}

TEST_F(BoxedCellTest, MoveAssignConstant) {
  BoxedCell original{0.25};
  BoxedCell dut{0.333};
  dut = std::move(original);
  EXPECT_EQ(original.constant(), 0.0);
  EXPECT_EQ(dut.constant(), 0.25);
}

TEST_F(BoxedCellTest, Swap) {
  BoxedCell foo{1.0};
  BoxedCell bar{2.0};
  swap(foo, bar);
  EXPECT_EQ(foo.constant(), 2.0);
  EXPECT_EQ(bar.constant(), 1.0);
}

TEST_F(BoxedCellTest, Update) {
  BoxedCell dut;
  EXPECT_TRUE(dut.is_constant());
  EXPECT_EQ(dut.constant(), 0.0);

  dut.update_constant(0.25);
  EXPECT_TRUE(dut.is_constant());
  EXPECT_EQ(dut.constant(), 0.25);
}

TEST_F(BoxedCellTest, SetGetCell) {
  BoxedCell dut;
  dut.SetSharedCell(MakeVarCell());
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_TRUE(dut.is_kind<ExpressionKind::Var>());
  EXPECT_FALSE(dut.is_kind<ExpressionKind::Add>());
  EXPECT_FALSE(dut.is_constant());
  EXPECT_TRUE(std::isnan(dut.constant_or_nan()));
  EXPECT_TRUE(dut.trivially_equals(dut));
  EXPECT_FALSE(dut.trivially_equals(zero_));
  EXPECT_EQ(dut.cell().get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, TriviallyEquals) {
  BoxedCell boxed_var_cell;
  BoxedCell boxed_sqrt_cell;
  BoxedCell boxed_nan_cell;
  boxed_var_cell.SetSharedCell(MakeVarCell());
  boxed_sqrt_cell.SetSharedCell(MakeSqrtCell());
  boxed_nan_cell.SetSharedCell(MakeSqrtCell());
  std::array<const BoxedCell*, 4> items{
      // BR
      &zero_, &boxed_var_cell, &boxed_sqrt_cell, &boxed_nan_cell};
  for (const BoxedCell* first : items) {
    for (const BoxedCell* second : items) {
      EXPECT_EQ(first->trivially_equals(*second), first == second);
    }
  }
}

TEST_F(BoxedCellTest, Release) {
  BoxedCell dut;
  dut.SetSharedCell(MakeVarCell());
  dut = BoxedCell{0.25};
  EXPECT_TRUE(dut.is_constant());
  EXPECT_EQ(dut.constant(), 0.25);
}

TEST_F(BoxedCellTest, CopyCtorCell) {
  auto original = std::make_unique<BoxedCell>();
  original->SetSharedCell(MakeVarCell());
  const BoxedCell dut{*original};
  EXPECT_EQ(original->get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_TRUE(dut.trivially_equals(*original));
  EXPECT_EQ(dut.cell().use_count(), 2);

  original.reset();
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, MoveCtorCell) {
  BoxedCell original;
  original.SetSharedCell(MakeVarCell());
  const BoxedCell dut{std::move(original)};
  EXPECT_EQ(original.get_kind(), ExpressionKind::Constant);
  EXPECT_EQ(original.constant(), 0.0);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, CopyAssignCellOntoCell) {
  auto original = std::make_unique<BoxedCell>();
  original->SetSharedCell(MakeVarCell());
  BoxedCell dut;
  dut.SetSharedCell(MakeSqrtCell());
  dut = *original;
  EXPECT_EQ(original->get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_TRUE(dut.trivially_equals(*original));
  EXPECT_EQ(dut.cell().use_count(), 2);

  original.reset();
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, CopyAssignCellOntoSelf) {
  BoxedCell dut;
  dut.SetSharedCell(MakeVarCell());
  BoxedCell& self = dut;
  dut = self;
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_TRUE(dut.trivially_equals(dut));
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, CopyAssignCellOntoConstant) {
  auto original = std::make_unique<BoxedCell>();
  original->SetSharedCell(MakeVarCell());
  BoxedCell dut{0.25};
  dut = *original;
  EXPECT_EQ(original->get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_TRUE(dut.trivially_equals(*original));
  EXPECT_EQ(dut.cell().use_count(), 2);

  original.reset();
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, CopyAssignConstantOntoCell) {
  auto original = std::make_unique<BoxedCell>(0.25);
  BoxedCell dut{0.25};
  dut.SetSharedCell(MakeSqrtCell());
  dut = *original;
  EXPECT_EQ(original->get_kind(), ExpressionKind::Constant);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Constant);
  EXPECT_EQ(dut.constant(), 0.25);
  EXPECT_TRUE(dut.trivially_equals(*original));

  original.reset();
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Constant);
}

TEST_F(BoxedCellTest, MoveAssignCellOntoCell) {
  BoxedCell original;
  original.SetSharedCell(MakeVarCell());
  BoxedCell dut;
  dut.SetSharedCell(MakeSqrtCell());
  dut = std::move(original);
  EXPECT_EQ(original.get_kind(), ExpressionKind::Constant);
  EXPECT_EQ(original.constant(), 0.0);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, MoveAssignCellOntoConstant) {
  BoxedCell original;
  original.SetSharedCell(MakeVarCell());
  BoxedCell dut{0.25};
  dut = std::move(original);
  EXPECT_EQ(original.get_kind(), ExpressionKind::Constant);
  EXPECT_EQ(original.constant(), 0.0);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Var);
  EXPECT_EQ(dut.cell().use_count(), 1);
}

TEST_F(BoxedCellTest, MoveAssignConstantOntoCell) {
  BoxedCell original{0.25};
  BoxedCell dut;
  dut.SetSharedCell(MakeSqrtCell());
  dut = std::move(original);
  EXPECT_EQ(original.get_kind(), ExpressionKind::Constant);
  EXPECT_EQ(original.constant(), 0.0);
  EXPECT_EQ(dut.get_kind(), ExpressionKind::Constant);
  EXPECT_EQ(dut.constant(), 0.25);
}

// Sanity check for NaN cell vs BoxedCell::value_ being NaN.
// The two concepts are unrelated.
TEST_F(BoxedCellTest, CellIsNaN) {
  BoxedCell dut;
  dut.SetSharedCell(MakeNaNCell());
  EXPECT_EQ(dut.get_kind(), ExpressionKind::NaN);
  EXPECT_TRUE(dut.is_kind<ExpressionKind::NaN>());
  EXPECT_FALSE(dut.is_kind<ExpressionKind::Add>());
  EXPECT_FALSE(dut.is_constant());
  EXPECT_TRUE(std::isnan(dut.constant_or_nan()));
  EXPECT_TRUE(dut.trivially_equals(dut));
  EXPECT_FALSE(dut.trivially_equals(zero_));
  EXPECT_EQ(dut.cell().get_kind(), ExpressionKind::NaN);
  EXPECT_EQ(dut.cell().use_count(), 1);
  dut = BoxedCell{0.25};
  EXPECT_TRUE(dut.is_constant());
  EXPECT_EQ(dut.constant(), 0.25);
}

}  // namespace
}  // namespace internal
}  // namespace symbolic
}  // namespace drake
