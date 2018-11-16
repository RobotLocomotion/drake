#include "drake/solvers/solver_options.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {

GTEST_TEST(SolverOptionsTest, ToString) {
  SolverOptions dut;
  EXPECT_EQ(to_string(dut), "{SolverOptions empty}");

  const SolverId id1("id1");
  const SolverId id2("id2");

  dut.SetOption(id1, "some_double", 1.1);
  dut.SetOption(id1, "some_before", 1.2);
  dut.SetOption(id1, "some_int", 2);

  dut.SetOption(id2, "some_int", "3");
  dut.SetOption(id2, "some_string", "foo");

  EXPECT_EQ(
      to_string(dut),
      "{SolverOptions,"
      " id1:some_before=1.2,"
      " id1:some_double=1.1,"
      " id1:some_int=2,"
      " id2:some_int=3,"
      " id2:some_string=foo}");
}

GTEST_TEST(SolverOptionsTest, Ids) {
  using Set = std::unordered_set<SolverId>;

  SolverOptions dut;
  EXPECT_EQ(dut.GetSolverIds(), Set{});

  // Each type (double, int, string) can affect the "known IDs" result.
  const SolverId id1("id1");
  dut.SetOption(id1, "some_double", 0.0);
  EXPECT_EQ(dut.GetSolverIds(), Set({id1}));
  const SolverId id2("id2");
  dut.SetOption(id2, "some_int", 1);
  EXPECT_EQ(dut.GetSolverIds(), Set({id1, id2}));
  const SolverId id3("id3");
  dut.SetOption(id3, "some_string", "foo");
  EXPECT_EQ(dut.GetSolverIds(), Set({id1, id2, id3}));

  // Having the same ID used by in more than one type is okay.
  dut.SetOption(id1, "some_int", 2);
  dut.SetOption(id2, "some_string", "bar");
  dut.SetOption(id3, "some_double", 1.0);
  EXPECT_EQ(dut.GetSolverIds(), Set({id1, id2, id3}));
}

void UpdateMergeResult(const SolverOptions& solver_options1,
                       const SolverOptions& solver_options2,
                       SolverOptions* merge_option1_into_option2,
                       SolverOptions* merge_option2_into_option1) {
  *merge_option1_into_option2 = solver_options2;
  merge_option1_into_option2->Merge(solver_options1);
  *merge_option2_into_option1 = solver_options1;
  merge_option2_into_option1->Merge(solver_options2);
}

GTEST_TEST(SolverOptionsTest, Merge) {
  const SolverId id1("foo1");
  const SolverId id2("foo2");
  SolverOptions solver_options1, solver_options2;
  SolverOptions merge_option2_into_option1, merge_option1_into_option2;
  solver_options1.SetOption(id1, "key1", 1);
  EXPECT_NE(solver_options1, solver_options2);

  UpdateMergeResult(solver_options1, solver_options2,
                    &merge_option1_into_option2, &merge_option2_into_option1);
  EXPECT_EQ(solver_options1, merge_option1_into_option2);
  EXPECT_EQ(solver_options1, merge_option2_into_option1);

  solver_options2.SetOption(id1, "key1", 2);
  UpdateMergeResult(solver_options1, solver_options2,
                    &merge_option1_into_option2, &merge_option2_into_option1);
  EXPECT_EQ(merge_option1_into_option2, solver_options2);
  EXPECT_EQ(merge_option2_into_option1, solver_options1);

  // Now solver_options2 contains a key for solver foo, that is not contained in
  // solver_options1.
  solver_options2.SetOption(id1, "key2", 1);
  UpdateMergeResult(solver_options1, solver_options2,
                    &merge_option1_into_option2, &merge_option2_into_option1);
  EXPECT_EQ(merge_option1_into_option2, solver_options2);
  SolverOptions merge_option2_into_option1_expected = solver_options1;
  merge_option2_into_option1_expected.SetOption(id1, "key2", 1);
  EXPECT_EQ(merge_option2_into_option1, merge_option2_into_option1_expected);

  // Now solver_options1 contains a different solver id
  solver_options1.SetOption(id2, "key1", 0.1);
  UpdateMergeResult(solver_options1, solver_options2,
                    &merge_option1_into_option2, &merge_option2_into_option1);
  merge_option2_into_option1_expected.SetOption(id2, "key1", 0.1);
  EXPECT_EQ(merge_option2_into_option1, merge_option2_into_option1_expected);
  SolverOptions merge_option1_into_option2_expected = solver_options2;
  merge_option1_into_option2_expected.SetOption(id2, "key1", 0.1);
  EXPECT_EQ(merge_option1_into_option2, merge_option1_into_option2_expected);
}
}  // namespace solvers
}  // namespace drake
