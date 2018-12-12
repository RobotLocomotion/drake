#include "drake/solvers/solver_options.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

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
  dut.SetOption(id1, "some_string", "bar");
  dut.SetOption(id1, "some_double", 1.0);
  EXPECT_EQ(dut.GetSolverIds(), Set({id1, id2, id3}));
}

GTEST_TEST(SolverOptionsTest, Merge) {
  const SolverId id1("foo1");
  const SolverId id2("foo2");
  SolverOptions dut, dut_expected;

  SolverOptions foo;
  foo.SetOption(id1, "key1", 1);
  dut.Merge(foo);
  dut_expected.SetOption(id1, "key1", 1);
  EXPECT_EQ(dut, dut_expected);

  // Duplicate solver and key. No-op
  foo.SetOption(id1, "key1", 2);
  dut.Merge(foo);
  EXPECT_EQ(dut, dut_expected);

  // foo contains a key that is not contained in dut.
  foo.SetOption(id1, "key2", 2);
  dut.Merge(foo);
  dut_expected.SetOption(id1, "key2", 2);
  EXPECT_EQ(dut, dut_expected);

  // foo contains a solver that is not contained in dut.
  foo.SetOption(id2, "key1", 1);
  dut.Merge(foo);
  dut_expected.SetOption(id2, "key1", 1);
  EXPECT_EQ(dut, dut_expected);
}

GTEST_TEST(SolverOptionsTest, CheckOptionKeysForSolver) {
  const SolverId id1("id1");
  const SolverId id2("id2");

  SolverOptions solver_options;
  solver_options.SetOption(id1, "key1", 1.2);
  solver_options.SetOption(id1, "key2", 1);
  solver_options.SetOption(id1, "key3", "foo");

  // First check a solver id not in solver_options.
  EXPECT_NO_THROW(solver_options.CheckOptionKeysForSolver(id2, {"key1"},
                                                          {"key2"}, {"key3"}));
  // Check the solver id in solver_options.
  EXPECT_NO_THROW(solver_options.CheckOptionKeysForSolver(id1, {"key1"},
                                                          {"key2"}, {"key3"}));

  // Check an option not set for id1.
  solver_options.SetOption(id1, "key2", 1.3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.CheckOptionKeysForSolver(id1, {"key1"}, {"key2"},
                                              {"key3"}),
      std::invalid_argument,
      "key2 is not allowed in the SolverOptions for id1.");

  EXPECT_NO_THROW(solver_options.CheckOptionKeysForSolver(id1, {"key1", "key2"},
                                                          {"key2"}, {"key3"}));
}
}  // namespace solvers
}  // namespace drake
