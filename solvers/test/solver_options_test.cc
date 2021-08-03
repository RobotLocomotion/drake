#include "drake/solvers/solver_options.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
GTEST_TEST(SolverOptionsTest, SetGetOption) {
  SolverOptions dut;
  EXPECT_EQ(to_string(dut), "{SolverOptions empty}");
  EXPECT_EQ(dut.get_print_file_name(), "");
  EXPECT_EQ(dut.get_print_to_console(), false);

  const SolverId id1("id1");
  const SolverId id2("id2");

  dut.SetOption(id1, "some_double", 1.1);
  dut.SetOption(id1, "some_before", 1.2);
  dut.SetOption(id1, "some_int", 2);

  dut.SetOption(id2, "some_int", "3");
  dut.SetOption(id2, "some_string", "foo");

  dut.SetOption(CommonSolverOption::kPrintFileName, "foo.txt");
  dut.SetOption(CommonSolverOption::kPrintToConsole, 1);

  EXPECT_EQ(to_string(dut),
            "{SolverOptions,"
            " CommonSolverOption::kPrintFileName=foo.txt,"
            " CommonSolverOption::kPrintToConsole=1,"
            " id1:some_before=1.2,"
            " id1:some_double=1.1,"
            " id1:some_int=2,"
            " id2:some_int=3,"
            " id2:some_string=foo}");
  EXPECT_EQ(dut.get_print_file_name(), "foo.txt");
  EXPECT_EQ(dut.get_print_to_console(), true);

  const std::unordered_map<CommonSolverOption,
                           std::variant<double, int, std::string>>
      common_options_expected(
          {{CommonSolverOption::kPrintToConsole, 0},
           {CommonSolverOption::kPrintFileName, "foo.txt"}});
  // TODO(hongkai.dai): Test GetOption<double>() and `GetOptionDouble()` when
  // a CommonSolverOption takes a double value.
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

  // foo contains a non-empty drake_solver_option map
  foo.SetOption(CommonSolverOption::kPrintFileName, "bar.txt");
  dut.Merge(foo);
  dut_expected.SetOption(CommonSolverOption::kPrintFileName, "bar.txt");
  EXPECT_EQ(dut, dut_expected);

  // Duplicate drake_solver_option map, no-op.
  foo.SetOption(CommonSolverOption::kPrintFileName, "bar_new.txt");
  dut.Merge(foo);
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
  DRAKE_EXPECT_NO_THROW(solver_options.CheckOptionKeysForSolver(
      id2, {"key1"}, {"key2"}, {"key3"}));
  // Check the solver id in solver_options.
  DRAKE_EXPECT_NO_THROW(solver_options.CheckOptionKeysForSolver(id1, {"key1"},
                                                          {"key2"}, {"key3"}));

  // Check an option not set for id1.
  solver_options.SetOption(id1, "key2", 1.3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.CheckOptionKeysForSolver(id1, {"key1"}, {"key2"},
                                              {"key3"}),
      std::invalid_argument,
      "key2 is not allowed in the SolverOptions for id1.");

  DRAKE_EXPECT_NO_THROW(solver_options.CheckOptionKeysForSolver(
      id1, {"key1", "key2"}, {"key2"}, {"key3"}));
}

GTEST_TEST(SolverOptionsTest, SetOptionError) {
  SolverOptions solver_options;
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.SetOption(CommonSolverOption::kPrintFileName, 1),
      std::runtime_error,
      "SolverOptions::SetOption support kPrintFileName only with std::string "
      "value.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.SetOption(CommonSolverOption::kPrintToConsole, 2),
      std::runtime_error, "kPrintToConsole expects value either 0 or 1");
}
}  // namespace solvers
}  // namespace drake
