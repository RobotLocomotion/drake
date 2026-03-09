#include "drake/solvers/solver_options.h"

#include <limits>
#include <string>
#include <unordered_set>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace solvers {

using testing::Pair;
using testing::UnorderedElementsAre;

GTEST_TEST(OptionValueTest, ToString) {
  using OptionValue = SolverOptions::OptionValue;
  const OptionValue value_double = 22.0;
  const OptionValue value_int = 10;
  const OptionValue value_string = "hello";
  EXPECT_EQ(internal::OptionValueToString(value_double), "22.0");
  EXPECT_EQ(internal::OptionValueToString(value_int), "10");
  EXPECT_EQ(internal::OptionValueToString(value_string), "\"hello\"");

  const OptionValue value_nan = std::numeric_limits<double>::quiet_NaN();
  const OptionValue value_inf = std::numeric_limits<double>::infinity();
  const OptionValue value_neg_inf = -std::numeric_limits<double>::infinity();
  EXPECT_EQ(internal::OptionValueToString(value_nan), "nan");
  EXPECT_EQ(internal::OptionValueToString(value_inf), "inf");
  EXPECT_EQ(internal::OptionValueToString(value_neg_inf), "-inf");

  const OptionValue value_needs_escaping = "hello, \n\"world\"";
  EXPECT_EQ(internal::OptionValueToString(value_needs_escaping),
            fmt::format("{dq}hello, {bs}n{bs}{dq}world{bs}{dq}{dq}",
                        fmt::arg("bs", '\\'), fmt::arg("dq", '"')));
}

GTEST_TEST(SolverOptionsTest, CommonToString) {
  const CommonSolverOption dut = CommonSolverOption::kPrintFileName;
  EXPECT_EQ(to_string(dut), "kPrintFileName");
  EXPECT_EQ(fmt::to_string(dut), "kPrintFileName");
}

GTEST_TEST(SolverOptionsTest, Comparison) {
  SolverOptions foo;
  SolverOptions bar;
  EXPECT_TRUE(foo == bar);
  EXPECT_TRUE(bar == foo);
  EXPECT_FALSE(foo != bar);
  EXPECT_FALSE(bar != foo);

  foo.SetOption(CommonSolverOption::kPrintToConsole, 1);
  EXPECT_FALSE(foo == bar);
  EXPECT_FALSE(bar == foo);
  EXPECT_TRUE(foo != bar);
  EXPECT_TRUE(bar != foo);

  bar.SetOption(CommonSolverOption::kPrintToConsole, 1);
  EXPECT_TRUE(foo == bar);
  EXPECT_TRUE(bar == foo);
  EXPECT_FALSE(foo != bar);
  EXPECT_FALSE(bar != foo);
}

GTEST_TEST(SolverOptionsTest, SetGetOption) {
  SolverOptions dut;
  EXPECT_TRUE(dut.options.empty());
  EXPECT_EQ(dut.to_string(), "SolverOptions(options={})");

  const SolverId id1("id1");
  const SolverId id2("id2");
  dut.SetOption(id1, "some_double", 1.1);
  dut.SetOption(id1, "some_int", 2);
  dut.SetOption(id2, "some_string", "foo");
  dut.SetOption(CommonSolverOption::kPrintFileName, "foo.txt");
  dut.SetOption(CommonSolverOption::kPrintToConsole, 1);
  dut.SetOption(CommonSolverOption::kStandaloneReproductionFileName, "bar.py");
  dut.SetOption(CommonSolverOption::kMaxThreads, 2);

  EXPECT_THAT(
      dut.options,
      UnorderedElementsAre(
          Pair("id1", UnorderedElementsAre(Pair("some_double", 1.1),
                                           Pair("some_int", 2))),
          Pair("id2", UnorderedElementsAre(Pair("some_string", "foo"))),
          Pair("Drake", UnorderedElementsAre(
                            Pair("kPrintFileName", "foo.txt"),
                            Pair("kPrintToConsole", 1),
                            Pair("kStandaloneReproductionFileName", "bar.py"),
                            Pair("kMaxThreads", 2)))));
  EXPECT_EQ(dut.to_string(),
            "SolverOptions(options={"
            "\"Drake\":{"
            "\"kMaxThreads\":2,"
            "\"kPrintFileName\":\"foo.txt\","
            "\"kPrintToConsole\":1,"
            "\"kStandaloneReproductionFileName\":\"bar.py\""
            "},"
            "\"id1\":{"
            "\"some_double\":1.1,"
            "\"some_int\":2"
            "},"
            "\"id2\":{"
            "\"some_string\":\"foo\""
            "}"
            "})");
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

GTEST_TEST(SolverOptionsTest, SetOptionError) {
  SolverOptions solver_options;
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.SetOption(CommonSolverOption::kPrintFileName, 1),
      ".*SetOption.*kPrintFileName.*must be a string, not 1.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.SetOption(CommonSolverOption::kPrintToConsole, 2),
      ".*SetOption.*kPrintToConsole.*must be 0 or 1, not 2.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.SetOption(CommonSolverOption::kMaxThreads, 2.1),
      ".*SetOption.*kMaxThreads.*must be an int > 0, not 2.1.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.SetOption(CommonSolverOption::kMaxThreads, -1),
      ".*SetOption.*kMaxThreads.*must be an int > 0, not -1.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_options.SetOption(
          CommonSolverOption::kStandaloneReproductionFileName, 1),
      ".*SetOption.*kStandalone.*must be a string, not 1.");
}

GTEST_TEST(SolverOptionsTest, Serialization) {
  SolverOptions dut;
  const SolverId id1("id1");
  const SolverId id2("id2");
  dut.SetOption(id1, "some_double", 1.1);
  dut.SetOption(id1, "some_int", 2);
  dut.SetOption(id2, "some_string", "foo");
  dut.SetOption(CommonSolverOption::kPrintFileName, "foo.txt");
  dut.SetOption(CommonSolverOption::kPrintToConsole, 1);
  dut.SetOption(CommonSolverOption::kStandaloneReproductionFileName, "bar.py");
  dut.SetOption(CommonSolverOption::kMaxThreads, 2);

  // If you change either of these two string constants, then you must
  // make the same change to the Python mathematicalprogram_test.py.
  const std::string cxx_expected = R"""(options:
  Drake:
    kMaxThreads: !!int 2
    kPrintFileName: !!str foo.txt
    kPrintToConsole: !!int 1
    kStandaloneReproductionFileName: !!str bar.py
  id1:
    some_double: 1.1
    some_int: !!int 2
  id2:
    some_string: !!str foo
)""";
  const std::string py_expected = R"""(options:
  Drake:
    kMaxThreads: !!int '2'
    kPrintFileName: !!str 'foo.txt'
    kPrintToConsole: !!int '1'
    kStandaloneReproductionFileName: !!str 'bar.py'
  id1:
    some_double: 1.1
    some_int: !!int '2'
  id2:
    some_string: !!str 'foo'
)""";

  // Check that C++ can save and then re-load the options.
  const std::string actual_written = yaml::SaveYamlString(dut);
  EXPECT_EQ(actual_written, cxx_expected);
  SolverOptions readback;
  EXPECT_NO_THROW(readback =
                      yaml::LoadYamlString<SolverOptions>(actual_written));
  EXPECT_EQ(readback, dut);

  // Cross-check that the output written by the Python unit test can be
  // re-loaded in C++.
  SolverOptions py_readback;
  EXPECT_NO_THROW(py_readback =
                      yaml::LoadYamlString<SolverOptions>(py_expected));
  EXPECT_EQ(py_readback, dut);
}

}  // namespace solvers
}  // namespace drake
