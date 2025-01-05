#include "drake/solvers/specific_options.h"

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/never_destroyed.h"
#include "drake/common/string_map.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {

using testing::UnorderedElementsAre;

const SolverId& GetSolverId() {
  static const never_destroyed<SolverId> result("test_id");
  return result.access();
}

GTEST_TEST(SpecificOptionsTest, NullChecks) {
  const SolverId& solver_id = GetSolverId();
  SolverOptions options;
  EXPECT_NO_THROW(SpecificOptions(&solver_id, &options));
  DRAKE_EXPECT_THROWS_MESSAGE(SpecificOptions(nullptr, &options), ".*null.*");
  DRAKE_EXPECT_THROWS_MESSAGE(SpecificOptions(&solver_id, nullptr), ".*null.*");
}

GTEST_TEST(SpecificOptionsTest, BasicPop) {
  const SolverId& solver_id = GetSolverId();
  SolverOptions solver_options;
  solver_options.SetOption(solver_id, "some_default", 1234.5);
  SpecificOptions dut(&solver_id, &solver_options);

  // Popping with the wrong type is a no-op.
  EXPECT_EQ(dut.Pop<std::string>("some_default"), std::nullopt);

  // Popping with the correct type retrieves it.
  EXPECT_EQ(dut.Pop<double>("some_default"), 1234.5);

  // It's gone now.
  EXPECT_EQ(dut.Pop<double>("some_default"), std::nullopt);
  EXPECT_NO_THROW(dut.CopyToCallbacks({}, {}, {}));
}

GTEST_TEST(SpecificOptionsTest, RespellThenPop) {
  const SolverId& solver_id = GetSolverId();
  const SolverOptions solver_options;
  SpecificOptions dut(&solver_id, &solver_options);

  // Add an option via respelling.
  dut.Respell([](const CommonSolverOptionValues& common,
                 string_unordered_map<SolverOptions::OptionValue>* respelled) {
    ASSERT_TRUE(respelled != nullptr);
    respelled->emplace("some_default", 1234.5);
  });

  // Popping with the correct type retrieves it.
  EXPECT_EQ(dut.Pop<double>("some_default"), 1234.5);

  // It's gone now.
  EXPECT_EQ(dut.Pop<double>("some_default"), std::nullopt);
  EXPECT_NO_THROW(dut.CopyToCallbacks({}, {}, {}));
}

GTEST_TEST(SpecificOptionsTest, RespellThenPopWrongType) {
  const SolverId& solver_id = GetSolverId();
  const SolverOptions solver_options;
  SpecificOptions dut(&solver_id, &solver_options);

  // Add an option via respelling.
  dut.Respell([](const CommonSolverOptionValues& common,
                 string_unordered_map<SolverOptions::OptionValue>* respelled) {
    ASSERT_TRUE(respelled != nullptr);
    respelled->emplace("some_default", 1234.5);
  });

  // The solver back-end is not allowed to mismatch its own types.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Pop<int>("some_default"),
                              ".*respelled.*wrong type.*");
}

GTEST_TEST(SpecificOptionsTest, CopyToCallbacksIntPromotesToDouble) {
  const SolverId& solver_id = GetSolverId();
  SolverOptions solver_options;
  solver_options.SetOption(solver_id, "my_double", 3);
  SpecificOptions dut(&solver_id, &solver_options);
  auto fail = [](const auto&...) {
    GTEST_FAIL();
  };
  string_map<double> values_double;
  dut.CopyToCallbacks(
      /* set_double = */
      [&values_double](const std::string& key, double value) {
        values_double.emplace(key, value);
      },
      /* set_int = */ nullptr, /* set_string = */ fail);
  EXPECT_THAT(values_double, UnorderedElementsAre(std::pair{"my_double", 3.0}));
}

GTEST_TEST(SpecificOptionsTest, CopyToCallbacksNoDouble) {
  const SolverId& solver_id = GetSolverId();
  SolverOptions solver_options;
  solver_options.SetOption(solver_id, "my_double", 1.5);
  SpecificOptions dut(&solver_id, &solver_options);
  auto fail = [](const auto&...) {
    GTEST_FAIL();
  };
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.CopyToCallbacks(/* set_double = */ nullptr,
                          /* set_int = */ fail, /* set_string = */ fail),
      ".*floating-point.*not supported.*");
}

GTEST_TEST(SpecificOptionsTest, CopyToCallbacksNoDoubleNorInt) {
  const SolverId& solver_id = GetSolverId();
  SolverOptions solver_options;
  solver_options.SetOption(solver_id, "my_int", 3);
  SpecificOptions dut(&solver_id, &solver_options);
  auto fail = [](const auto&...) {
    GTEST_FAIL();
  };
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.CopyToCallbacks(/* set_double = */ nullptr, /* set_int = */ nullptr,
                          /* set_string = */ fail),
      ".*integer and floating.point.*not supported.*");
}

GTEST_TEST(SpecificOptionsTest, CopyToCallbacksString) {
  const SolverId& solver_id = GetSolverId();
  SolverOptions solver_options;
  solver_options.SetOption(solver_id, "my_string", "hello");
  SpecificOptions dut(&solver_id, &solver_options);
  auto fail = [](const auto&...) {
    GTEST_FAIL();
  };
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.CopyToCallbacks(/* set_double = */ fail, /* set_int = */ fail,
                          /* set_string = */ nullptr),
      ".*string.*not supported.*");
}

SolverOptions MakeFullFledgedSolverOptions() {
  const SolverId& id = GetSolverId();
  SolverOptions result;

  // These options should be processed.
  result.SetOption(id, "my_double", 1.5);
  result.SetOption(id, "my_int", 3);
  result.SetOption(id, "my_string", "hello");

  // These will be popped off before processing.
  result.SetOption(id, "double_to_pop", 0.5);
  result.SetOption(id, "int_to_pop", 2);
  result.SetOption(id, "string_to_pop", "popped");

  // Set all common options to non-default values.
  result.SetOption(CommonSolverOption::kPrintFileName, "print.log");
  result.SetOption(CommonSolverOption::kPrintToConsole, 1);
  result.SetOption(CommonSolverOption::kStandaloneReproductionFileName,
                   "repro.txt");
  result.SetOption(CommonSolverOption::kMaxThreads, 4);

  // These options are for a different solver, so will be ignored.
  const SolverId some_other_solver_id("ignored");
  result.SetOption(some_other_solver_id, "double_ignored", 0.0);
  result.SetOption(some_other_solver_id, "int_ignored", 0);
  result.SetOption(some_other_solver_id, "string_ignored", "");

  return result;
}

GTEST_TEST(SpecificOptionsTest, CopyToCallbacksTypicalWorkflow) {
  const SolverId& solver_id = GetSolverId();
  const SolverOptions solver_options = MakeFullFledgedSolverOptions();

  SpecificOptions dut(&solver_id, &solver_options);

  // Pop things which exist.
  EXPECT_EQ(dut.Pop<double>("double_to_pop"), 0.5);
  EXPECT_EQ(dut.Pop<int>("int_to_pop"), 2);
  EXPECT_EQ(dut.Pop<std::string>("string_to_pop"), "popped");

  // Now they are gone.
  EXPECT_EQ(dut.Pop<double>("double_to_pop"), std::nullopt);
  EXPECT_EQ(dut.Pop<int>("int_to_pop"), std::nullopt);
  EXPECT_EQ(dut.Pop<std::string>("string_to_pop"), std::nullopt);

  // Pop things which do not exist.
  EXPECT_EQ(dut.Pop<double>("no_such_double"), std::nullopt);
  EXPECT_EQ(dut.Pop<int>("no_such_int"), std::nullopt);
  EXPECT_EQ(dut.Pop<std::string>("no_such_string"), std::nullopt);

  // Respell.
  dut.Respell([](const CommonSolverOptionValues& common,
                 string_unordered_map<SolverOptions::OptionValue>* respelled) {
    ASSERT_TRUE(respelled != nullptr);
    respelled->emplace("print_file_name", common.print_file_name);
    respelled->emplace("print_to_console", common.print_to_console);
    respelled->emplace("standalone_reproduction_file_name",
                       common.standalone_reproduction_file_name);
    respelled->emplace("max_threads", common.max_threads.value_or(256));
    respelled->emplace("some_default", 1234.5);
  });

  // Extract the options via callbacks.
  string_map<double> values_double;
  string_map<int> values_int;
  string_map<std::string> values_string;
  dut.CopyToCallbacks(
      [&values_double](const std::string& key, double value) {
        values_double.emplace(key, value);
      },
      [&values_int](const std::string& key, int value) {
        values_int.emplace(key, value);
      },
      [&values_string](const std::string& key, const std::string& value) {
        values_string.emplace(key, value);
      });

  EXPECT_THAT(values_double,
              UnorderedElementsAre(std::pair{"my_double", 1.5},
                                   std::pair{"some_default", 1234.5}));
  EXPECT_THAT(values_int,  // BR
              UnorderedElementsAre(std::pair{"my_int", 3},
                                   std::pair{"print_to_console", 1},
                                   std::pair{"max_threads", 4}));
  EXPECT_THAT(values_string,
              UnorderedElementsAre(
                  std::pair{"my_string", "hello"},
                  std::pair{"print_file_name", "print.log"},
                  std::pair{"standalone_reproduction_file_name", "repro.txt"}));
}

struct OptionsStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(my_double));
    a->Visit(DRAKE_NVP(my_int));
    a->Visit(DRAKE_NVP(my_string));
    a->Visit(DRAKE_NVP(print_file_name));
    a->Visit(DRAKE_NVP(print_to_console));
    a->Visit(DRAKE_NVP(standalone_reproduction_file_name));
    a->Visit(DRAKE_NVP(max_threads));
    a->Visit(DRAKE_NVP(some_default));
    a->Visit(DRAKE_NVP(unchanged));
  }

  double my_double{};
  int my_int{};
  std::string my_string;
  std::string print_file_name;
  bool print_to_console{};
  std::string standalone_reproduction_file_name;
  uint32_t max_threads{};
  double some_default{};
  std::string unchanged{"original_value"};
};

// NOLINTNEXTLINE(runtime/references) to match Serialize concept.
void Serialize(internal::SpecificOptions* archive, OptionsStruct& options) {
  options.Serialize(archive);
}

GTEST_TEST(SpecificOptionsTest, CopyToSerializableStructTypicalWorkflow) {
  const SolverId& solver_id = GetSolverId();
  const SolverOptions solver_options = MakeFullFledgedSolverOptions();

  SpecificOptions dut(&solver_id, &solver_options);

  // Pop things which exist.
  EXPECT_EQ(dut.Pop<double>("double_to_pop"), 0.5);
  EXPECT_EQ(dut.Pop<int>("int_to_pop"), 2);
  EXPECT_EQ(dut.Pop<std::string>("string_to_pop"), "popped");

  // Now they are gone.
  EXPECT_EQ(dut.Pop<double>("double_to_pop"), std::nullopt);
  EXPECT_EQ(dut.Pop<int>("int_to_pop"), std::nullopt);
  EXPECT_EQ(dut.Pop<std::string>("string_to_pop"), std::nullopt);

  // Pop things which do not exist.
  EXPECT_EQ(dut.Pop<double>("no_such_double"), std::nullopt);
  EXPECT_EQ(dut.Pop<int>("no_such_int"), std::nullopt);
  EXPECT_EQ(dut.Pop<std::string>("no_such_string"), std::nullopt);

  // Respell.
  dut.Respell([](const CommonSolverOptionValues& common,
                 string_unordered_map<SolverOptions::OptionValue>* respelled) {
    ASSERT_TRUE(respelled != nullptr);
    respelled->emplace("print_file_name", common.print_file_name);
    respelled->emplace("print_to_console", common.print_to_console);
    respelled->emplace("standalone_reproduction_file_name",
                       common.standalone_reproduction_file_name);
    respelled->emplace("max_threads", common.max_threads.value_or(-1));
    respelled->emplace("some_default", 1234.5);
  });

  // Extract the options via serialization.
  OptionsStruct options_struct;
  dut.CopyToSerializableStruct(&options_struct);
  EXPECT_EQ(options_struct.my_double, 1.5);
  EXPECT_EQ(options_struct.some_default, 1234.5);
  EXPECT_EQ(options_struct.my_int, 3);
  EXPECT_EQ(options_struct.print_to_console, 1);
  EXPECT_EQ(options_struct.max_threads, 4);
  EXPECT_EQ(options_struct.my_string, "hello");
  EXPECT_EQ(options_struct.print_file_name, "print.log");
  EXPECT_EQ(options_struct.standalone_reproduction_file_name, "repro.txt");
}

template <typename T>
OptionsStruct CopyOneOptionToSerializableStruct(const std::string& key,
                                                const T& value) {
  const SolverId& solver_id = GetSolverId();
  SolverOptions solver_options;
  solver_options.SetOption(solver_id, key, value);
  SpecificOptions dut(&solver_id, &solver_options);
  OptionsStruct options_struct;
  dut.CopyToSerializableStruct(&options_struct);
  return options_struct;
}

GTEST_TEST(SpecificOptionsTest, StructIntValuePromotionToDoubleMemberField) {
  const OptionsStruct options_struct =
      CopyOneOptionToSerializableStruct("my_double", 2);
  EXPECT_EQ(options_struct.my_double, 2.0);
}

GTEST_TEST(SpecificOptionsTest, StructNoSuchField) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("problematic_double", 0.5),
      ".*not recognized.*problematic_double.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("problematic_int", 1),
      ".*not recognized.*problematic_int.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("problematic_string", "foo"),
      ".*not recognized.*problematic_string.*");
}

GTEST_TEST(SpecificOptionsTest, StructWrongType) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("my_double", "foo"),
      ".*floating.point.*my_double=\"foo\".*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("my_int", "foo"),
      ".*integer.*my_int=\"foo\".*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("my_string", 0.5),
      ".*string.*my_string=0.5.*");
}

GTEST_TEST(SpecificOptionsTest, StructBadBool) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("print_to_console", -1),
      ".*(0 or 1).*print_to_console=-1.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("print_to_console", 2),
      ".*(0 or 1).*print_to_console=2.*");
}

GTEST_TEST(SpecificOptionsTest, StructBadUint32) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      CopyOneOptionToSerializableStruct("max_threads", -1),
      ".*non-negative.*max_threads=-1.*");
}

}  // namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake
