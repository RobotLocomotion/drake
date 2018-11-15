#include "drake/solvers/solver_options.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
class SolverOptionsTester {
 public:
  explicit SolverOptionsTester(const SolverOptions& solver_options)
      : solver_options_{solver_options} {}

  const std::unordered_map<SolverId, std::unordered_map<std::string, double>>&
  solver_options_double() const {
    return solver_options_.solver_options_double_;
  }

  const std::unordered_map<SolverId, std::unordered_map<std::string, int>>&
  solver_options_int() const {
    return solver_options_.solver_options_int_;
  }

  const std::unordered_map<
    SolverId, std::unordered_map<std::string, std::string>>&
  solver_options_str() const {
    return solver_options_.solver_options_str_;
  }

 private:
  SolverOptions solver_options_;
};

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

template <typename T>
void CheckSolverOptionsHelper(
    const std::unordered_map<SolverId, std::unordered_map<std::string, T>>&
        options,
    const std::unordered_map<SolverId, std::unordered_map<std::string, T>>&
        options_expected) {
  EXPECT_EQ(options.size(), options_expected.size());
  for (const auto& options_pair : options) {
    auto it = options_expected.find(options_pair.first);
    if (it == options_expected.end()) {
      std::cout << "Solver " << options_pair.first.name() << " not found.\n";
    }
    EXPECT_NE(it, options_expected.end());
    for (const auto& key_value : options_pair.second) {
      auto key_value_expected_it = it->second.find(key_value.first);
      if (key_value_expected_it == it->second.end()) {
        std::cout << "Solver " << options_pair.first.name() << " option "
                  << key_value.first << " value " << key_value.second
                  << " not found.\n";
      }
      EXPECT_NE(key_value_expected_it, it->second.end());
      EXPECT_EQ(key_value.second, key_value_expected_it->second);
    }
  }
}

void CheckSolverOptions(const SolverOptionsTester& options,
                        const SolverOptionsTester& options_expected) {
  CheckSolverOptionsHelper(options.solver_options_double(),
                           options_expected.solver_options_double());
  CheckSolverOptionsHelper(options_expected.solver_options_double(),
                           options.solver_options_double());
  CheckSolverOptionsHelper(options.solver_options_int(),
                           options_expected.solver_options_int());
  CheckSolverOptionsHelper(options_expected.solver_options_int(),
                           options.solver_options_int());
  CheckSolverOptionsHelper(options.solver_options_str(),
                           options_expected.solver_options_str());
  CheckSolverOptionsHelper(options_expected.solver_options_str(),
                           options.solver_options_str());
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
  CheckSolverOptions(SolverOptionsTester(merge_option1_into_option2),
                     SolverOptionsTester(solver_options2));
  CheckSolverOptions(SolverOptionsTester(merge_option2_into_option1),
                     SolverOptionsTester(solver_options1));

  // Now solver_options2 contains a key for solver foo, that is not contained in
  // solver_options1.
  solver_options2.SetOption(id1, "key2", 1);
  UpdateMergeResult(solver_options1, solver_options2,
                    &merge_option1_into_option2, &merge_option2_into_option1);
  CheckSolverOptions(SolverOptionsTester(merge_option1_into_option2),
                     SolverOptionsTester(solver_options2));
  SolverOptions merge_option2_into_option1_expected = solver_options1;
  merge_option2_into_option1_expected.SetOption(id1, "key2", 1);
  CheckSolverOptions(SolverOptionsTester(merge_option2_into_option1),
                     SolverOptionsTester(merge_option2_into_option1_expected));

  // Now solver_options1 contains a different solver id
  solver_options1.SetOption(id2, "key1", 0.1);
  UpdateMergeResult(solver_options1, solver_options2,
                    &merge_option1_into_option2, &merge_option2_into_option1);
  merge_option2_into_option1_expected.SetOption(id2, "key1", 0.1);
  CheckSolverOptions(SolverOptionsTester(merge_option2_into_option1),
                     SolverOptionsTester(merge_option2_into_option1_expected));
  SolverOptions merge_option1_into_option2_expected = solver_options2;
  merge_option1_into_option2_expected.SetOption(id2, "key1", 0.1);
  CheckSolverOptions(SolverOptionsTester(merge_option1_into_option2),
                     SolverOptionsTester(merge_option1_into_option2_expected));
}
}  // namespace solvers
}  // namespace drake
