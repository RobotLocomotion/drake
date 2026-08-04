#include <iostream>
#include <string_view>

#include <benchmark/benchmark.h>
#include <gflags/gflags.h>

#include "drake/tools/performance/fixture_memory.h"

DEFINE_bool(memory_manager, true, "Enable memory reporting");

int main(int argc, char** argv) {
  benchmark::MaybeReenterWithoutASLR(argc, argv);
  gflags::SetUsageMessage("see drake/tools/performance/README.md");
  for (int i = 1; i < argc; ++i) {
    if (std::string_view(argv[i]) == "--help") {
      gflags::ShowUsageWithFlags(argv[0]);
      std::cout << "\n\n";
      std::cout << "Flags from @google_benchmark:\n";
      benchmark::Initialize(&argc, argv);
      return 0;
    }
  }
  benchmark::Initialize(&argc, argv);
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  if (::benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
  if (FLAGS_memory_manager) {
    drake::tools::performance::EnableMemoryManager();
  }
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  return 0;
}
