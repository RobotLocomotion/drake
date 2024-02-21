#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/parsing/detail_schema_checker.h"

namespace drake {
namespace multibody {
namespace {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[SCHEMA-RNC-FILE] [XML-DOCUMENT-FILE]\n"
                          "Run schema checker; print errors if any");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 3) {
    drake::log()->error("missing input filename");
    return 1;
  }

  // If running under `bazel run`, set working directory to where the user
  // invoked this program, not somewhere in runfiles.
  const char* workspace = ::getenv("BUILD_WORKSPACE_DIRECTORY");
  if (workspace) {
    const char* working = ::getenv("BUILD_WORKING_DIRECTORY");
    DRAKE_DEMAND(working != nullptr);
    std::filesystem::current_path(working);
  }

  // Hardcode a log pattern that gives us un-decorated error messages.
  // This defeats the `-spdlog_pattern` command line option; oh well.
  drake::logging::set_log_pattern("%v");

  drake::log()->info("checking {} against {}", argv[2], argv[1]);

  DiagnosticPolicy policy;

  try {
    std::filesystem::path schema(argv[1]);
    std::filesystem::path document(argv[2]);
    return !internal::CheckDocumentFileAgainstRncSchemaFile(
        policy, schema, document, internal::Strictness::kLax);
  } catch (const std::exception& e) {
    drake::log()->error(e.what());
    return EXIT_FAILURE;
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}
