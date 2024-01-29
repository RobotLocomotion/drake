#include "drake/multibody/parsing/detail_schema_checker.h"

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

extern "C" {
// Fix these by adjusting the internal build rules.
// NOLINTNEXTLINE(build/include)
#include "er.h"
// NOLINTNEXTLINE(build/include)
#include "xcl.h"
};

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

namespace {

// Reprocesses errors and warnings from the rnv library to a diagnostic policy.
bool ReplayDiagnostics(const DiagnosticPolicy& diagnostic,
                       const std::vector<std::string>& rnv_messages,
                       Strictness strictness) {
  bool success = true;

  // Allow adjustments of severity based on strictness.
  enum class Severity { Error, Warning } severity = Severity::Error;
  auto adjusted_severity = [&severity, &strictness]() {
    switch (strictness) {
      case Strictness::kLax:
        return Severity::Warning;
      case Strictness::kStrict:
        return Severity::Error;
      case Strictness::kNormal:
        return severity;
    }
    DRAKE_UNREACHABLE();
  };

  // Since the library issues informative multiline comments, group them here
  // into multiline Error() or Warning() events.
  static constexpr char kPattern[] =
      R"""(^([^:]*):(\d+):(\d+:)? (warning|error): (.*))""";
  std::regex regex(kPattern);

  // These hold the work-in-progress message.
  DiagnosticDetail detail;
  auto flush = [&diagnostic, &success, &detail, &adjusted_severity]() {
    if (!detail.message.empty()) {
      if (adjusted_severity() == Severity::Error) {
        diagnostic.Error(detail);
        success = false;
      } else {
        diagnostic.Warning(detail);
      }
    }
    detail = {};
  };

  // Loop over all of the "printed" messages.
  for (const std::string& rnv_message : rnv_messages) {
    std::smatch match;
    std::regex_search(rnv_message, match, regex);
    if (match.empty()) {
      detail.message += rnv_message;
    } else {
      flush();
      detail.filename = match[1];
      detail.line = std::stoi(match[2]);
      severity = (*match[4].first == 'w') ? Severity::Warning : Severity::Error;
      detail.message = match[5];
    }
  }

  // Be sure to get the last one.
  flush();

  return success;
}

// While `rnv` is a very nice tool to use from the command line, it is not well
// suited for use as a library. The `RnvGuard` takes care of several necessary
// adaptations.
//
// * static global state: hold the global library mutex
// * static global state: init and clear operations
// * diagnostic redirection
class RnvGuard {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RnvGuard)

  explicit RnvGuard(std::vector<std::string>* messages) : messages_(messages) {
    DRAKE_DEMAND(g_singleton == nullptr);
    g_singleton = this;
    ::er_vprintf = &RnvGuard::vprintf_callback;
    xcl_init();
  }

  ~RnvGuard() {
    xcl_clear();
    er_vprintf = nullptr;
    g_singleton = nullptr;
  }

 private:
  // This mutex ensures that at most one RnvGuard is ever created per process.
  static std::mutex& singleton_mutex() {
    static drake::never_destroyed<std::mutex> mutex;
    return mutex.access();
  }

  static int vprintf_callback(char* format, va_list ap) {
    DRAKE_DEMAND(g_singleton != nullptr);
    std::array<char, 4096> buffer{};
    ::vsnprintf(buffer.data(), buffer.size(), format, ap);
    buffer[buffer.size() - 1] = 0;
    std::string message(buffer.data());
    int result = message.size();
    g_singleton->messages_->push_back(std::move(message));
    return result;
  }

  std::lock_guard<std::mutex> guard_{singleton_mutex()};
  std::vector<std::string>* const messages_;
  static RnvGuard* g_singleton;
};

RnvGuard* RnvGuard::g_singleton = nullptr;

}  // namespace

bool CheckDocumentFileAgainstRncSchemaFile(
    const DiagnosticPolicy& diagnostic, const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document,
    Strictness strictness) {
  const std::optional<std::string> contents = ReadFile(document);
  if (!contents) {
    diagnostic.Error(
        fmt::format("Could not open file '{}'", document.string()));
  }
  return CheckDocumentStringAgainstRncSchemaFile(
      diagnostic, rnc_schema, *contents, document, strictness);
}

bool CheckDocumentStringAgainstRncSchemaFile(
    const DiagnosticPolicy& diagnostic, const std::filesystem::path& rnc_schema,
    const std::string& document_contents,
    const std::string& document_filename,
    Strictness strictness) {
  const std::string rnc_data = ReadFileOrThrow(rnc_schema);
  std::vector<std::string> rnv_messages;
  {
    RnvGuard rnv_guard(&rnv_messages);
    xcl_rnl_s(const_cast<char*>(rnc_schema.c_str()),
              const_cast<char*>(rnc_data.c_str()), rnc_data.size());
    if (!rnv_messages.empty()) {
      throw std::runtime_error(fmt::format("Errors parsing '{}':\n{}",
                                           rnc_schema.string(),
                                           fmt::join(rnv_messages, "")));
    }
    xcl_validate_memory(const_cast<char*>(document_contents.c_str()),
                        document_contents.size(), document_filename.c_str());
  }
  return ReplayDiagnostics(diagnostic, rnv_messages, strictness);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
