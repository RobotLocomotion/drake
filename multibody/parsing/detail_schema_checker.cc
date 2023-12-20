#include "drake/multibody/parsing/detail_schema_checker.h"

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <regex>
#include <string>
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
#include "drake/common/never_destroyed.h"
#include "drake/common/scope_exit.h"
#include "drake/common/ssize.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

namespace {

// Use this singleton mutex to protect both the one global variable below, and
// the static global state within the rnv library.
static std::mutex& library_mutex() {
  static drake::never_destroyed<std::mutex> mutex;
  return mutex.access();
}

// This helper allows writing assertions about the mutex state.
bool library_mutex_is_locked() {
  auto& mutex = library_mutex();
  if (mutex.try_lock()) {
    mutex.unlock();
    return false;
  }
  return true;
}

// Capture and redirect error messages from the rnv library to a diagnostic
// policy object. Note that this requires manipulation of global state
// (function and instance pointers) both in this class and in the rnv library.
//
// This class does not fully enforce thread safety itself, but requires that
// any client be holding the library mutex at construction time, and assumes
// that it will be held for the lifetime of any instance.
class DiagnosticAdapter {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagnosticAdapter)
  explicit DiagnosticAdapter(const DiagnosticPolicy* policy)
  : policy_(policy) {
    DRAKE_DEMAND(library_mutex_is_locked());
    DRAKE_ASSERT(the_adapter == nullptr);
    the_adapter = this;
    old_er_vprintf_ = ::er_vprintf;
    ::er_vprintf = DiagnosticAdapter::er_vprintf;
  }

  ~DiagnosticAdapter() {
    ::er_vprintf = old_er_vprintf_;
    the_adapter = nullptr;
  }

  // This could have been the destructor, except for the fact that Error() most
  // commonly throws exceptions.
  void Finish() {
    flush();
    for (const auto& event : diagnostic_events_) {
      switch (event.mode) {
        case Unknown: { break; }
        case Error: { policy_->Error(event.detail); break; }
        case Warning: { policy_->Warning(event.detail); break; }
      }
    }
  }

  int error_count() const { return error_count_; }

 private:
  static int er_vprintf(char* format, va_list ap) {
    return the_adapter->ErVprintf(format, ap);
  }

  int ErVprintf(char* format, va_list ap) noexcept {
    std::array<char, 4096> buffer{};
    vsnprintf(buffer.data(), buffer.size(), format, ap);
    buffer[buffer.size() - 1] = 0;
    std::string result(buffer.data());

    // Since the library issues informative multiline comments, parse them here
    // into multiline Error() or Warning() events.
    std::smatch m;
    static constexpr char kPattern[] =
        R"""(^([^:]*):(\d+):(\d+:)? (warning|error): (.*))""";
    std::regex_search(result, m, std::regex(kPattern));
    if (!m.empty()) {
      flush();
      // set mode, filename, line.
      switch (*m[4].first) {
        case 'w': { mode_ = Warning; break; }
        case 'e': { mode_ = Error; ++error_count_; break; }
        default: { mode_ = Unknown; break; }
      }
      filename_ = m[1];
      line_ = std::stoi(m[2]);
      result = m[5];
    }
    // store.
    buffer_.push_back(result);
    return result.size();
  }

  void flush() {
    DiagnosticDetail detail;
    detail.filename = filename_;
    detail.line = line_;
    for (int k = 0; k < ssize(buffer_); ++k) {
      detail.message += buffer_[k];
    }
    // We can't throw any exceptions while the rnv-implemented parse is
    // running, so buffer up the completed events and emit them later.
    switch (mode_) {
      case Unknown: { break; }
      case Error: { diagnostic_events_.push_back({Error, detail}); break; }
      case Warning: { diagnostic_events_.push_back({Warning, detail}); break; }
    }
    buffer_.clear();
  }

  // This is global state containing and instance pointer, which allows us to
  // crawl back from a patched global function pointer to the relevant live
  // instance of this class.
  static DiagnosticAdapter* the_adapter;

  const DiagnosticPolicy* policy_;
  typedef int (*ErVprintfFunc)(char *format, va_list ap);
  ErVprintfFunc old_er_vprintf_{};

  enum Mode { Unknown, Error, Warning } mode_ {Unknown};
  std::string filename_;
  int line_{};
  int error_count_{0};
  std::vector<std::string> buffer_;
  struct Event {
    Mode mode;
    DiagnosticDetail detail;
  };
  std::vector<Event> diagnostic_events_;
};
DiagnosticAdapter* DiagnosticAdapter::the_adapter{nullptr};

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
  explicit RnvGuard(const DiagnosticPolicy* policy)
  : adapter_(policy) {
    xcl_init();
  }
  ~RnvGuard() {
    xcl_clear();
  }

  int error_count() const {
    return adapter_.error_count();
  }

  void Finish() {
    adapter_.Finish();
  }

 private:
  // This guard holds the mutex to protect both the DiagnosticAdapter, and the
  // underlying global state in the rnv library.
  std::lock_guard<std::mutex> guard_{library_mutex()};
  DiagnosticAdapter adapter_;
};

// Manage a file read descriptor in RAII style, with error reporting via
// diagnostic policy.
class ReadFd {
 public:
  explicit ReadFd(const DiagnosticPolicy& diagnostic,
    const std::string& filename) {
    fd_ = open(filename.c_str(), O_RDONLY);
    if (!ok()) {
        diagnostic.Error(fmt::format("Failed to read file '{}': {}",
                                     filename, strerror(errno)));
    }
  }
  ~ReadFd() {
    if (ok()) { close(fd_); }
  }
  bool ok() const { return fd_ >= 0; }
  operator int () { return fd_; }
 private:
  int fd_{};
};

}  // namespace

bool CheckDocumentFileAgainstRncSchemaFile(
    const DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document) {
  RnvGuard rnv_guard(&diagnostic);
  DRAKE_ASSERT(library_mutex_is_locked());
  ReadFd schema_fd(diagnostic, rnc_schema);
  if (!schema_fd.ok()) { return false; }
  xcl_rnl_fd(const_cast<char*>(rnc_schema.c_str()), schema_fd);
  if (rnv_guard.error_count() > 0) {
    rnv_guard.Finish();
    return false;
  }
  ReadFd fd(diagnostic, document);
  if (!fd.ok()) { return false; }
  xcl_validate_fd(fd, document.c_str());
  rnv_guard.Finish();
  return (rnv_guard.error_count() == 0);
}

bool CheckDocumentStringAgainstRncSchemaFile(
    const DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::string& document_contents,
    const std::string& document_filename) {
  RnvGuard rnv_guard(&diagnostic);
  DRAKE_ASSERT(library_mutex_is_locked());
  ReadFd schema_fd(diagnostic, rnc_schema);
  if (!schema_fd.ok()) { return false; }
  xcl_rnl_fd(const_cast<char*>(rnc_schema.c_str()), schema_fd);
  if (rnv_guard.error_count() > 0) {
    rnv_guard.Finish();
    return false;
  }
  xcl_validate_memory(const_cast<char*>(document_contents.c_str()),
                      document_contents.size(),
                      document_filename.c_str());
  rnv_guard.Finish();
  return (rnv_guard.error_count() == 0);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
