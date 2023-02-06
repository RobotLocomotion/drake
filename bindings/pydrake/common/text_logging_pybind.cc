#ifdef HAVE_SPDLOG
#include "drake/bindings/pydrake/common/text_logging_pybind.h"

#include <atomic>
#include <memory>

// clang-format off to disable clang-format-includes
// N.B. text-logging.h must be included before spdlog headers
// to avoid "SPDLOG_ACTIVE_LEVEL" redefined warning (#13771).
#include "drake/common/text_logging.h"
// clang-format on

#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_sinks.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_assert.h"
#endif

namespace drake {
namespace pydrake {
namespace internal {

#ifdef HAVE_SPDLOG
namespace {
class pylogging_sink final
    // We use null_mutex below because we'll use the GIL as our *only* mutex.
    // This is critically important to avoid deadlocks by lock order inversion.
    : public spdlog::sinks::base_sink<spdlog::details::null_mutex> {
 public:
  pylogging_sink() {
    // Add a Python logging.Logger to be used by Drake.
    py::object logging = py::module::import("logging");
    py::object logger = logging.attr("getLogger")(name_);

    // Annotate that the logger is alive (fed by spdlog).
    py::setattr(logger, "_tied_to_spdlog", py::cast(true));

    // Match the C++ default level.
    logger.attr("setLevel")(to_py_level(drake::log()->level()));

    // Memoize the member functions we need to call.
    is_enabled_for_ = logger.attr("isEnabledFor");
    make_record_ = logger.attr("makeRecord");
    handle_ = logger.attr("handle");
  }

 protected:
  void sink_it_(const spdlog::details::log_msg& msg) final {
    py::gil_scoped_acquire acquire;

    // Bail out quickly in case this log level is disabled.
    const int level = to_py_level(msg.level);
    if (!is_enabled_for_(level).cast<bool>()) {
      return;
    }

    // Ensure that basicConfig happens at least once prior to posting and log
    // message. It's safe to call basicConfig more than once.
    if (!is_configured_.load()) {
      py::module::import("logging").attr("basicConfig")();
      is_configured_.store(true);
    }

    // NOLINTNEXTLINE(build/namespaces) This is how pybind11 wants it.
    using namespace pybind11::literals;

    // Construct the LogRecord.
    // https://docs.python.org/3/library/logging.html#logrecord-objects
    py::object record = make_record_(                                   // BR
        "name"_a = name_,                                               // BR
        "level"_a = level,                                              // BR
        "fn"_a = msg.source.filename,                                   // BR
        "lno"_a = msg.source.line,                                      // BR
        "func"_a = msg.source.funcname,                                 // BR
        "msg"_a = std::string(msg.payload.begin(), msg.payload.end()),  // BR
        "args"_a = py::list(),                                          // BR
        "exc_info"_a = py::none());
    py::setattr(record, "thread", py::cast(msg.thread_id));
    // We don't pass msg.time along into the record time because we'd need to
    // fix up record.created, record.msecs, and record.relativeCreated, and we'd
    // still be brittle with respect to future changes to the record class's
    // internal bookkeeping. Instead, we'll allow the record's own time from its
    // constructor to survive.

    // Publish the log record.
    handle_(record);
  }

  void flush_() final {}

 private:
  // https://docs.python.org/3/library/logging.html#logging-levels
  static int to_py_level(spdlog::level::level_enum level) {
    using Enum = spdlog::level::level_enum;
    switch (level) {
      case Enum::trace:
        return 5;
      case Enum::debug:
        return 10;
      case Enum::info:
        return 20;
      case Enum::warn:
        return 30;
      case Enum::err:
        return 40;
      case Enum::critical:
        return 50;
      case Enum::off:
        break;
#if SPDLOG_VERSION >= 10600
      case Enum::n_levels:
        break;
#endif
    }
    DRAKE_UNREACHABLE();
  }

  std::atomic<bool> is_configured_{false};
  py::object name_{py::cast("drake")};
  py::object is_enabled_for_;
  py::object make_record_;
  py::object handle_;
};
}  // namespace

void MaybeRedirectPythonLogging() {
  // Inspect the spdlog configuration to check that it exactly matches how
  // drake/common/text_logging.cc configures itself by default. If the spdlog
  // configuration we observe here differs in any way, then we'll assume that
  // a user has configured it to their taste already and we won't change it.
  std::vector<std::shared_ptr<spdlog::sinks::sink> >& root_sinks =
      drake::log()->sinks();
  if (root_sinks.size() != 1) {
    drake::log()->debug(
        "Will not redirect C++ logging to Python (num root sinks != 1)");
    return;
  }
  spdlog::sinks::sink* const root_sink = root_sinks.front().get();
  auto* dist_sink = dynamic_cast<spdlog::sinks::dist_sink_mt*>(root_sink);
  if (dist_sink == nullptr) {
    drake::log()->debug(
        "Will not redirect C++ logging to Python (wrong root sink)");
    return;
  }
  const std::vector<std::shared_ptr<spdlog::sinks::sink> >& dist_sinks =
      dist_sink->sinks();
  if (dist_sinks.size() != 1) {
    drake::log()->debug(
        "Will not redirect C++ logging to Python (num sinks != 1)");
    return;
  }
  spdlog::sinks::sink* const only_sink = dist_sinks.front().get();
  if (dynamic_cast<spdlog::sinks::stderr_sink_mt*>(only_sink) == nullptr) {
    drake::log()->debug(
        "Will not redirect C++ logging to Python (not a stderr sink)");
    return;
  }
  // If we reach this point, then we've matched text_logging.cc exactly.

  // Replace the stderr sink with a python sink.
  spdlog::sink_ptr& old_sink = root_sinks.at(0);
  spdlog::sink_ptr new_sink = std::make_shared<pylogging_sink>();
  std::atomic_exchange(&old_sink, std::move(new_sink));
  drake::log()->trace("Successfully redirected C++ logs to Python");
}
#else
void MaybeRedirectPythonLogging() {}
#endif

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
