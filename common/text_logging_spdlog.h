#pragma once

/** @file
This file contains functions that are only available when spdlog is enabled in
Drake's build flags. */

#ifdef HAVE_SPDLOG
#include <spdlog/spdlog.h>
#endif  // HAVE_SPDLOG

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"

namespace drake {
namespace logging {

#ifdef HAVE_SPDLOG

/** When spdlog is enabled in this build, drake::logging::sink is an alias for
spdlog::sinks::sink. When spdlog is disabled, it is an empty class. */
using spdlog::sinks::sink;

/** (Advanced) Retrieves the default sink for all Drake logs. When spdlog is
enabled, the return value can be cast to spdlog::sinks::dist_sink_mt and thus
allows consumers of Drake to redirect Drake's text logs to locations other than
the default of stderr. When spdlog is disabled, the return value is an empty
class. */
sink* get_dist_sink();

#else  // HAVE_SPDLOG

/* A stubbed-out version of `spdlog::sinks::sink`. */
class sink {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(sink);
  sink();
};

DRAKE_DEPRECATED("2026-03-01",
                 "When spdlog is disabled, this header file may not be used.")
sink* get_dist_sink();

#endif  // HAVE_SPDLOG

}  // namespace logging
}  // namespace drake
