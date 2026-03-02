#pragma once

/** @file
This file contains functions that are only available when spdlog is enabled in
Drake's build flags. */

#include <spdlog/sinks/dist_sink.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"

namespace drake {
namespace logging {

using sink DRAKE_DEPRECATED("2026-06-01", "Use spdlog::sinks::sink instead.") =
    spdlog::sinks::sink;

/** (Advanced) Retrieves the default sink for all Drake logs. This allows
consumers of Drake to redirect Drake's text logs to locations other than the
default of stderr. */
spdlog::sinks::dist_sink_mt* get_dist_sink();

}  // namespace logging
}  // namespace drake
