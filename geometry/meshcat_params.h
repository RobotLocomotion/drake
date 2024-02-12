#pragma once

#include <optional>
#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/** The set of parameters for configuring Meshcat. */
struct MeshcatParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(host));
    a->Visit(DRAKE_NVP(port));
    a->Visit(DRAKE_NVP(web_url_pattern));
    a->Visit(DRAKE_NVP(show_stats_plot));
  }

  /** Meshcat will listen only on the given hostname (e.g., "localhost").
  If "*" is specified, then it will listen on all interfaces.
  If empty, an appropriate default value will be chosen (currently "*"). */
  std::string host{"*"};

  /** Meshcat will listen on the given http `port`. If no port is specified,
  then it will listen on the first available port starting at 7000 (up to 7999).
  If port 0 is specified, it will listen on an arbitrary "ephemeral" port.
  @pre We require `port` == 0 || `port` >= 1024. */
  std::optional<int> port{std::nullopt};

  /** The `web_url_pattern` may be used to change the web_url() (and therefore
  the ws_url()) reported by Meshcat. This may be useful in case %Meshcat sits
  behind a firewall or proxy.

  The pattern follows the
  <a href="https://en.cppreference.com/w/cpp/utility/format">std::format</a>
  specification language, except that `arg-id` substitutions are performed
  using named arguments instead of positional indices.

  There are two arguments available to the pattern:
  - `{port}` will be substituted with the %Meshcat server's listen port number;
  - `{host}` will be substituted with this params structure's `host` field, or
    else with "localhost" in case the `host` was one of the placeholders for
    "all interfaces".
  */
  std::string web_url_pattern{"http://{host}:{port}"};

  /** Determines whether or not to display the stats plot widget in the Meshcat
  user interface. This plot including realtime rate and WebGL render
  statistics. */
  bool show_stats_plot{true};
};

}  // namespace geometry
}  // namespace drake
