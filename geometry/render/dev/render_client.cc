#include "drake/geometry/render/dev/render_client.h"

#include <atomic>
#include <string>

#include <curl/curl.h>

#include "drake/common/temp_directory.h"

namespace drake {
namespace geometry {
namespace render {

namespace {

// Render client constructor increases, destructor decreases n_clients.
std::atomic<int> n_clients{0};
// is_initialized should only be modified in static_curl_{init,cleanup}.
std::atomic<bool> is_initialized{false};

void static_curl_init() {
  if (!is_initialized) {
    curl_global_init(CURL_GLOBAL_ALL);
    is_initialized = true;
  }
}

void static_curl_cleanup() {
  if (n_clients == 0) {
    curl_global_cleanup();
    is_initialized = false;
  }
}

}  // namespace

RenderClient::RenderClient(const RenderClientParams& parameters)
    : RenderEngine(parameters.default_label ? *parameters.default_label
                                            : RenderLabel::kUnspecified),
      temp_directory_{drake::temp_directory()},
      url_{parameters.url},
      port_{parameters.port},
      upload_endpoint_{parameters.upload_endpoint},
      render_endpoint_{parameters.render_endpoint},
      curl_verbose_{parameters.curl_verbose} {
  static_curl_init();
  ++n_clients;
}

RenderClient::RenderClient(const RenderClient& other)
    : RenderEngine(other),
      temp_directory_{other.temp_directory_},
      url_{other.url_},
      port_{other.port_},
      upload_endpoint_{other.upload_endpoint_},
      render_endpoint_{other.render_endpoint_},
      curl_verbose_{other.curl_verbose_} {
  static_curl_init();
  ++n_clients;
}

RenderClient::~RenderClient() {
  --n_clients;
  static_curl_cleanup();
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
