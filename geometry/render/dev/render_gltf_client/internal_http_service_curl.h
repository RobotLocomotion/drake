#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/geometry/render/dev/render_gltf_client/internal_http_service.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* An HttpService that uses libcurl to communicate with the server. */
class HttpServiceCurl : public HttpService {
 public:
  HttpServiceCurl();
  ~HttpServiceCurl() override;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HttpServiceCurl);

  /* @see HttpService::PostForm */
  HttpResponse PostForm(
      const std::string& temp_directory, const std::string& url, int port,
      const std::string& endpoint,
      const std::map<std::string, std::string>& data_fields,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields,
      bool verbose = false) override;
};

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
