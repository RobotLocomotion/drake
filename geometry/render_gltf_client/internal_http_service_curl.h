#pragma once

#include <map>
#include <optional>
#include <string>
#include <utility>

#include "drake/geometry/render_gltf_client/internal_http_service.h"

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

 protected:
  /* @see HttpService::DoPostForm */
  HttpResponse DoPostForm(const std::string& temp_directory,
                          const std::string& url,
                          const DataFieldsMap& data_fields,
                          const FileFieldsMap& file_fields,
                          bool verbose = false) override;
};

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
