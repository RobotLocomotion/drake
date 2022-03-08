#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/geometry/render/dev/http_service.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/** An HttpService that uses libcurl to communicate with the server. */
class HttpServiceCurl : public HttpService {
 public:
  /** Constructs a libcurl based HttpService.  @sa HttpService::HttpService */
  HttpServiceCurl(const std::string& temp_directory, const std::string& url,
                  int32_t port, bool verbose);
  ~HttpServiceCurl() override;

  /** @see HttpService::PostForm */
  HttpResponse PostForm(
      const std::string& endpoint,
      const std::map<std::string, std::string>& data_fields,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields) override;

 protected:
  /** Copy constructor for the purpose of cloning. */
  HttpServiceCurl(const HttpServiceCurl& other);

  /** Clones this %HttpServiceCurl. */
  std::unique_ptr<HttpService> DoClone() const override;
};

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
