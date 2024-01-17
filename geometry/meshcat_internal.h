#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace drake {
namespace geometry {
namespace internal {

/* Returns the static content for the given URL, or an nullopt when the URL is
invalid. The valid static resource URLs are:
- `/`
- `/favicon.ico`
- `/index.html`
- `/meshcat.html`
- `/meshcat.js`
- `/stats.min.js` */
std::optional<std::string_view> GetMeshcatStaticResource(
    std::string_view url_path);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
