#include "drake/geometry/meshcat_graphviz.h"

#include <utility>

namespace drake {
namespace geometry {
namespace internal {

using Params = systems::SystemBase::GraphvizFragmentParams;
using Result = systems::SystemBase::GraphvizFragment;

namespace {
/* Resolves a possibly-relative path, per the policy of the Meshcat class. */
std::string ResolvePath(std::string_view path) {
  if (path.substr(0, 1) == "/") {
    return std::string{path};
  }
  if (path.empty()) {
    return "/drake";
  }
  return fmt::format("/drake/{}", path);
}
}  // namespace

MeshcatGraphviz::MeshcatGraphviz(std::optional<std::string_view> path,
                                 bool subscribe)
    : path_(ResolvePath(path.value_or(""))),
      publish_(path.has_value()),
      subscribe_(subscribe) {}

Params MeshcatGraphviz::DecorateParams(const Params& params) {
  node_id_ = params.node_id;
  Params new_params{params};
  if (publish_) {
    new_params.header_lines.push_back(fmt::format("path={}", path_));
  }
  return new_params;
}

Result MeshcatGraphviz::DecorateResult(Result&& result) {
  Result new_result = std::move(result);
  DRAKE_THROW_UNLESS(!node_id_.empty());
  if (publish_) {
    new_result.fragments.push_back(
        fmt::format("meshcat_in [label=Meshcat, color=magenta];\n"
                    "{}:e -> meshcat_in [style=dashed, color=magenta]\n",
                    node_id_));
  }
  if (subscribe_) {
    new_result.fragments.push_back(
        fmt::format("meshcat_out [label=Meshcat, color=magenta];\n"
                    "meshcat_out -> {}:w [style=dashed, color=magenta]\n",
                    node_id_));
  }
  return new_result;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
