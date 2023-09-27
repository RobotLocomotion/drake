#pragma once

#include <string>
#include <string_view>

#include "drake/systems/framework/system_base.h"

namespace drake {
namespace geometry {
namespace internal {

/* Encapsulates how to annotate use of Meshcat in a System's Graphviz. */
class MeshcatGraphviz {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatGraphviz);

  MeshcatGraphviz(std::string_view path, bool publish, bool subscribe);

  systems::SystemBase::GraphvizFragmentParams DecorateParams(
      const systems::SystemBase::GraphvizFragmentParams& params);

  systems::SystemBase::GraphvizFragment DecorateResult(
      systems::SystemBase::GraphvizFragment&& result);

 private:
  const std::string path_;
  const bool publish_;
  const bool subscribe_;

  std::string node_id_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
