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

  /* Specifies how to customize the Graphviz for a Meshcat-using System.
  @param path When `publish` is `true`, this is the meshcat path. See "Meshcat
   paths and the scene tree" in Drake's meshcat class overview docs.
  @param publish True iff the system publishes messages to meshcat. This draws
   a Graphviz arrow from the system to meshcat.
  @param subscribe True iff the system receives messages from meshcat (e.g.,
   uses buttons or sliders from control panel). This draws a Graphviz arrow
   from meshcat to the system. */
  MeshcatGraphviz(std::string_view path, bool publish, bool subscribe);

  /* Rewrites the `params` to customize for Meshcat, by returning a possibly-
  edited copy. */
  systems::SystemBase::GraphvizFragmentParams DecorateParams(
      const systems::SystemBase::GraphvizFragmentParams& params);

  /* Rewrites the `result` to customize for Meshcat.
  @pre DecorateParams has already been called. */
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
