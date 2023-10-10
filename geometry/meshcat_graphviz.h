#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "drake/systems/framework/system_base.h"

namespace drake {
namespace geometry {
namespace internal {

/* Encapsulates how to annotate use of Meshcat in a System's Graphviz.

Use this class within the implementation of DoGetGraphvizFragment, e.g.:

```c++
// Overrides the Parent::DoGetGraphvizFragment member function.
Parent::GraphvizFragment MySystem::DoGetGraphvizFragment(
    const Parent::GraphvizFragmentParams& params) const override {
  MeshcatGraphviz helper(...);
  return helper.DecorateResult(
      Parent::DoGetGraphvizFragment(helper.DecorateParams(params)));
}
```

Grep for existing uses of this class to find other examples. */
class MeshcatGraphviz {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatGraphviz);

  /* Specifies how to customize the Graphviz for a Meshcat-using System.
  @param path When the system publishes messages to Meshcat, provide the
   (non-null) meshcat path in this parameter. This draws a Graphviz arrow from
   the system to meshcat. See "Meshcat paths and the scene tree" in Drake's
   meshcat class overview docs for the semantics of `path`. When this system is
   subscribe-only, pass nullopt instead.
  @param subscribe True iff the system receives messages from meshcat (e.g.,
   uses buttons or sliders from control panel). This draws a Graphviz arrow
   from meshcat to the system. */
  MeshcatGraphviz(std::optional<std::string_view> path, bool subscribe);

  /* Rewrites the `params` to customize for Meshcat, by returning a possibly-
  edited copy. Intended for use alongside DecorateResult(). */
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
