#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {
namespace lcm {
namespace internal {

/* Encapsulates how to annotate use of LCM in a System's Graphviz.

Use this class within the implementation of DoGetGraphvizFragment, e.g.:

```c++
// Overrides the Parent::DoGetGraphvizFragment member function.
Parent::GraphvizFragment MySystem::DoGetGraphvizFragment(
    const Parent::GraphvizFragmentParams& params) const override {
  LcmSystemGraphviz helper(...);
  return helper.DecorateResult(
      Parent::DoGetGraphvizFragment(helper.DecorateParams(params)));
}
```

Grep for existing uses of this class to find other examples. */
class LcmSystemGraphviz {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmSystemGraphviz);

  /* Specifies how to customize the Graphviz for a LCM-using System.
  @param lcm The interface the system communicates with.
  @param channel The channel name the system publishes to (or receives from).
  @param message_type (Optional) The `typeid(lcmt_foo_bar)` type the system
   publishes (or receives).
  @param publish True iff the system transmits messages.
   This draws a Graphviz arrow from the system to `lcm`.
  @param subscribe True iff the system receives messages.
   This draws a Graphviz arrow from `lcm` to the system. */
  LcmSystemGraphviz(const drake::lcm::DrakeLcmInterface& lcm,
                    std::string_view channel,
                    const std::type_info* message_type, bool publish,
                    bool subscribe);

  /* Rewrites the `params` to customize for LCM, by returning a possibly-
  edited copy. Intended for use alongside DecorateResult(). */
  SystemBase::GraphvizFragmentParams DecorateParams(
      const SystemBase::GraphvizFragmentParams& params);

  /* Rewrites the `result` to customize for LCM.
  @pre DecorateParams has already been called. */
  SystemBase::GraphvizFragment DecorateResult(
      SystemBase::GraphvizFragment&& result);

  /* Returns the Graphviz node ID for the given `lcm` interface. */
  static std::string get_node_id(const drake::lcm::DrakeLcmInterface& lcm);

  /* Returns the color for LCM nodes and edges. */
  static std::string_view get_color();

 private:
  const std::string lcm_interface_node_id_;
  const std::string channel_line_;
  const std::optional<std::string> type_line_;
  const bool publish_;
  const bool subscribe_;

  std::string node_id_;
};

}  // namespace internal
}  // namespace lcm
}  // namespace systems
}  // namespace drake
