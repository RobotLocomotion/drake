#include "drake/systems/lcm/lcm_system_graphviz.h"

#include <utility>

namespace drake {
namespace systems {
namespace lcm {
namespace internal {

using drake::lcm::DrakeLcmInterface;

using Params = systems::SystemBase::GraphvizFragmentParams;
using Result = systems::SystemBase::GraphvizFragment;

LcmSystemGraphviz::LcmSystemGraphviz(const DrakeLcmInterface& lcm,
                                     std::string_view channel,
                                     const std::type_info* message_type,
                                     bool publish, bool subscribe)
    : lcm_interface_node_id_{get_node_id(lcm)},
      channel_line_{fmt::format("channel={}", channel)},
      type_line_{message_type != nullptr
                     ? std::optional<std::string>{fmt::format(
                           "type={}", NiceTypeName::RemoveNamespaces(
                                          NiceTypeName::Get(*message_type)))}
                     : std::nullopt},
      publish_{publish},
      subscribe_{subscribe} {}

Params LcmSystemGraphviz::DecorateParams(const Params& params) {
  node_id_ = params.node_id;
  Params new_params{params};
  if (publish_ || subscribe_) {
    new_params.header_lines.push_back(channel_line_);
  }
  if (type_line_.has_value()) {
    new_params.header_lines.push_back(*type_line_);
  }
  return new_params;
}

Result LcmSystemGraphviz::DecorateResult(Result&& result) {
  Result new_result = std::move(result);
  DRAKE_THROW_UNLESS(!node_id_.empty());
  if (publish_) {
    new_result.fragments.push_back(
        fmt::format("{}:e -> {}in [style=\"dashed\", color=\"{}\"];\n",
                    node_id_, lcm_interface_node_id_, get_color()));
  }
  if (subscribe_) {
    new_result.fragments.push_back(
        fmt::format("{}out -> {}:w [style=\"dashed\", color=\"{}\"];\n",
                    lcm_interface_node_id_, node_id_, get_color()));
  }
  return new_result;
}

std::string LcmSystemGraphviz::get_node_id(
    const drake::lcm::DrakeLcmInterface& lcm) {
  return fmt::format("drakelcminterface{}", reinterpret_cast<uintptr_t>(&lcm));
}

std::string_view LcmSystemGraphviz::get_color() {
  return "webpurple";
}

}  // namespace internal
}  // namespace lcm
}  // namespace systems
}  // namespace drake
