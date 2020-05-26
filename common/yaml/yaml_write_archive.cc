#include "drake/common/yaml/yaml_write_archive.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "third_party/com_github_jbeder_yaml_cpp/include/yaml-cpp/emitfromevents.h"  // NOLINT

#include "drake/common/drake_assert.h"

namespace drake {
namespace yaml {
namespace {

constexpr const char* const kKeyOrder = "__key_order";

// This function uses the same approach as YAML::NodeEvents::Emit.
// https://github.com/jbeder/yaml-cpp/blob/release-0.5.2/src/nodeevents.cpp#L55
void RecursiveEmit(const YAML::Node& node, YAML::EmitFromEvents* sink) {
  const YAML::Mark no_mark;
  const YAML::anchor_t no_anchor = YAML::NullAnchor;
  switch (node.Type()) {
    case YAML::NodeType::Undefined: { return; }
    case YAML::NodeType::Null: { return; }
    case YAML::NodeType::Scalar: {
      sink->OnScalar(no_mark, node.Tag(), no_anchor, node.Scalar());
      return;
    }
    case YAML::NodeType::Sequence: {
      // If all children are scalars, then format this sequence onto a
      // single line; otherwise, format as a bulleted list.
      auto style = YAML::EmitterStyle::Flow;
      for (const auto& child : node) {
        if (child.IsSequence() || child.IsMap()) {
          style = YAML::EmitterStyle::Block;
        }
      }
      sink->OnSequenceStart(no_mark, node.Tag(), no_anchor, style);
      for (const auto& child : node) {
        RecursiveEmit(child, sink);
      }
      sink->OnSequenceEnd();
      return;
    }
    case YAML::NodeType::Map: {
      // If there are no children, then format this map onto a single line;
      // otherwise, format over multiple "key: value\n" lines.
      auto style = YAML::EmitterStyle::Block;
      if (node.size() == 0) {
        style = YAML::EmitterStyle::Flow;
      }
      sink->OnMapStart(no_mark, node.Tag(), no_anchor, style);
      // In yaml-cpp < 0.6.0, the iteration order of yaml-cpp NodeType::Map is
      // not deterministic.  If there is a __key_order node inserted (as part
      // of the Accept() method in our header file), use it to specify output
      // order; otherwise, use alphabetical order.
      std::vector<std::string> key_order;
      const YAML::Node key_order_node = node[kKeyOrder];
      if (key_order_node) {
        // Use Accept()'s ordering.
        for (const auto& item : key_order_node) {
          key_order.push_back(item.Scalar());
        }
      } else {
        // Use alphabetical ordering.
        for (const auto& yaml_key_value : node) {
          key_order.push_back(yaml_key_value.first.Scalar());
        }
        std::sort(key_order.begin(), key_order.end());
      }
      for (const auto& string_key : key_order) {
        const YAML::Node key(string_key);
        RecursiveEmit(key, sink);
        RecursiveEmit(node[string_key], sink);
      }
      sink->OnMapEnd();
      return;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

const char* const YamlWriteArchive::kKeyOrderName = kKeyOrder;

// Convert the given document to a string ala YAML::Dump, but emit Map nodes
// using a deterministic key ordering.  (By default, the ordering build in to
// YAML::Dump is non-deterministic in < 0.6 and addition-order in >= 0.6.  Once
// we are using >= 0.6, perhaps this function can evaporate.)
std::string YamlWriteArchive::YamlDumpWithSortedMaps(
    const YAML::Node& document) {
  YAML::Emitter emitter;
  YAML::EmitFromEvents sink(emitter);
  RecursiveEmit(document, &sink);
  return emitter.c_str();
}

}  // namespace yaml
}  // namespace drake
