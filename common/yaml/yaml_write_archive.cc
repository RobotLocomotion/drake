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
        // Use Accept()'s ordering.  (If SubstractDefaults has been called,
        // some of the keys may have disappeared.)
        for (const auto& item : key_order_node) {
          const std::string& key = item.Scalar();
          if (node[key].IsDefined()) {
            key_order.push_back(key);
          }
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

std::string YamlWriteArchive::EmitString(const std::string& root_name) const {
  std::string result;
  if (root_.IsNull()) {
    if (root_name.empty()) {
      result = "{}\n";
    } else {
      result = root_name + ":\n";
    }
  } else {
    if (root_name.empty()) {
      result = YamlDumpWithSortedMaps(root_) + "\n";
    } else {
      YAML::Node document;
      document[root_name] = root_;
      result = YamlDumpWithSortedMaps(document) + "\n";
    }
  }
  return result;
}

namespace {

// Returns true iff x and y have a identical node types and values throughout
// their entire tree structure, but without trying to match representationally
// distinct but semantically equal values (e.g., 0x0a compared to 10 may return
// false even though they refer to the same integer).
//
// See https://github.com/jbeder/yaml-cpp/issues/274 for a feature request to
// provide this kind of function directly as part of yaml-cpp.
bool AreLexicallyEqual(const YAML::Node& x, const YAML::Node& y) {
  DRAKE_DEMAND(x.IsDefined() && y.IsDefined());
  const YAML::NodeType::value type = x.Type();
  if (y.Type() != type) {
    return false;
  }
  switch (type) {
    case YAML::NodeType::Undefined: { DRAKE_UNREACHABLE(); }
    case YAML::NodeType::Null: {
      // Two nulls are always equal to each other.
      return true;
    }
    case YAML::NodeType::Scalar: {
      return x.Scalar() == y.Scalar();
    }
    case YAML::NodeType::Sequence: {
      size_t size = x.size();
      if (y.size() != size) {
        return false;
      }
      for (size_t i = 0; i < size; ++i) {
        if (!AreLexicallyEqual(x[i], y[i])) {
          return false;
        }
      }
      return true;
    }
    case YAML::NodeType::Map: {
      if (x.size() != y.size()) {
        return false;
      }
      if (x.Tag() != y.Tag()) {
        return false;
      }
      for (const auto& x_key_value : x) {
        const std::string& key = x_key_value.first.Scalar();
        const YAML::Node& x_val = x_key_value.second;
        const YAML::Node& y_val = y[key];
        if (!y_val.IsDefined()) {
          return false;
        }
        if (!AreLexicallyEqual(x_val, y_val)) {
          return false;
        }
      }
      return true;
    }
  }
  DRAKE_UNREACHABLE();
}

// Implements YamlWriteArchive::EraseMatchingMaps recursively.
void DoEraseMatchingMaps(YAML::Node* x, const YAML::Node* y) {
  // If x and y are different types, then no subtraction that can be done.
  DRAKE_DEMAND((x != nullptr) && (y != nullptr));
  DRAKE_DEMAND(x->IsDefined() && y->IsDefined());
  const YAML::NodeType::value type = x->Type();
  if (y->Type() != type) {
    // If the types differ, we should not subtract.
    return;
  }
  // If their type is non-map, then we do not subtract them.  The reader's
  // retain_map_defaults mode only merges default maps; it does not, e.g.,
  // concatenate sequences.
  switch (type) {
    case YAML::NodeType::Undefined: { DRAKE_UNREACHABLE(); }
    case YAML::NodeType::Null: { return; }
    case YAML::NodeType::Scalar: { return; }
    case YAML::NodeType::Sequence: { return; }
    case YAML::NodeType::Map: {
      break;
    }
  }
  // Both x are y are maps.  Remove from x any key-value pair that is identical
  // within both x and y.
  std::vector<std::string> keys_to_prune;
  for (const auto& x_key_value : *x) {
    const std::string& key = x_key_value.first.Scalar();
    if (key == kKeyOrder) {
      // Subtraction should always leave the special kKeyOrder node alone when
      // considering which keys to prune.  If any keys are unpruned, we still
      // want to know in what order they should appear.  The only way to remove
      // a kKeyOrder node is when its enclosing Map is removed wholesale.
      continue;
    }
    const YAML::Node& x_val = x_key_value.second;
    const YAML::Node& y_val = (*y)[key];
    if (!y_val.IsDefined()) {
      // The key only appears in x, so we should not prune.
      continue;
    }
    if (!AreLexicallyEqual(x_val, y_val)) {
      // The values do not match, so we should not prune.
      continue;
    }
    keys_to_prune.push_back(key);
  }
  for (const auto& key : keys_to_prune) {
    x->remove(key);
  }
  // Recurse into any children of x and y with the same key name.
  for (auto&& x_key_value : *x) {
    const std::string& key = x_key_value.first.Scalar();
    if (key == kKeyOrder) {
      // Subtraction should always leave the special kKeyOrder node alone.
      // (See the other kKeyOrder guard a few lines above for details.)
      continue;
    }
    YAML::Node& x_val = x_key_value.second;
    const YAML::Node& y_val = (*y)[key];
    if (!y_val.IsDefined()) {
      // The key only appears in x, so we cannot recurse.
      continue;
    }
    if (x_val.Tag() != y_val.Tag()) {
      // The maps are tagged differently, so we should not subtract their
      // children, since they may have different semantics.
      continue;
    }
    DoEraseMatchingMaps(&x_val, &y_val);
  }
}

}  // namespace

void YamlWriteArchive::EraseMatchingMaps(const YamlWriteArchive& other) {
  // Note that the implementations of DoEraseMatchingMaps and AreLexicallyEqual
  // are both recursive.  It's possible that we could improve performance by
  // merging them into a single recursive traveral.  However, for readability
  // they are kept separate until we see a proven need for that optimization.
  DoEraseMatchingMaps(&(this->root_), &(other.root_));
}

}  // namespace yaml
}  // namespace drake
