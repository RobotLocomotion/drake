#include "drake/common/yaml/yaml_write_archive.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "third_party/com_github_jbeder_yaml_cpp/include/yaml-cpp/emitfromevents.h"  // NOLINT
#include <yaml-cpp/yaml.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace yaml {
namespace {

// Boilerplate for std::visit.
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

constexpr const char* const kKeyOrder = "__key_order";

// This function uses the same approach as YAML::NodeEvents::Emit.
// https://github.com/jbeder/yaml-cpp/blob/release-0.5.2/src/nodeevents.cpp#L55
//
// The `sink` object keeps track of document state.  Our job is to feed it with
// an event stream (e.g., start mapping, "foo", start sequence, "1", "2",
// end sequence, end mapping) and then its job is to spit out the equivalent
// YAML syntax for that stream (e.g., "foo: [1, 2]") with appropriately matched
// delimiters (i.e., `:` or `{}` or `[]`) and horizontal indentation levels.
void RecursiveEmit(const internal::Node& node, YAML::EmitFromEvents* sink) {
  const YAML::Mark no_mark;
  const YAML::anchor_t no_anchor = YAML::NullAnchor;
  node.Visit(overloaded{
    [&](const internal::Node::ScalarData& data) {
      sink->OnScalar(no_mark, node.GetTag(), no_anchor, data.scalar);
    },
    [&](const internal::Node::SequenceData& data) {
      // If all children are scalars, then format this sequence onto a
      // single line; otherwise, format as a bulleted list.
      auto style = YAML::EmitterStyle::Flow;
      for (const auto& child : data.sequence) {
        if (!child.IsScalar()) {
          style = YAML::EmitterStyle::Block;
        }
      }
      sink->OnSequenceStart(no_mark, node.GetTag(), no_anchor, style);
      for (const auto& child : data.sequence) {
        RecursiveEmit(child, sink);
      }
      sink->OnSequenceEnd();
    },
    [&](const internal::Node::MappingData& data) {
      // If there are no children, then format this map onto a single line;
      // otherwise, format over multiple "key: value\n" lines.
      auto style = YAML::EmitterStyle::Block;
      if (data.mapping.empty()) {
        style = YAML::EmitterStyle::Flow;
      }
      sink->OnMapStart(no_mark, node.GetTag(), no_anchor, style);
      // If there is a __key_order node inserted (as part of the Accept()
      // member function in our header file), use it to specify output order;
      // otherwise, use alphabetical order.
      std::vector<std::string> key_order;
      if (data.mapping.count(kKeyOrder)) {
        const internal::Node& key_order_node = data.mapping.at(kKeyOrder);
        // Use Accept()'s ordering.  (If EraseMatchingMaps has been called,
        // some of the keys may have disappeared.)
        for (const auto& item : key_order_node.GetSequence()) {
          const std::string& key = item.GetScalar();
          if (data.mapping.count(key)) {
            key_order.push_back(key);
          }
        }
      } else {
        // Use alphabetical ordering.
        for (const auto& [key, value] : data.mapping) {
          unused(value);
          key_order.push_back(key);
        }
      }
      for (const auto& string_key : key_order) {
        RecursiveEmit(internal::Node::MakeScalar(string_key), sink);
        RecursiveEmit(data.mapping.at(string_key), sink);
      }
      sink->OnMapEnd();
    },
  });
}

}  // namespace

const char* const YamlWriteArchive::kKeyOrderName = kKeyOrder;

// Convert the given document to a string ala YAML::Dump, but emit Map nodes
// using a specific key ordering (the visit order in case of structs, or else
// alphabetical key order for any other kind of Map node).
std::string YamlWriteArchive::YamlDumpWithSortedMaps(
    const internal::Node& document) {
  YAML::Emitter emitter;
  YAML::EmitFromEvents sink(emitter);
  RecursiveEmit(document, &sink);
  return emitter.c_str();
}

std::string YamlWriteArchive::EmitString(const std::string& root_name) const {
  std::string result;
  if (root_.GetMapping().empty()) {
    if (root_name.empty()) {
      result = "{}\n";
    } else {
      result = root_name + ":\n";
    }
  } else {
    if (root_name.empty()) {
      result = YamlDumpWithSortedMaps(root_) + "\n";
    } else {
      auto document = internal::Node::MakeMapping();
      document.Add(root_name, root_);
      result = YamlDumpWithSortedMaps(document) + "\n";
    }
  }
  return result;
}

namespace {

// Implements YamlWriteArchive::EraseMatchingMaps recursively.
void DoEraseMatchingMaps(internal::Node* x, const internal::Node* y) {
  DRAKE_DEMAND((x != nullptr) && (y != nullptr));

  // If x and y are different types, then no subtraction can be done.
  // If their type is non-map, then we do not subtract them.  The reader's
  // retain_map_defaults mode only merges default maps; it does not, e.g.,
  // concatenate sequences.
  if (!(x->IsMapping())) { return; }
  if (!(y->IsMapping())) { return; }
  const std::map<std::string, internal::Node>& y_map = y->GetMapping();

  // Both x are y are maps.  Remove from x any key-value pair that is identical
  // within both x and y.
  std::vector<std::string> keys_to_prune;
  for (const auto& [x_key, x_value] : x->GetMapping()) {
    if (x_key == kKeyOrder) {
      // Subtraction should always leave the special kKeyOrder node alone when
      // considering which keys to prune.  If any keys are unpruned, we still
      // want to know in what order they should appear.  The only way to remove
      // a kKeyOrder node is when its enclosing Map is removed wholesale.
      continue;
    }
    auto iter = y_map.find(x_key);
    if (iter == y_map.end()) {
      // The key only appears in x, so we should neither prune nor recurse.
      continue;
    }
    const internal::Node& y_value = iter->second;
    if (x_value == y_value) {
      // The values match, so we should prune.  (We can't call Remove here,
      // because it would invalidate the range-for iterator.)
      keys_to_prune.push_back(x_key);
      continue;
    }
    // The maps are tagged differently, so we should not subtract their
    // children, since they may have different semantics.
    if (x_value.GetTag() != y_value.GetTag()) {
      continue;
    }
    // Recurse into any children of x and y with the same key name.
    internal::Node& x_value_mutable = x->At(x_key);
    DoEraseMatchingMaps(&x_value_mutable, &y_value);
  }
  for (const auto& key : keys_to_prune) {
    x->Remove(key);
  }
}

}  // namespace

void YamlWriteArchive::EraseMatchingMaps(const YamlWriteArchive& other) {
  DoEraseMatchingMaps(&(this->root_), &(other.root_));
}

}  // namespace yaml
}  // namespace drake
