#include "drake/common/yaml/yaml_write_archive.h"

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>

#include <yaml-cpp/emitfromevents.h>
#include <yaml-cpp/yaml.h>

#include "drake/common/drake_assert.h"
#include "drake/common/overloaded.h"
#include "drake/common/unused.h"

namespace drake {
namespace yaml {
namespace internal {
namespace {

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
  std::string tag{node.GetTag()};
  if ((tag == internal::Node::kTagNull) || (tag == internal::Node::kTagBool) ||
      (tag == internal::Node::kTagInt) || (tag == internal::Node::kTagFloat) ||
      (tag == internal::Node::kTagStr) || (tag == internal::Node::kTagBinary)) {
    // In most cases we don't need to emit the "JSON Schema" tags for YAML data,
    // because they are implied by default. However, YamlWriteArchive on variant
    // types sometimes marks the tag as important.
    if (node.IsTagImportant()) {
      DRAKE_DEMAND(tag.size() > 0);
      // The `internal::Node::kTagFoo` all look like "tag:yaml.org,2002:foo".
      // We only want the "foo" part (after the second colon).
      tag = "!!" + tag.substr(18);
    } else {
      tag.clear();
    }
  }
  node.Visit(overloaded{
      [&](const internal::Node::ScalarData& data) {
        sink->OnScalar(no_mark, tag, no_anchor, data.scalar);
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
        sink->OnSequenceStart(no_mark, tag, no_anchor, style);
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
        sink->OnMapStart(no_mark, tag, no_anchor, style);
        // If there is a __key_order node inserted (as part of the Accept()
        // member function in our header file), use it to specify output order;
        // otherwise, use alphabetical order.
        std::vector<std::string> key_order;
        if (data.mapping.contains(kKeyOrder)) {
          const internal::Node& key_order_node = data.mapping.at(kKeyOrder);
          // Use Accept()'s ordering.  (If EraseMatchingMaps has been called,
          // some of the keys may have disappeared.)
          for (const auto& item : key_order_node.GetSequence()) {
            const std::string& key = item.GetScalar();
            if (data.mapping.contains(key)) {
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

/* Returns a quoted and escaped JSON string literal per
https://www.json.org/json-en.html. */
std::string QuotedJsonString(std::string_view x) {
  std::string result;
  result.reserve(x.size() + 2);
  result.push_back('"');
  for (const char ch : x) {
    if ((ch == '"') || (ch == '\\') || (ch < ' ')) {
      // This character needs escaping.
      result.append(fmt::format("\\u00{:02x}", static_cast<uint8_t>(ch)));
    } else {
      // No escaping required.
      result.push_back(ch);
    }
  }
  result.push_back('"');
  return result;
}

/* Outputs the given `node` to `os`. This is the recursive implementation of
YamlWriteArchive::ToJson. */
void WriteJson(std::ostream& os, const internal::Node& node) {
  const std::string_view tag = node.GetTag();
  switch (node.GetType()) {
    case internal::NodeType::kScalar: {
      const std::string& scalar = node.GetScalar();
      if ((tag == internal::Node::kTagNull) ||
          (tag == internal::Node::kTagBool) ||
          (tag == internal::Node::kTagInt)) {
        // These values are spelled the same in YAML and JSON (without quotes).
        os << scalar;
        return;
      }
      if (tag == internal::Node::kTagFloat) {
        // Floats are the same in YAML and JSON except for non-finite values.
        if (scalar == ".nan") {
          os << "NaN";
          return;
        }
        if (scalar == ".inf") {
          os << "Infinity";
          return;
        }
        if (scalar == "-.inf") {
          os << "-Infinity";
          return;
        }
        os << scalar;
        return;
      }
      if (tag.empty() || (tag == internal::Node::kTagStr)) {
        // JSON strings are quoted and escaped.
        os << QuotedJsonString(scalar);
        return;
      }
      throw std::logic_error(fmt::format(
          "SaveJsonString: Cannot save a YAML scalar '{}' with tag '{}' "
          "to JSON",
          scalar, tag));
    }
    case internal::NodeType::kSequence: {
      if (!tag.empty()) {
        // N.B. As currently written, yaml_write_archive can never add a tag to
        // a sequence. We'll still fail-fast in case it ever does.
        throw std::logic_error(fmt::format(
            "SaveJsonString: Cannot save a YAML sequence with tag '{}' to JSON",
            tag));
      }
      os << "[";
      const auto& node_vector = node.GetSequence();
      bool first = true;
      for (const auto& sub_node : node_vector) {
        if (first) {
          first = false;
        } else {
          os << ",";
        }
        WriteJson(os, sub_node);
      }
      os << "]";
      return;
    }
    case internal::NodeType::kMapping: {
      if (!tag.empty()) {
        throw std::logic_error(fmt::format(
            "SaveJsonString: Cannot save a YAML mapping with tag '{}' to JSON",
            tag));
      }
      os << "{";
      bool first = true;
      for (const auto& [name, sub_node] : node.GetMapping()) {
        if (name == kKeyOrder) {
          // For YAML output, we use a __key_order node to improve readability
          // of the output (i.e., to follow the schema declaration order). For
          // JSON, we don't expect the output to be readable in the first place
          // so we'll simply ignore the declaration order and use the default
          // (alphabetical) ordering instead.
          continue;
        }
        if (first) {
          first = false;
        } else {
          os << ",";
        }
        os << QuotedJsonString(name);
        os << ":";
        WriteJson(os, sub_node);
      }
      os << "}";
      return;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

std::string YamlWriteArchive::ToJson() const {
  std::stringstream result;
  WriteJson(result, root_);
  return result.str();
}

namespace {

// Implements YamlWriteArchive::EraseMatchingMaps recursively.
void DoEraseMatchingMaps(internal::Node* x, const internal::Node* y) {
  DRAKE_DEMAND((x != nullptr) && (y != nullptr));

  // If x and y are different types, then no subtraction can be done.
  // If their type is non-map, then we do not subtract them.  The reader's
  // retain_map_defaults mode only merges default maps; it does not, e.g.,
  // concatenate sequences.
  if (!(x->IsMapping())) {
    return;
  }
  if (!(y->IsMapping())) {
    return;
  }
  const string_map<internal::Node>& y_map = y->GetMapping();

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

namespace {

template <typename T>
bool ParseAs(const std::string& s) {
  T junk;
  return YAML::convert<T>::decode(YAML::Node(s), junk);
}

}  // namespace

void YamlWriteArchive::VisitPathScalar(const char* name,
                                       std::filesystem::path value) {
  std::string path_str = value.string();
  // We need to know if the path can be interpreted as a primitive (e.g.,
  // the file 1234.5).
  const bool is_primitive = [&path_str]() {
    return (ParseAs<bool>(path_str) || ParseAs<double>(path_str) ||
            ParseAs<int64_t>(path_str) || ParseAs<uint64_t>(path_str));
  }();

  // Empty paths should become ".".
  std::string prepped = value.empty() ? std::string(".") : std::move(path_str);

  auto scalar = internal::Node::MakeScalar(std::move(prepped));
  if (is_primitive) {
    scalar.SetTag(internal::JsonSchemaTag::kStr, /* important */ true);
  }
  root_.Add(name, std::move(scalar));
}

void YamlWriteArchive::VisitStringScalar(const char* name, std::string value) {
  const bool is_binary = [&value]() {
    for (char c : value) {
      if (!std::isprint(c) && c != '\n' && c != '\r' && c != '\t') return true;
    }
    return false;
  }();
  if (is_binary) {
    std::string encoded = YAML::EncodeBase64(
        reinterpret_cast<const unsigned char*>(value.c_str()), value.size());
    auto scalar = internal::Node::MakeScalar(std::move(encoded));
    scalar.SetTag(internal::JsonSchemaTag::kBinary, /* important= */ true);
    root_.Add(name, std::move(scalar));
  } else {
    root_.Add(name, internal::Node::MakeScalar(std::move(value)));
  }
}

}  // namespace internal
}  // namespace yaml
}  // namespace drake
