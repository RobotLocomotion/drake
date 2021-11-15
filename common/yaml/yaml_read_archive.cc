#include "drake/common/yaml/yaml_read_archive.h"

#include <algorithm>
#include <cstring>

#include <fmt/ostream.h>
#include <yaml-cpp/yaml.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace yaml {
namespace {

// The source and destination are both of type Map.  Copy the key-value pairs
// from source into destination, but don't overwrite any existing keys.
void CopyWithMergeKeySemantics(
    const YAML::Node& source, YAML::Node* destination) {
  for (const auto& key_value : source) {
    const YAML::Node& key = key_value.first;
    const YAML::Node& value = key_value.second;
    YAML::Node entry = (*destination)[key.Scalar()];
    if (!entry) {
      // N.B. This assignment indirectly mutates the `destination`!  With the
      // yaml-cpp API, a YAML::Node returned from a mapping lookup (operator[])
      // is a back-pointer into the mapping -- something akin to an iterator.
      entry = value;
    }
  }
}

// If the given `node` (of type Map) has a YAML merge key defined, then mutates
// the `node` in place to replace the merge key entry with the merged values.
//
// See https://yaml.org/type/merge.html for details on syntax and semantics.
//
// If the YAML merge key syntax is violated, then this function throws an
// exception.  In the future, it might be nice to use the ReportError helper
// instead of throwing, but that is currently too awkward.
//
// The `parent` is only used to provide context during error reporting.
//
// If yaml-cpp adds native support for merge keys during its own parsing, then
// we should be able to remove this helper.
void RewriteMergeKeys(const YAML::Node& parent, YAML::Node* node) {
  DRAKE_DEMAND(node != nullptr);
  DRAKE_DEMAND(node->Type() == YAML::NodeType::Map);
  const YAML::Node& merge_key = (*node)["<<"];
  if (!merge_key) {
    return;
  }
  (*node).remove("<<");
  const char* error_message = nullptr;
  YAML::Node error_locus;
  switch (merge_key.Type()) {
    case YAML::NodeType::Map: {
      // Merge `merge_key` Map into `node` Map.
      CopyWithMergeKeySemantics(merge_key, node);
      return;
    }
    case YAML::NodeType::Sequence: {
      // Merge each Map in `merge_key` Sequence-of-Maps into the `node` Map.
      for (const YAML::Node& merge_key_item : merge_key) {
        if (merge_key_item.Type() != YAML::NodeType::Map) {
          error_message =
              "has invalid merge key type (Sequence-of-non-Mapping).";
          error_locus = *node;
          break;
        }
        CopyWithMergeKeySemantics(merge_key_item, node);
      }
      if (error_message != nullptr) {
        break;
      }
      return;
    }
    case YAML::NodeType::Scalar: {
      error_message = "has invalid merge key type (Scalar).";
      error_locus = parent;
      break;
    }
    case YAML::NodeType::Null: {
      error_message = "has invalid merge key type (Null).";
      error_locus = parent;
      break;
    }
    case YAML::NodeType::Undefined: {
      // We should never reach here due to the "if (!merge_key)" guard above.
      DRAKE_UNREACHABLE();
    }
  }
  DRAKE_DEMAND(error_message != nullptr);
  DRAKE_DEMAND(error_locus.IsDefined());
  // Sort the keys so that our error message is deterministic.
  std::vector<std::string> keys;
  for (const auto& map_pair : error_locus) {
    keys.push_back(map_pair.first.as<std::string>());
  }
  std::sort(keys.begin(), keys.end());
  throw std::runtime_error(fmt::format(
      "YAML node of type Mapping (with size {} and keys {{{}}}) {}",
      keys.size(), fmt::join(keys, ", "), error_message));
}

// Convert a jbeder/yaml-cpp YAML::Node `node` to a Drake yaml::internal::Node.
// See https://github.com/jbeder/yaml-cpp/wiki/Tutorial for a jbeder reference.
// The `parent` is only used to provide context during error reporting.
internal::Node ConvertJbederYamlNodeToDrakeYamlNode(
    const YAML::Node& parent, const YAML::Node& node) {
  switch (node.Type()) {
    case YAML::NodeType::Undefined: {
      throw std::runtime_error("A yaml-cpp node was unexpectedly Undefined");
    }
    case YAML::NodeType::Null: {
      return internal::Node::MakeNull();
    }
    case YAML::NodeType::Scalar: {
      auto result = internal::Node::MakeScalar(node.Scalar());
      result.SetTag(node.Tag());
      return result;
    }
    case YAML::NodeType::Sequence: {
      auto result = internal::Node::MakeSequence();
      result.SetTag(node.Tag());
      for (size_t i = 0; i < node.size(); ++i) {
        result.Add(ConvertJbederYamlNodeToDrakeYamlNode(node, node[i]));
      }
      return result;
    }
    case YAML::NodeType::Map: {
      YAML::Node merged_node(node);
      RewriteMergeKeys(parent, &merged_node);
      auto result = internal::Node::MakeMapping();
      result.SetTag(merged_node.Tag());
      for (const auto& key_value : merged_node) {
        const YAML::Node& key = key_value.first;
        const YAML::Node& value = key_value.second;
        result.Add(key.Scalar(), ConvertJbederYamlNodeToDrakeYamlNode(
            node, value));
      }
      return result;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

YamlReadArchive::YamlReadArchive(const YAML::Node& root)
    : YamlReadArchive(root, Options{}) {}

YamlReadArchive::YamlReadArchive(const YAML::Node& root, const Options& options)
    : YamlReadArchive(ConvertJbederYamlNodeToDrakeYamlNode({}, root), options) {
}

#pragma GCC diagnostic pop

YamlReadArchive::YamlReadArchive(internal::Node root, const Options& options)
    : owned_root_(std::move(root)),
      root_(&owned_root_.value()),
      mapish_item_key_(nullptr),
      mapish_item_value_(nullptr),
      options_(options),
      parent_(nullptr) {}

// N.B. This is unit tested via yaml_io_test with calls to LoadYamlFile (and
// not as part of yaml_read_archive_test as would be typical).  In the future,
// it might be better to refactor document parsing and merge key handling into
// a separate file with more specific testing, but for the moment our use of
// yaml-cpp in our public API makes that difficult.
internal::Node YamlReadArchive::LoadFileAsNode(
    const std::string& filename,
    const std::optional<std::string>& child_name) {
  YAML::Node root = YAML::LoadFile(filename);
  if (child_name.has_value()) {
    YAML::Node child_node = root[*child_name];
    if (!child_node) {
      throw std::runtime_error(fmt::format(
          "When loading '{}', there was no such top-level map entry '{}'",
          filename, *child_name));
    }
    return ConvertJbederYamlNodeToDrakeYamlNode({}, child_node);
  } else {
    return ConvertJbederYamlNodeToDrakeYamlNode({}, root);
  }
}

// N.B. This is unit tested via yaml_io_test with calls to LoadYamlString (and
// not as part of yaml_read_archive_test as would be typical).  In the future,
// it might be better to refactor document parsing and merge key handling into
// a separate file with more specific testing, but for the moment our use of
// yaml-cpp in our public API makes that difficult.
internal::Node YamlReadArchive::LoadStringAsNode(
    const std::string& data,
    const std::optional<std::string>& child_name) {
  YAML::Node root = YAML::Load(data);
  if (child_name.has_value()) {
    YAML::Node child_node = root[*child_name];
    if (!child_node) {
      throw std::runtime_error(fmt::format(
          "When loading YAML, there was no such top-level map entry '{}'",
          *child_name));
    }
    return ConvertJbederYamlNodeToDrakeYamlNode({}, child_node);
  } else {
    return ConvertJbederYamlNodeToDrakeYamlNode({}, root);
  }
}

// TODO(jwnimmer-tri) On 2022-03-01 when the deprecated YAML::Node functions are
// removed, the header file implementation that calls `convert<>` should move
// into the cc file here, as an anonymous helper function.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

void YamlReadArchive::ParseScalar(const std::string& value, bool* result) {
  ParseScalar<bool>(value, result);
}

void YamlReadArchive::ParseScalar(const std::string& value, float* result) {
  ParseScalar<float>(value, result);
}

void YamlReadArchive::ParseScalar(const std::string& value, double* result) {
  ParseScalar<double>(value, result);
}

void YamlReadArchive::ParseScalar(const std::string& value, int32_t* result) {
  ParseScalar<int32_t>(value, result);
}

void YamlReadArchive::ParseScalar(const std::string& value, uint32_t* result) {
  ParseScalar<uint32_t>(value, result);
}

void YamlReadArchive::ParseScalar(const std::string& value, int64_t* result) {
  ParseScalar<int64_t>(value, result);
}

void YamlReadArchive::ParseScalar(const std::string& value, uint64_t* result) {
  ParseScalar<uint64_t>(value, result);
}

#pragma GCC diagnostic pop

void YamlReadArchive::ParseScalar(
    const std::string& value, std::string* result) {
  DRAKE_DEMAND(result != nullptr);
  *result = value;
}

const internal::Node* YamlReadArchive::MaybeGetSubNode(const char* name) const {
  DRAKE_DEMAND(name != nullptr);
  if (mapish_item_key_ != nullptr) {
    DRAKE_DEMAND(mapish_item_value_ != nullptr);
    if (std::strcmp(mapish_item_key_, name) == 0) {
      return mapish_item_value_;
    }
    return nullptr;
  }
  DRAKE_DEMAND(root_ != nullptr);
  DRAKE_DEMAND(root_->IsMapping());
  const auto& map = root_->GetMapping();
  auto iter = map.find(name);
  if (iter == map.end()) {
    return nullptr;
  }
  return &(iter->second);
}

const internal::Node* YamlReadArchive::GetSubNodeScalar(
    const char* name) const {
  const internal::Node* result = GetSubNodeAny(
      name, internal::NodeType::kScalar);
  if ((result != nullptr) && (result->GetTag() == internal::Node::kTagNull)) {
    ReportError("has non-Scalar (Null)");
    result = nullptr;
  }
  return result;
}

const internal::Node* YamlReadArchive::GetSubNodeSequence(
    const char* name) const {
  return GetSubNodeAny(name, internal::NodeType::kSequence);
}

const internal::Node* YamlReadArchive::GetSubNodeMapping(
    const char* name) const {
  return GetSubNodeAny(name, internal::NodeType::kMapping);
}

const internal::Node* YamlReadArchive::GetSubNodeAny(
    const char* name, internal::NodeType expected_type) const {
  const internal::Node* result = MaybeGetSubNode(name);
  if (result == nullptr) {
    if (!options_.allow_cpp_with_no_yaml) {
      ReportError("is missing");
    }
    return result;
  }
  internal::NodeType actual_type = result->GetType();
  if (actual_type != expected_type) {
    std::string_view expected_type_string =
        internal::Node::GetTypeString(expected_type);
    std::string_view actual_type_string = result->GetTypeString();
    if (result->GetTag() == internal::Node::kTagNull) {
      actual_type_string = "Null";
    }
    ReportError(fmt::format(
        "has non-{} ({})", expected_type_string, actual_type_string));
    result = nullptr;
  }
  return result;
}

void YamlReadArchive::CheckAllAccepted() const {
  // This function is only ever called on Serializeable nodes (i.e., where we
  // have a real Mapping node).  Calling it with a map-ish key (i.e., while
  // parsing a sequence) would mean that YamlReadArchive went off the rails.
  DRAKE_DEMAND(mapish_item_key_ == nullptr);
  DRAKE_DEMAND(root_->IsMapping());
  if (options_.allow_yaml_with_no_cpp) {
    return;
  }
  for (const auto& [key, value] : root_->GetMapping()) {
    unused(value);
    if (visited_names_.count(key) == 0) {
      ReportError(fmt::format(
          "key {} did not match any visited value", key));
    }
  }
}

void YamlReadArchive::ReportError(const std::string& note) const {
  std::ostringstream e;  // A buffer for the error message text.
  this->PrintNodeSummary(e);
  fmt::print(e, " {} entry for ", note);
  PrintVisitNameType(e);
  for (auto* archive = parent_; archive; archive = archive->parent_) {
    fmt::print(e, " while accepting ");
    archive->PrintNodeSummary(e);
    if (archive->debug_visit_name_ != nullptr) {
      fmt::print(e, " while visiting ");
      archive->PrintVisitNameType(e);
    }
  }
  fmt::print(e, ".");
  throw std::runtime_error(e.str());
}

void YamlReadArchive::PrintNodeSummary(std::ostream& s) const {
  if (mapish_item_key_ != nullptr) {
    fmt::print(s, " (with size 1 and keys {{{}}})", mapish_item_key_);
    return;
  }

  DRAKE_DEMAND(root_ != nullptr);
  fmt::print(s, "YAML node of type {}", root_->GetTypeString());
  if (!root_->IsMapping()) {
    // Don't log any additional details for non-Mappings.
    return;
  }

  // Grab the mapping's keys.  (It's a std::map, so the ordering here is
  // fully deterministic.)
  std::vector<std::string_view> keys;
  for (const auto& [key, value] : root_->GetMapping()) {
    unused(value);
    keys.push_back(key);
  }

  // Output the details of the keys.
  fmt::print(s, " (with size {} and keys {{{}}})",
             keys.size(), fmt::join(keys, ", "));
}

void YamlReadArchive::PrintVisitNameType(std::ostream& s) const {
  if (debug_visit_name_ == nullptr) {
    s << "<root>";
    return;
  }
  DRAKE_DEMAND(debug_visit_name_ != nullptr);
  DRAKE_DEMAND(debug_visit_type_ != nullptr);
  fmt::print(s, "{} {}",
             drake::NiceTypeName::Get(*debug_visit_type_),
             debug_visit_name_);
}

std::ostream& operator<<(std::ostream& os, const YamlReadArchive::Options& x) {
  return os << "{.allow_yaml_with_no_cpp = "
            << x.allow_yaml_with_no_cpp
            << ", .allow_cpp_with_no_yaml = "
            << x.allow_cpp_with_no_yaml
            << ", .retain_map_defaults = "
            << x.retain_map_defaults << "}";
}

}  // namespace yaml
}  // namespace drake
