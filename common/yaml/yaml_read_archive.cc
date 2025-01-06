#include "drake/common/yaml/yaml_read_archive.h"

#include <algorithm>
#include <cstring>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <yaml-cpp/yaml.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace yaml {
namespace internal {
namespace {

// The source and destination are both of type Map.  Copy the key-value pairs
// from source into destination, but don't overwrite any existing keys.
void CopyWithMergeKeySemantics(const YAML::Node& source,
                               YAML::Node* destination) {
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
  throw std::runtime_error(
      fmt::format("YAML node of type Mapping (with size {} and keys {{{}}}) {}",
                  keys.size(), fmt::join(keys, ", "), error_message));
}

// Convert a jbeder/yaml-cpp YAML::Node `node` to a Drake yaml::internal::Node.
// See https://github.com/jbeder/yaml-cpp/wiki/Tutorial for a jbeder reference.
// The `parent` is only used to provide context during error reporting.
internal::Node ConvertJbederYamlNodeToDrakeYamlNode(const YAML::Node& parent,
                                                    const YAML::Node& node) {
  std::optional<Node::Mark> mark;
  if (node.Mark().line >= 0 && node.Mark().column >= 0) {
    // The jbeder convention is 0-based numbering; we want 1-based.
    mark = Node::Mark{.line = node.Mark().line + 1,
                      .column = node.Mark().column + 1};
  }
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
      result.SetMark(mark);
      return result;
    }
    case YAML::NodeType::Sequence: {
      auto result = internal::Node::MakeSequence();
      result.SetTag(node.Tag());
      result.SetMark(mark);
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
      result.SetMark(mark);
      for (const auto& key_value : merged_node) {
        const YAML::Node& key = key_value.first;
        const YAML::Node& value = key_value.second;
        result.Add(key.Scalar(),
                   ConvertJbederYamlNodeToDrakeYamlNode(node, value));
      }
      return result;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

YamlReadArchive::YamlReadArchive(internal::Node root,
                                 const LoadYamlOptions& options)
    : owned_root_(std::move(root)),
      root_(&owned_root_.value()),
      mapish_item_key_(nullptr),
      mapish_item_value_(nullptr),
      options_(options),
      parent_(nullptr) {
  if (!root_->IsMapping()) {
    throw std::runtime_error(fmt::format(
        "{}: invalid document: the top level element should be a Mapping "
        "(not a {})",
        root_->GetFilename().value_or("<string>"), root_->GetTypeString()));
  }
}

// N.B. This is unit tested via yaml_io_test with calls to LoadYamlFile (and
// not as part of yaml_read_archive_test as would be typical).  In the future,
// it might be better to refactor document parsing and merge key handling into
// a separate file with more specific testing, but for the moment our use of
// yaml-cpp in our public API makes that difficult.
internal::Node YamlReadArchive::LoadFileAsNode(
    const std::string& filename, const std::optional<std::string>& child_name) {
  internal::Node result = internal::Node::MakeNull();
  YAML::Node root = YAML::LoadFile(filename);
  if (child_name.has_value()) {
    YAML::Node child_node = root[*child_name];
    if (!child_node) {
      throw std::runtime_error(fmt::format(
          "When loading '{}', there was no such top-level map entry '{}'",
          filename, *child_name));
    }
    result = ConvertJbederYamlNodeToDrakeYamlNode({}, child_node);
  } else {
    result = ConvertJbederYamlNodeToDrakeYamlNode({}, root);
  }
  result.SetFilename(filename);
  return result;
}

// N.B. This is unit tested via yaml_io_test with calls to LoadYamlString (and
// not as part of yaml_read_archive_test as would be typical).  In the future,
// it might be better to refactor document parsing and merge key handling into
// a separate file with more specific testing, but for the moment our use of
// yaml-cpp in our public API makes that difficult.
internal::Node YamlReadArchive::LoadStringAsNode(
    const std::string& data, const std::optional<std::string>& child_name) {
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

template <typename T>
void YamlReadArchive::ParseScalarImpl(const internal::Node& scalar, T* result) {
  DRAKE_DEMAND(result != nullptr);

  // Detect one specific case of tag mismatch. Ideally we should detect any kind
  // of tag incompatiblity, but for now we'll just check for this one.
  if (scalar.GetTag() == internal::Node::kTagBinary) {
    ReportError(
        fmt::format("has incompatible !!binary tag in yaml document for",
                    drake::NiceTypeName::Get<T>()));
    return;
  }

  if constexpr (std::is_same_v<T, std::string>) {
    *result = scalar.GetScalar();
    return;
  }

  // For the decode-able types, see /usr/include/yaml-cpp/node/convert.h.
  // Generally, all of the POD types are supported.
  const std::string& value = scalar.GetScalar();
  bool success = YAML::convert<T>::decode(YAML::Node(value), *result);
  if (!success) {
    ReportError(
        fmt::format("could not parse {} value", drake::NiceTypeName::Get<T>()));
  }
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar, bool* result) {
  ParseScalarImpl<bool>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar, float* result) {
  ParseScalarImpl<float>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  double* result) {
  ParseScalarImpl<double>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  int32_t* result) {
  ParseScalarImpl<int32_t>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  uint32_t* result) {
  ParseScalarImpl<uint32_t>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  int64_t* result) {
  ParseScalarImpl<int64_t>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  uint64_t* result) {
  ParseScalarImpl<uint64_t>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  std::string* result) {
  ParseScalarImpl<std::string>(scalar, result);
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  std::filesystem::path* result) {
  DRAKE_DEMAND(result != nullptr);
  std::string string_value;
  ParseScalarImpl<std::string>(scalar, &string_value);
  // Python deserialization normalizes paths (i.e., a//b becomes a/b). We'll
  // mirror that behavior for consistency's sake.
  *result = std::filesystem::path(std::move(string_value)).lexically_normal();
}

void YamlReadArchive::ParseScalar(const internal::Node& scalar,
                                  std::vector<std::byte>* result) {
  DRAKE_DEMAND(result != nullptr);

  if (scalar.GetTag() != internal::Node::kTagBinary) {
    ReportError(fmt::format(
        "must be base64 encoded with the !!binary tag (not {}) in the",
        scalar.GetTag()));
    return;
  }

  const std::string& encoded = scalar.GetScalar();
  std::vector<unsigned char> chars = YAML::DecodeBase64(encoded);
  std::string decoded(chars.begin(), chars.end());
  if (decoded.empty() != encoded.empty()) {
    // Decoded should only be empty if the input is empty. This is a good but
    // imperfect test. If `encoded` were nothing but whitespace, it would
    // *functionally* be an empty base64 string and *should* produce an empty
    // result. If necessary, we can attempt stripping whitespace from encoded.

    // Grab the leading snippet of encoded text for the error message.
    const std::string_view head = std::string_view{encoded}.substr(0, 25);
    ReportError(fmt::format("contains invalid base64 text '{}{}'", head,
                            head.size() < encoded.size() ? "..." : ""));
    return;
  }

  const std::byte* data = reinterpret_cast<const std::byte*>(decoded.data());
  *result = std::vector<std::byte>(data, data + decoded.size());
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
  const internal::Node* result =
      GetSubNodeAny(name, internal::NodeType::kScalar);
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
    ReportError(fmt::format("has non-{} ({})", expected_type_string,
                            actual_type_string));
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
    if (!visited_names_.contains(key)) {
      ReportError(fmt::format("key '{}' did not match any visited value", key));
    }
  }
}

void YamlReadArchive::ReportError(const std::string& note) const {
  std::ostringstream e;  // A buffer for the error message text.
  // Output the filename.
  bool found_filename = false;
  for (auto* archive = this; archive != nullptr; archive = archive->parent_) {
    if ((archive->root_ != nullptr) &&
        (archive->root_->GetFilename().has_value())) {
      const std::string& filename = archive->root_->GetFilename().value();
      fmt::print(e, "{}:", filename);
      found_filename = true;
      break;
    }
  }
  if (!found_filename) {
    e << "<string>:";
  }
  // Output the nearby line and column number. It's usually the mark for `this`
  // but for a "mapish item" can a nearby ancestor.
  for (auto* archive = this; archive != nullptr; archive = archive->parent_) {
    if (archive->root_ != nullptr) {
      if (archive->root_->GetMark().has_value()) {
        const Node::Mark& mark = archive->root_->GetMark().value();
        fmt::print(e, "{}:{}:", mark.line, mark.column);
      }
      break;
    }
  }
  e << " ";
  // Describe this node.
  this->PrintNodeSummary(e);
  fmt::print(e, " {} entry for ", note);
  PrintVisitNameType(e);
  // Describe its parents.
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
  // Grab the mapping's keys.  (It's a std::map, so the ordering here is
  // fully deterministic.)
  DRAKE_DEMAND(root_->IsMapping());
  std::vector<std::string_view> keys;
  for (const auto& [key, value] : root_->GetMapping()) {
    unused(value);
    keys.push_back(key);
  }

  // Output the details of the keys.
  fmt::print(s, " (with size {} and keys {{{}}})", keys.size(),
             fmt::join(keys, ", "));
}

void YamlReadArchive::PrintVisitNameType(std::ostream& s) const {
  if (debug_visit_name_ == nullptr) {
    s << "<root>";
    return;
  }
  DRAKE_DEMAND(debug_visit_name_ != nullptr);
  DRAKE_DEMAND(debug_visit_type_ != nullptr);
  fmt::print(s, "{} {}", drake::NiceTypeName::Get(*debug_visit_type_),
             debug_visit_name_);
}

}  // namespace internal
}  // namespace yaml
}  // namespace drake
