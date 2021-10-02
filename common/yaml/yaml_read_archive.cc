#include "drake/common/yaml/yaml_read_archive.h"

#include <algorithm>
#include <cstring>

#include <fmt/ostream.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace yaml {
namespace {

// Boilerplate for std::visit.
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// The source and destination are both of type Map.  Copy the key-value pairs
// from source into destination, but don't overwrite any existing keys.
void CopyMergeKeys(const YAML::Node& source, YAML::Node* destination) {
  for (const auto& key_value : source) {
    const YAML::Node& key = key_value.first;
    const YAML::Node& value = key_value.second;
    YAML::Node entry = (*destination)[key.Scalar()];
    if (!entry) {
      entry = value;
    }
  }
}

// Move the merge key values (if any) into the given node using the merge key
// semantics; see https://yaml.org/type/merge.html for details.  If yaml-cpp
// adds native support for merge keys, then we should remove this helper.
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
      CopyMergeKeys(merge_key, node);
      return;
    }
    case YAML::NodeType::Sequence: {
      // Merge each Map in `merge_key` Sequence-of-Maps into the `node` Map.
      for (const YAML::Node& merge_key_item : merge_key) {
        if (merge_key_item.Type() != YAML::NodeType::Map) {
          error_message = "has invalid merge key type (Sequence-of-non-Map).";
          error_locus = *node;
          break;
        }
        CopyMergeKeys(merge_key_item, node);
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
      "YAML node of type Map (with size {} and keys {{{}}}) {}",
      keys.size(), fmt::join(keys, ", "), error_message));
}

internal::Node TransmogrifyNode(
    const YAML::Node& parent, const YAML::Node& node) {
  switch (node.Type()) {
    case YAML::NodeType::Undefined: {
      throw std::runtime_error("TransmogrifyNode on Undefined");
    }
    case YAML::NodeType::Null: {
      return internal::Node();
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
        result.Add(TransmogrifyNode(node, node[i]));
      }
      return result;
    }
    case YAML::NodeType::Map: {
      YAML::Node merged_node(node);
      RewriteMergeKeys(parent, &merged_node);
      auto result = internal::Node::MakeMap();
      result.SetTag(merged_node.Tag());
      for (const auto& key_value : merged_node) {
        const YAML::Node& key = key_value.first;
        const YAML::Node& value = key_value.second;
        result.Add(key.Scalar(), TransmogrifyNode(node, value));
      }
      return result;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

YamlReadArchive::YamlReadArchive(const YAML::Node& root)
    : YamlReadArchive(root, Options{}) {}

YamlReadArchive::YamlReadArchive(const YAML::Node& root, const Options& options)
    : owned_root_(TransmogrifyNode({}, root)),
      root_(&owned_root_),
      mapish_item_key_(nullptr),
      mapish_item_value_(nullptr),
      options_(options),
      parent_(nullptr) {}

void YamlReadArchive::ParseScalar(
    const std::string& value, std::string* result) {
  DRAKE_DEMAND(result != nullptr);
  *result = value;
}

void YamlReadArchive::ParseScalar(
    const std::string& value, double* result) {
  DRAKE_DEMAND(result != nullptr);
  YAML::Node node(value);
  bool success = YAML::convert<double>::decode(node, *result);
  if (!success) {
    ReportError("could not parse double value");
  }
}

void YamlReadArchive::ParseScalar(
    const std::string& value, bool* result) {
  DRAKE_DEMAND(result != nullptr);
  YAML::Node node(value);
  bool success = YAML::convert<bool>::decode(node, *result);
  if (!success) {
    ReportError("could not parse bool value");
  }
}

bool YamlReadArchive::has_root() const {
  if (mapish_item_key_ != nullptr) {
    return true;
  }
  DRAKE_DEMAND(root_ != nullptr);
  return root_->IsMap();
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
  DRAKE_DEMAND(root_->IsMap());
  const auto& map = root_->GetMap();
  auto iter = map.find(name);
  if (iter == map.end()) {
    return nullptr;
  }
  return &(iter->second);
}

const internal::Node* YamlReadArchive::GetSubNodeScalar(
    const char* name) const {
  const internal::Node* result = DoGetSubNode(name, "Scalar");
  if (result != nullptr) {
    if (result->IsEmptyScalar()) {
      ReportError("has non-Scalar (Null)");
    }
  }
  return result;
}

const internal::Node* YamlReadArchive::GetSubNodeSequence(
    const char* name) const {
  return DoGetSubNode(name, "Sequence");
}

const internal::Node* YamlReadArchive::GetSubNodeMap(
    const char* name) const {
  return DoGetSubNode(name, "Map");
}

const internal::Node* YamlReadArchive::DoGetSubNode(
    const char* name, std::string_view expected_type) const {
  const internal::Node* result = MaybeGetSubNode(name);
  if (result == nullptr) {
    if (!options_.allow_cpp_with_no_yaml) {
      ReportError("is missing");
    }
    return result;
  }
  std::string_view actual_type = result->GetTypeString();
  if (actual_type != expected_type) {
    if (result->IsEmptyScalar()) {
      actual_type = "Null";
    }
    ReportError(fmt::format("has non-{} ({})", expected_type, actual_type));
    result = nullptr;
  }
  return result;
}

void YamlReadArchive::CheckAllAccepted() const {
  DRAKE_DEMAND(mapish_item_key_ == nullptr);
  DRAKE_DEMAND(root_->IsMap());
  if (options_.allow_yaml_with_no_cpp) {
    return;
  }
  for (const auto& [key, value] : root_->GetMap()) {
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
  if (!root_->IsMap()) {
    // Don't log any additional details for non-Maps.
    return;
  }

  // Sort the keys so that our error message is deterministic.
  std::vector<std::string> keys;
  for (const auto& [key, value] : root_->GetMap()) {
    unused(value);
    keys.push_back(key);
  }
  std::sort(keys.begin(), keys.end());

  // Output the details of the keys.
  const size_t size = keys.size();
  fmt::print(s, " (with size {} and keys {{", size);
  for (size_t i = 0; i < size; ++i) {
    if (i == 0) {
      fmt::print(s, "{}", keys[i]);
    } else {
      fmt::print(s, ", {}", keys[i]);
    }
  }
  fmt::print(s, "}})");
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
