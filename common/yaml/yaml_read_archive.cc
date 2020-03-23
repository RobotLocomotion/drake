#include "drake/common/yaml/yaml_read_archive.h"

#include <algorithm>
#include <cstring>

#include <fmt/ostream.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace yaml {
namespace {

// Add the `to_add` merge key value into the root using the merge key
// semantics; see https://yaml.org/type/merge.html for details.
// If yaml-cpp adds native support for merge keys, then we should can
// remove this helper.
void AddMergeKeys(const YAML::Node& to_add, YAML::Node* root) {
  DRAKE_DEMAND(root != nullptr);
  switch (to_add.Type()) {
    case YAML::NodeType::Map: {
      // Merge `to_all` map into `root` map.
      for (const auto& key_value : to_add) {
        const YAML::Node& key = key_value.first;
        const YAML::Node& value = key_value.second;
        YAML::Node entry = (*root)[key.Scalar()];
        // An existing value takes precedence over new values.
        if (!entry) {
          entry = value;
        }
      }
      return;
    }
    case YAML::NodeType::Sequence: {
      // Merge each map in `to_all` into the `root` map.
      for (const YAML::Node& child : to_add) {
        if (child.Type() != YAML::NodeType::Map) {
          throw std::runtime_error(fmt::format(
              "YamlReadArchive found unexpected merge key value of type {} "
              "while processing {}", child.Type(), to_add));
        }
        AddMergeKeys(child, root);
      }
      return;
    }
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
    case YAML::NodeType::Scalar: {
      throw std::runtime_error(fmt::format(
          "YamlReadArchive found unexpected merge key value of type {} "
          "while processing {}", to_add.Type(), to_add));
    }
  }
  DRAKE_UNREACHABLE();
}

// Computes `owned_root_` for our constructor's member initializer list.
YAML::Node ConstructOwnedRoot(
    const bool should_copy_root,
    const YAML::Node* const root) {
  if (root != nullptr) {
    // We're in the "root" constructor case (not "mapish").
    // Check if we have a merge key.
    const YAML::Node& merge_key = (*root)["<<"];
    if (!merge_key) {
      // No merge, so just obey the should_copy_root request.
      return should_copy_root ? *root : YAML::Node{YAML::NodeType::Undefined};
    } else {
      // Handle merge keys.
      YAML::Node merged_root(*root);
      merged_root.remove("<<");
      AddMergeKeys(merge_key, &merged_root);
      return merged_root;
    }
  } else {
    // We're in the "mapish" constructor case (not "root").
    DRAKE_DEMAND(!should_copy_root);
    return YAML::Node{YAML::NodeType::Undefined};
  }
}

}  // namespace

YamlReadArchive::YamlReadArchive(
    bool should_copy_root,
    const YAML::Node* root,
    const char* mapish_item_key,
    const YAML::Node* mapish_item_value,
    const YamlReadArchive* parent)
    : owned_root_(ConstructOwnedRoot(should_copy_root, root)),
      root_(owned_root_ ? &owned_root_ : root),
      mapish_item_key_(mapish_item_key),
      mapish_item_value_(mapish_item_value),
      parent_(parent) {
  // We should have exactly one of root_ or both mapish_item_.
  DRAKE_DEMAND(!!root_ ^ !!mapish_item_key_);
  DRAKE_DEMAND(!!mapish_item_key_ == !!mapish_item_value_);
}

bool YamlReadArchive::has_root() const {
  if (mapish_item_key_ != nullptr) {
    return true;
  }
  DRAKE_DEMAND(root_ != nullptr);
  return !(root_->IsNull());
}

YAML::Node YamlReadArchive::MaybeGetSubNode(const char* name) const {
  DRAKE_DEMAND(name != nullptr);
  if (mapish_item_key_ != nullptr) {
    DRAKE_DEMAND(mapish_item_value_ != nullptr);
    if (std::strcmp(mapish_item_key_, name) == 0) {
      return *mapish_item_value_;
    }
    return {};
  }
  DRAKE_DEMAND(root_ != nullptr);
  return (*root_)[name];
}

YAML::Node YamlReadArchive::GetSubNode(
    const char* name, YAML::NodeType::value expected_type) const {
  const YAML::Node result = MaybeGetSubNode(name);
  if (!result) {
    ReportMissingYaml("is missing");
    return {};
  }
  const auto& actual_type = result.Type();
  if (actual_type != expected_type) {
    ReportMissingYaml(fmt::format(
        "has non-{} ({})", to_string(expected_type), to_string(actual_type)));
    return {};
  }
  return result;
}

void YamlReadArchive::ReportMissingYaml(const std::string& note) const {
  std::ostringstream e;  // A buffer for the error message text.
  this->PrintNodeSummary(e);
  fmt::print(e, " {} entry for ", note);
  PrintVisitNameType(e);
  for (auto* archive = parent_; archive; archive = archive->parent_) {
    fmt::print(e, " while accepting ");
    archive->PrintNodeSummary(e);
    if (archive->debug_visit_name_) {
      fmt::print(e, " while visiting ");
      archive->PrintVisitNameType(e);
    }
  }
  fmt::print(e, ".");
  throw std::runtime_error(e.str());
}

void YamlReadArchive::PrintNodeSummary(std::ostream& s) const {
  YAML::Node to_print;
  if (mapish_item_key_ != nullptr) {
    DRAKE_DEMAND(mapish_item_value_ != nullptr);
    to_print[mapish_item_key_] = *mapish_item_value_;
  } else {
    DRAKE_DEMAND(root_ != nullptr);
    to_print = *root_;
  }
  fmt::print(s, "YAML node of type {}", to_string(to_print.Type()));
  switch (to_print.Type()) {
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
    case YAML::NodeType::Scalar:
    case YAML::NodeType::Sequence: {
      // Don't log any details of these.
      break;
    }
    case YAML::NodeType::Map: {
      const size_t size = to_print.size();
      fmt::print(s, " (with size {} and keys {{", size);
      // Sort the keys so that our error message is deterministic.
      std::vector<std::string> keys;
      keys.reserve(size);
      for (const auto& map_pair : to_print) {
        keys.emplace_back(map_pair.first.as<std::string>());
      }
      std::sort(keys.begin(), keys.end());
      DRAKE_DEMAND(keys.size() == size);
      for (size_t i = 0; i < size; ++i) {
        if (i == 0) {
          fmt::print(s, "{}", keys[i]);
        } else {
          fmt::print(s, ", {}", keys[i]);
        }
      }
      fmt::print(s, "}})");
      break;
    }
  }
}

void YamlReadArchive::PrintVisitNameType(std::ostream& s) const {
  DRAKE_DEMAND(debug_visit_name_);
  DRAKE_DEMAND(debug_visit_type_);
  fmt::print(s, "{} {}",
             drake::NiceTypeName::Get(*debug_visit_type_),
             debug_visit_name_);
}

const char* YamlReadArchive::to_string(YAML::NodeType::value x) {
  switch (x) {
    case YAML::NodeType::Undefined: return "Undefined";
    case YAML::NodeType::Null: return "Null";
    case YAML::NodeType::Scalar: return "Scalar";
    case YAML::NodeType::Sequence: return "Sequence";
    case YAML::NodeType::Map: return "Map";
  }
  return "UNKNOWN";
}

}  // namespace yaml
}  // namespace drake
