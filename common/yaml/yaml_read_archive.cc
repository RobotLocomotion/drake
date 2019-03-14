#include "drake/common/yaml/yaml_read_archive.h"

#include <algorithm>

#include <fmt/ostream.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace yaml {

YAML::Node YamlReadArchive::GetSubNode(
    const char* name, YAML::NodeType::value expected_type) const {
  const YAML::Node result = root_[name];
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
  fmt::print(s, "YAML node of type {}", to_string(root_.Type()));
  switch (root_.Type()) {
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
    case YAML::NodeType::Scalar:
    case YAML::NodeType::Sequence: {
      // Don't log any details of these.
      break;
    }
    case YAML::NodeType::Map: {
      const size_t size = root_.size();
      fmt::print(s, " (with size {} and keys {{", size);
      // Sort the keys so that our error message is deterministic.
      std::vector<std::string> keys;
      keys.reserve(size);
      for (const auto& map_pair : root_) {
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
