#include "drake/geometry/render/render_label_manager.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

using std::move;
using std::pair;
using std::string;
using std::unordered_map;
using std::vector;

RenderLabelManager::RenderLabelManager(SourceId id)
    : maximum_value_(RenderLabel::maximum_unreserved()) {
  // The reserved labels all belong to the source that owns the manager.
  name_label_map_["empty"].push_back(
      RenderLabelClass("empty", id, RenderLabel::empty_label()));
  name_label_map_["do_not_render"].push_back(RenderLabelClass(
      "do_not_render", id, RenderLabel::do_not_render_label()));
  name_label_map_["dont_care"].push_back(
      RenderLabelClass("dont_care", id, RenderLabel::dont_care_label()));
  name_label_map_["unspecified"].push_back(
      RenderLabelClass("unspecified", id, RenderLabel::unspecified_label()));
}

RenderLabel RenderLabelManager::GetRenderLabel(SourceId source_id,
                                               string class_name) {
  // Check to see if this name has been used by the source id already.
  if (name_label_map_.count(class_name) > 0) {
    for (const auto& label_class : name_label_map_[class_name]) {
      if (label_class.source_id == source_id) return label_class.label;
    }
  }

  // It hasn't been used; create a label and record usage.
  if (next_value_ <= maximum_value_) {
    RenderLabel label(next_value_++);
    name_label_map_[class_name].push_back(
        RenderLabelClass(class_name, source_id, label));
    return label;
  } else {
    throw std::logic_error("All render labels have been allocated");
  }
}

unordered_map<string, vector<pair<string, RenderLabel>>>
RenderLabelManager::GetLabelsByName(
    const unordered_map<SourceId, string>& source_names) const {
  unordered_map<string, vector<pair<string, RenderLabel>>> data;
  for (const auto& pair : name_label_map_) {
    const string& label_name = pair.first;
    for (const auto label_class : pair.second) {
      const std::string& source = source_names.at(label_class.source_id);
      data[label_name].emplace_back(source, label_class.label);
    }
  }

  return data;
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
