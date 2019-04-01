#include "drake/geometry/render/render_label_manager.h"

#include <fmt/format.h>

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
    : maximum_value_(RenderLabel::kMaxUnreserved) {
  // The reserved labels all belong to the provided source id.
  name_label_map_["empty"].push_back(
      RenderLabelClass("empty", id, RenderLabel::kEmpty));
  name_label_map_["do_not_render"].push_back(
      RenderLabelClass("do_not_render", id, RenderLabel::kDoNotRender));
  name_label_map_["dont_care"].push_back(
      RenderLabelClass("dont_care", id, RenderLabel::kDontCare));
  name_label_map_["unspecified"].push_back(
      RenderLabelClass("unspecified", id, RenderLabel::kUnspecified));
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
    throw std::logic_error(fmt::format(
        "All {} render labels have been allocated", maximum_value_));
  }
}

vector<RenderLabelClass> RenderLabelManager::GetRenderLabelClasses() const {
  vector<RenderLabelClass> classes;
  for (const auto& multi_pair : name_label_map_) {
    const vector<RenderLabelClass> named_classes = multi_pair.second;
    classes.insert(classes.end(), named_classes.begin(), named_classes.end());
  }
  return classes;
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
