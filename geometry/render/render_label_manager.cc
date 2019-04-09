#include "drake/geometry/render/render_label_manager.h"

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {
namespace internal {

using std::move;
using std::pair;
using std::string;
using std::unordered_multimap;
using std::vector;

RenderLabelManager::RenderLabelManager(SourceId id)
    : maximum_value_(RenderLabel::kMaxUnreserved) {
  // The reserved labels all belong to the provided source id.
  name_label_map_.insert(
      {"empty", RenderLabelClass("empty", id, RenderLabel::kEmpty)});
  name_label_map_.insert(
      {"do_not_render",
       RenderLabelClass("do_not_render", id, RenderLabel::kDoNotRender)});
  name_label_map_.insert(
      {"dont_care", RenderLabelClass("dont_care", id, RenderLabel::kDontCare)});
  name_label_map_.insert(
      {"unspecified",
       RenderLabelClass("unspecified", id, RenderLabel::kUnspecified)});
}

RenderLabel RenderLabelManager::GetRenderLabel(SourceId source_id,
                                               string class_name) {
  // Check to see if this name has been used by the source id already.
  auto range = name_label_map_.equal_range(class_name);
  for (auto it = range.first; it != range.second; ++it) {
    if (it->second.source_id == source_id) return it->second.label;
  }

  // It hasn't been used; create a label and record usage.
  if (next_value_ <= maximum_value_) {
    RenderLabel label(next_value_++);
    name_label_map_.insert(
        {class_name, RenderLabelClass(class_name, source_id, label)});
    return label;
  } else {
    throw std::logic_error(fmt::format(
        "All {} render labels have been allocated", maximum_value_));
  }
}

vector<RenderLabelClass> RenderLabelManager::GetRenderLabelClasses() const {
  vector<RenderLabelClass> classes;
  for (auto it = name_label_map_.begin(); it != name_label_map_.end(); ++it) {
    const RenderLabelClass& label_class = it->second;
    classes.push_back(label_class);
  }
  return classes;
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
