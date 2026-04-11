#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {

// Note: relies on the default constructor to define the unspecified value.
const RenderLabel RenderLabel::kUnspecified;
const RenderLabel RenderLabel::kEmpty(kUnspecified.value_ - 1, false);
const RenderLabel RenderLabel::kDoNotRender(kUnspecified.value_ - 2, false);
const RenderLabel RenderLabel::kDontCare(kUnspecified.value_ - 3, false);
const RenderLabel::ValueType RenderLabel::kMaxUnreserved(kUnspecified.value_ -
                                                         4);

std::string RenderLabel::to_string() const {
  return std::to_string(value_);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
