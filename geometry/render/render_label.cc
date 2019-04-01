#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {

constexpr RenderLabel::ValueType max_value =
    std::numeric_limits<RenderLabel::ValueType>::max();

const RenderLabel RenderLabel::kUnspecified(max_value, false);
const RenderLabel RenderLabel::kEmpty(max_value - 1, false);
const RenderLabel RenderLabel::kDoNotRender(max_value - 2, false);
const RenderLabel RenderLabel::kDontCare(max_value - 3, false);
const RenderLabel::ValueType RenderLabel::kMaxUnreserved = max_value - 4;

std::ostream& operator<<(std::ostream& out, const RenderLabel& label) {
  out << label.value_;
  return out;
}

std::string to_string(const RenderLabel& label) {
  return std::to_string(label.value_);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
