#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {
namespace render {

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
