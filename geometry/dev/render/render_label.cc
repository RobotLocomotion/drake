#include "drake/geometry/dev/render/render_label.h"

namespace drake {
namespace geometry {
namespace dev {
namespace render {

std::ostream& operator<<(std::ostream& out, const RenderLabel& label) {
  out << label.value_;
  return out;
}

std::string to_string(const RenderLabel& label) {
  return std::to_string(label.value_);
}

}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
