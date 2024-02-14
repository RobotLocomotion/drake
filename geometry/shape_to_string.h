#pragma once

#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

class DRAKE_DEPRECATED("2024-06-01",
                       "Use the Shape::to_string() member function instead")
    ShapeToString final : public ShapeReifier {
 public:
  ~ShapeToString() final;

  const std::string& string() const { return string_; }

 private:
  void DefaultImplementGeometry(const Shape& shape) final;

  std::string string_;
};

}  // namespace geometry
}  // namespace drake
