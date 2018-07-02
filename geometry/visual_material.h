#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** Definition of material for simple visualization. Default materials are a
 light grey. */
class VisualMaterial final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VisualMaterial)

  /** Constructs a material with the default grey color. */
  VisualMaterial();

  /** Constructs a material with the given diffuse color.
   @param diffuse  A vector of [r, g, b, a] values, each in the range [0, 1]. */
  explicit VisualMaterial(const Eigen::Vector4d& diffuse);

  /** Returns the material's diffuse color. */
  const Eigen::Vector4d& diffuse() const { return diffuse_; }

 private:
  // Default light grey color -- if this default changes, update the class
  // documentation.
  Eigen::Vector4d diffuse_{0.9, 0.9, 0.9, 1.0};
};

}  // namespace geometry
}  // namespace drake
