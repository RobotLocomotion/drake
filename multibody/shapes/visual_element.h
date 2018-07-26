#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/common/eigen_stl_types.h"
#include "drake/multibody/shapes/element.h"

namespace DrakeShapes {

class VisualElement final : public Element {
 public:
  explicit VisualElement(const Eigen::Isometry3d& T_element_to_local);

  /**
   * Constructs a geometry at a specified transform with
   * a given material (i.e. a color specified as a 4-vector
   * of RGBA, each on [0, 1]).
   */
  VisualElement(const Geometry& geometry,
                const Eigen::Isometry3d& T_element_to_local,
                const Eigen::Vector4d& material,
                const std::string& name = "");

  /** Copy constructor for use by, e.g., std::vector. */
  VisualElement(const VisualElement&) = default;
  void operator=(const VisualElement&) = delete;
  VisualElement(VisualElement&&) = delete;
  void operator=(VisualElement&&) = delete;

  ~VisualElement() override = default;

  /**
  * Sets the element's material color, in RGBA format.
  */
  void setMaterial(const Eigen::Vector4d& material);

  /**
  * Retrieves the element's material color, in RGBA format.
  */
  const Eigen::Vector4d& getMaterial() const;

 protected:
  Eigen::Vector4d material;
  std::string name;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// See http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
// for why this is necessary.
typedef drake::eigen_aligned_std_vector<VisualElement> VectorOfVisualElements;

}  // namespace DrakeShapes
