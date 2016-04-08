#ifndef DRAKE_SYSTEMS_PLANTS_SHAPES_VISUALELEMENT_H_
#define DRAKE_SYSTEMS_PLANTS_SHAPES_VISUALELEMENT_H_

#include <memory>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"

#include "drake/drakeShapes_export.h"

namespace DrakeShapes {
class DRAKESHAPES_EXPORT VisualElement : public Element {
 public:
  VisualElement(const Eigen::Isometry3d& T_element_to_local)
      : Element(T_element_to_local),
        material(Eigen::Vector4d(0.7, 0.7, 0.7, 1)) {}

  VisualElement(const Geometry& geometry,
                const Eigen::Isometry3d& T_element_to_local,
                const Eigen::Vector4d& material)
      : Element(geometry, T_element_to_local), material(material) {}

  virtual ~VisualElement() {}

  void setMaterial(const Eigen::Vector4d& material);

  const Eigen::Vector4d& getMaterial() const;

 protected:
  Eigen::Vector4d material;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// See http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
// for why this is necessary.
typedef std::vector<VisualElement, Eigen::aligned_allocator<VisualElement> >
    VectorOfVisualElements;
}

#endif  // DRAKE_SYSTEMS_PLANTS_SHAPES_VISUALELEMENT_H_
