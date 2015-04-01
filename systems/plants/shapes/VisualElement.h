#ifndef __DrakeShapesVisualElement_H__
#define __DrakeShapesVisualElement_H__
#include <memory>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"

#include "drakeShapesMacros.h"

namespace DrakeShapes
{
  class DLLEXPORT_drakeShapes VisualElement : public Element
  {
    public:
      VisualElement(const Eigen::Matrix4d& T_element_to_local)
    : Element(T_element_to_local), material(Eigen::Vector4d(0.7, 0.7, 0.7, 1)) {};

      VisualElement(const Geometry& geometry,
                    const Eigen::Matrix4d& T_element_to_local, 
                    const Eigen::Vector4d& material)
    : Element(geometry, T_element_to_local), material(material) {};

      virtual ~VisualElement(){};

      void setMaterial(const Eigen::Vector4d& material);

      const Eigen::Vector4d& getMaterial() const;
    protected:
      Eigen::Vector4d material;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // See http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
  // for why this is necessary.
  typedef std::vector< VisualElement, Eigen::aligned_allocator<VisualElement> >
    VectorOfVisualElements;
}
#endif
