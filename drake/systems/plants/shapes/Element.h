#ifndef DRAKE_SYSTEMS_PLANTS_SHAPES_ELEMENT_H_
#define DRAKE_SYSTEMS_PLANTS_SHAPES_ELEMENT_H_

#include <memory>
#include <utility>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Geometry.h"
#include "drake/drakeShapes_export.h"

namespace DrakeShapes {
class DRAKESHAPES_EXPORT Element {
 public:
  Element(const Geometry& geometry, const Eigen::Isometry3d& T_element_to_local)
      : geometry(geometry.clone()), T_element_to_local(T_element_to_local){}

  Element(const Geometry& geometry)
      : geometry(geometry.clone()),
        T_element_to_local(Eigen::Isometry3d::Identity()){}

  Element(const Eigen::Isometry3d& T_element_to_local)
      : geometry(), T_element_to_local(T_element_to_local){}

  virtual ~Element(){}

  virtual Element* clone() const;

  const Eigen::Isometry3d& getWorldTransform() const;

  const Eigen::Isometry3d& getLocalTransform() const;

  virtual void updateWorldTransform(const Eigen::Isometry3d& T_local_to_world);

  Shape getShape() const;

  void setGeometry(const Geometry& geometry);

  bool hasGeometry() const;

  const Geometry& getGeometry() const;

  void getTerrainContactPoints(Eigen::Matrix3Xd& points) const;

 protected:
  virtual void setWorldTransform(const Eigen::Isometry3d& T_elem_to_world);
  Eigen::Isometry3d T_element_to_world;
  const Eigen::Isometry3d T_element_to_local;
  std::unique_ptr<Geometry> geometry;

  Element(const Element&);
  Element& operator=(const Element&) { return *this; }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

#endif  // DRAKE_SYSTEMS_PLANTS_SHAPES_ELEMENT_H_
