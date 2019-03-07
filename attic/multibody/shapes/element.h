#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/multibody/shapes/geometry.h"

namespace DrakeShapes {
class Element {
 public:
  Element(const Geometry& geometry_in,
          const Eigen::Isometry3d& T_element_to_local_in)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(T_element_to_local_in),
        geometry(geometry_in.clone()) {}

  explicit Element(const Geometry& geometry_in)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(Eigen::Isometry3d::Identity()),
        geometry(geometry_in.clone()) {}

  explicit Element(const Eigen::Isometry3d& T_element_to_local_in)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(T_element_to_local_in),
        geometry() {}

  virtual ~Element() {}

  virtual Element* clone() const;

  const Eigen::Isometry3d& getWorldTransform() const;

  const Eigen::Isometry3d& getLocalTransform() const;

  void SetLocalTransform(const Eigen::Isometry3d& T_element_to_local);

  void updateWorldTransform(const Eigen::Isometry3d& T_local_to_world);

  Shape getShape() const;

  void setGeometry(const Geometry& geometry);

  bool hasGeometry() const;

  const Geometry& getGeometry() const;

  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getTerrainContactPoints(Eigen::Matrix3Xd& points) const;

 protected:
  // Provide a copy constructor for use by our subclasses' clone().
  // Delete all of the other operations.
  Element(const Element&);
  void operator=(const Element&) = delete;
  Element(Element&&) = delete;
  void operator=(Element&&) = delete;

  void setWorldTransform(const Eigen::Isometry3d& T_elem_to_world);

  Eigen::Isometry3d T_element_to_world;
  Eigen::Isometry3d T_element_to_local;
  std::unique_ptr<Geometry> geometry;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace DrakeShapes
