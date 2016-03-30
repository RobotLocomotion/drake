#ifndef DRAKE_SYSTEMS_PLANTS_COLLISION_POINT_PAIR_H_
#define DRAKE_SYSTEMS_PLANTS_COLLISION_POINT_PAIR_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"
#include "drake/drakeCollision_export.h"

namespace DrakeCollision {

class DRAKECOLLISION_EXPORT PointPair {
 public:
  PointPair(const ElementId idA, const ElementId idB, const Eigen::Vector3d ptA,
            const Eigen::Vector3d ptB, const Eigen::Vector3d normal,
            double distance)
      : idA_(idA),
        idB_(idB),
        ptA_(ptA),
        ptB_(ptB),
        normal_(normal),
        distance_(distance) {}

  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal) const;
  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal, double* distance) const;
  ElementId getIdA() const;
  ElementId getIdB() const;
  Eigen::Vector3d getNormal() const;
  Eigen::Vector3d getPtA() const;
  Eigen::Vector3d getPtB() const;
  double getDistance() const;

  bool operator<(const PointPair& pt) const {
    return (distance_ < pt.distance_);
  }

  bool operator==(const PointPair& pt) const {
    return (distance_ == pt.distance_);
  }

  bool operator!=(const PointPair& pt) const {
    return (distance_ != pt.distance_);
  }

  bool operator<=(const PointPair& pt) const {
    return (distance_ <= pt.distance_);
  }

  bool operator>(const PointPair& pt) const {
    return (distance_ > pt.distance_);
  }

  bool operator>=(const PointPair& pt) const {
    return (distance_ >= pt.distance_);
  }

 protected:
  ElementId idA_;
  ElementId idB_;
  Eigen::Vector3d ptA_;
  Eigen::Vector3d ptB_;
  Eigen::Vector3d normal_;
  double distance_;
};

}  // namespace DrakeCollision

#endif  // DRAKE_SYSTEMS_PLANTS_COLLISION_POINT_PAIR_H_

