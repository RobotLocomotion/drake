#ifndef __DrakeCollisionModel_H__
#define __DrakeCollisionModel_H__

#include <memory>
#include <map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drakeCollisionMacros.h"
#include "Element.h"
#include "PointPair.h"

namespace DrakeCollision
{
  typedef std::pair<ElementId, ElementId> ElementIdPair;

  class DLLEXPORT_drakeCollision Model {
    public:
      Model() {}

      virtual ElementId addElement(std::unique_ptr<Element> element);

      virtual const Element* readElement(ElementId id);

      virtual void updateModel() {};

      virtual bool updateElementWorldTransform(const ElementId id, 
          const Eigen::Matrix4d& T_local_to_world);

      virtual bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check, 
          const bool use_margins,
          std::vector<PointPair>& closest_points)
      { return false; };

      virtual bool collisionPointsAllToAll(const bool use_margins,
          std::vector<PointPair>& points)
      { return false; };

      virtual bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs, 
          const bool use_margins,
          std::vector<PointPair>& closest_points)
      { return false; };
      //
      // Performs raycasting collision detecting (like a LIDAR / laser rangefinder)
      //
      // @param origin Vector3d specifying the position of the ray's origin
      // @param ray_endpoint Vector3d specifying a second point on the ray in world coordinates
      // @param distance to the first collision, or -1 on no collision
      //
      virtual bool collisionRaycast(const Eigen::Matrix3Xd &origin, const Eigen::Matrix3Xd &ray_endpoint, bool use_margins, Eigen::VectorXd &distances) { return false; };

    protected:
      std::map< ElementId, std::unique_ptr<Element> >  elements;

    private:
      Model(const Model&) {}
      Model& operator=(const Model&) { return *this; }
  };

}
  
#endif
