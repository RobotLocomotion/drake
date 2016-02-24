#include <iostream>

#include "drake/systems/plants/collision/DrakeCollision.h"
#include "FCLModel.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision
{

  ElementId FCLModel::addElement(const Element& element)
  {
    ElementId id =  Model::addElement(element);

    if (id != 0) {
      switch (elements[id]->getShape()) {
        case DrakeShapes::BOX:
          {
            const auto box = static_cast<const DrakeShapes::Box&>(elements[id]->getGeometry());
	    // ...
          }
          break;
        case DrakeShapes::SPHERE:
          {
            const auto sphere = static_cast<const DrakeShapes::Sphere&>(elements[id]->getGeometry());
	    // ...
          }
          break;
        case DrakeShapes::CYLINDER:
          {
            const auto cylinder = static_cast<const DrakeShapes::Cylinder&>(elements[id]->getGeometry());
	    // ...
          }
          break;
        case DrakeShapes::MESH:
          {
            const auto mesh = static_cast<const DrakeShapes::Mesh&>(elements[id]->getGeometry());
	    // ...
          }
          break;
        case DrakeShapes::MESH_POINTS:
          {
            const auto mesh = static_cast<const DrakeShapes::MeshPoints&>(elements[id]->getGeometry());
	    // ...
          }
          break;
        case DrakeShapes::CAPSULE:
          {
            const auto capsule = static_cast<const DrakeShapes::Capsule&>(elements[id]->getGeometry());
	    // ...
          }
          break;
        default:
          cerr << "Warning: Collision elements[id] has an unknown type "
	       << elements[id]->getShape() << endl;
          throw unknownShapeException(elements[id]->getShape());
          break;
      }

      //if (elements[id]->isStatic()) {...}

    }
    return id;
  }

  vector<PointPair> FCLModel::potentialCollisionPoints(bool use_margins)
  {
    ResultCollector c;
    // ccl: c on stack but returns an ivar?
    return c.getResults();
  }

  bool FCLModel::collidingPointsCheckOnly(const vector<Vector3d>& points,
					  double collision_threshold)
  {
    //if (c.isInCollision()) {
    //  return true;
    //}

    return false;
  }

  vector<size_t> FCLModel::collidingPoints(const vector<Vector3d>& points, 
					   double collision_threshold)
  {
    vector<size_t> in_collision_indices;

    for (size_t i = 0; i < points.size(); ++i) {
      //if (c.isInCollision()) {
      //  in_collision_indices.push_back(i);
      //}
    }

    return in_collision_indices;
  }

  bool FCLModel::updateElementWorldTransform(const ElementId id, 
					     const Isometry3d& T_local_to_world)
  {
    const bool element_exists(Model::updateElementWorldTransform(id, T_local_to_world));
    if (element_exists) {
      const Isometry3d& T = elements[id]->getWorldTransform();
      //btMatrix3x3 rot;
      //btVector3 pos;
      //btTransform btT;

      //rot.setValue( T(0,0), T(0,1), T(0,2),
      //              T(1,0), T(1,1), T(1,2),
      //              T(2,0), T(2,1), T(2,2) );
      //btT.setBasis(rot);
      //pos.setValue( T(0,3), T(1,3), T(2,3) );
      //btT.setOrigin(pos);
      // ...
    }
    return element_exists;
  }

  void FCLModel::updateModel()
  {
    // ...
  }

  bool FCLModel::findClosestPointsBtwElements(const ElementId idA,
					      const ElementId idB,
					      const bool use_margins,
					      std::unique_ptr<ResultCollector>& c)
  {
    // special case: two spheres (because we need to handle the zero-radius sphere case)
    if (elements[idA]->getShape() == DrakeShapes::SPHERE && elements[idB]->getShape() == DrakeShapes::SPHERE)
    {
      const Isometry3d& TA_world = elements[idA]->getWorldTransform();
      const Isometry3d& TB_world = elements[idB]->getWorldTransform();
      auto xA_world = TA_world.translation();
      auto xB_world = TB_world.translation();
      double radiusA = dynamic_cast<const DrakeShapes::Sphere&>(elements[idA]->getGeometry()).radius;
      double radiusB = dynamic_cast<const DrakeShapes::Sphere&>(elements[idB]->getGeometry()).radius;
      double distance = (xA_world-xB_world).norm();
      c->addSingleResult(idA,
                         idB,
                         elements[idA]->getLocalTransform() * TA_world.inverse() * (xA_world + (xB_world-xA_world)*radiusA/distance), // ptA (in body A coords)
                         elements[idB]->getLocalTransform() * TB_world.inverse() * (xB_world + (xA_world-xB_world)*radiusB/distance), // ptB (in body B coords)
                         (xA_world-xB_world)/distance,
                         distance-radiusA-radiusB);
      return true;
    }
    // ...
    //convexConvex.getClosestPoints(input ,gjkOutput,0);

    if (elements[idA]->getShape() == DrakeShapes::MESH || 
        elements[idA]->getShape() == DrakeShapes::MESH_POINTS || 
        elements[idA]->getShape() == DrakeShapes::BOX) {
      //...
    } else {
      //...
    }
    if (elements[idB]->getShape() == DrakeShapes::MESH || 
        elements[idB]->getShape() == DrakeShapes::MESH_POINTS || 
        elements[idB]->getShape() == DrakeShapes::BOX) {
      //...
    } else {
      //...
    }

    //...
    //auto point_on_A = elements[idA]->getLocalTransform() * toVector3d(point_on_elemA);
    //auto point_on_B = elements[idB]->getLocalTransform() * toVector3d(point_on_elemB);
    //...
    //if (gjkOutput.m_hasResult) {
    //  c->addSingleResult(idA, idB, point_on_A, point_on_B, toVector3d(gjkOutput.m_normalOnBInWorld), (double) distance);
    //} else {
    //  throw std::runtime_error("In FCLModel::findClosestPointsBtwElements: No closest point found between " + to_string(idA) + " and " + to_string(idB));
    // }

    return (c->pts.size() > 0);
  }






  void FCLModel::collisionDetectFromPoints(const Matrix3Xd& points,
					   bool use_margins,
					   std::vector<PointPair>& closest_points)
  {
    closest_points.resize(points.cols(), PointPair(0, 0, Vector3d(), Vector3d(), Vector3d(), 0.0));
    VectorXd phi(points.cols());

    //...

    // do collision check against all bodies for each point using bullet's
    // internal getclosestpoitns solver
    for (int i=0; i<points.cols(); i++){
      bool got_one = false;
      //...

/*
        cout << "Point " << i << ": has result " << gjkOutput.m_hasResult << ", dist " << distance << endl;
        cout << "\tA in world: " << pointOnAinWorld.x() << "," << pointOnAinWorld.y() << "," << pointOnAinWorld.z() << endl;
        cout << "\tB in world: " << pointOnBinWorld.x() << "," << pointOnBinWorld.y() << "," << pointOnBinWorld.z() << endl;
        cout << "\tB normal: " << gjkOutput.m_normalOnBInWorld.x() << "," << gjkOutput.m_normalOnBInWorld.y() << "," << gjkOutput.m_normalOnBInWorld.z() << endl;
        cout << "\tBobj in world: " << input.m_transformB.getOrigin().x() << "," << input.m_transformB.getOrigin().y() << "," << input.m_transformB.getOrigin().z() << endl;
*/
      //closest_points[i] = PointPair(
      //          bt_objB_iter->first, bt_objB_iter->first,
      //          toVector3d(pointOnElemB), toVector3d(pointOnBinWorld),
      //          toVector3d(gjkOutput.m_normalOnBInWorld), distance);
    }
  }
  
  bool FCLModel::collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, bool use_margins, VectorXd &distances, Matrix3Xd &normals)
  {
    
    distances.resize(origins.cols());
    normals.resize(3, origins.cols());

    //...
    for (int i = 0; i < origins.cols(); i ++)
    {
        //...
        //if (ray_callback.hasHit()) {
            // compute distance to hit
            //btVector3 end = ray_callback.m_hitPointWorld;
            //Vector3d end_eigen(end.getX(), end.getY(), end.getZ());
            //distances(i) = (end_eigen - origins.col(i)).norm();
            //btVector3 normal = ray_callback.m_hitNormalWorld;
            //normals(0, i) = normal.getX();
            //normals(1, i) = normal.getY();
            //normals(2, i) = normal.getZ();
        //} else {
	//distances(i) = -1.;
	//normals(0, i) = 0.;
	//  normals(1, i) = 0.;
	//  normals(2, i) = 0.;
	//}
    }
    
    return true;
  } 

  bool FCLModel::closestPointsAllToAll(const vector<ElementId>& ids_to_check, 
				       const bool use_margins,
				       vector<PointPair>& closest_points)
  {
    vector<ElementIdPair> id_pairs;
    const auto elements_end = elements.end();
    for (auto idA_iter = ids_to_check.begin();
         idA_iter != ids_to_check.end();
         ++idA_iter) {
      auto elementA_iter = elements.find(*idA_iter);
      if (elementA_iter != elements_end) {
        for (auto idB_iter = idA_iter+1;
            idB_iter != ids_to_check.end();
            ++idB_iter) {
          auto elementB_iter = elements.find(*idB_iter);
          if (elementB_iter != elements_end) {
            if (elements[*idA_iter]->collidesWith(elements[*idB_iter].get())) {
              id_pairs.push_back(make_pair(*idA_iter, *idB_iter));
            }
          }
        }
      } 
    }
    return closestPointsPairwise(id_pairs, use_margins, closest_points);
  }

  bool FCLModel::closestPointsPairwise(const vector<ElementIdPair>& id_pairs, 
				       const bool use_margins,
				       vector<PointPair>& closest_points)
  {
    unique_ptr<ResultCollector> c(new ResultCollector());
    for (auto id_pair_iter = id_pairs.begin();
        id_pair_iter != id_pairs.end();
        ++id_pair_iter) {
      findClosestPointsBtwElements(id_pair_iter->first, id_pair_iter->second, 
                                   use_margins, c);
    }

    closest_points = c->getResults();
    return closest_points.size() > 0;
  }
  
  bool FCLModel::collisionPointsAllToAll(const bool use_margins,
					 vector<PointPair>& collision_points)
  {
    ResultCollector c;
    MatrixXd normals;
    vector<double> distance;

    //...
    collision_points = c.getResults();
    return c.pts.size() > 0;
  }

  FCLModel::unknownShapeException::unknownShapeException(DrakeShapes::Shape shape)
  { 
    std::ostringstream ostr; 
    ostr << shape; 
    this->shape_str = ostr.str(); 
  }
 
  const char* FCLModel::unknownShapeException::what() const throw()
  {
    return ("Unknown collision shape: " + shape_str + ". Ignoring this collision element").c_str();
  }

}
