#ifndef __DrakeCollisionBulletModel_H__
#define __DrakeCollisionBulletModel_H__

#include <map>

#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "BulletElement.h"
#include "BulletResultCollector.h"
#include "Body.h"

namespace DrakeCollision
{
  class BulletModel;    // forward declaration
  
  struct OverlapFilterCallback : public btOverlapFilterCallback
  {
    // return true when pairs need collision
    virtual bool  needBroadphaseCollision(btBroadphaseProxy* proxy0,
        btBroadphaseProxy* proxy1) const;

    BulletModel* parent_model;
  };

  struct ElementData 
  {
    int body_idx;
    Shape shape;
    
    ElementData(int body_idx, Shape shape) : body_idx(body_idx), shape(shape) {};
  };

  class BulletModel : public Model
  {
    public:
      BulletModel();
      ~BulletModel();

      typedef std::vector<BulletElement> ElementVec;
      
      //Required member functions for Model interface
      virtual void resize(int num_bodies) {};

      virtual void addElement(const int body_ind, const int parent_idx, 
                              const Eigen::Matrix4d& T_elem_to_link, Shape shape, 
                              const std::vector<double>& params, 
                              const std::string& group_name,
                              bool is_static,
                              bool use_margins = true);

      virtual bool updateElementsForBody(const int body_idx,
                                  const Eigen::Matrix4d& T_link_to_world);

      virtual bool setCollisionFilter(const int body_idx, 
                                      const uint16_t group, 
                                      const uint16_t mask);

      virtual bool findClosestPointsBtwElements(const int bodyA_idx,
                                                const int bodyB_idx, 
                                                const BulletElement& elemA, 
                                                const BulletElement& elemB, 
                                                const std::set<std::string>& active_element_groups,
                                                const ResultCollShPtr& c);

      virtual bool findCollisionPointsBtwElements(const int bodyA_idx,
                                                  const int bodyB_idx, 
                                                  const BulletElement& elemA, 
                                                  const BulletElement& elemB, 
                                                  const ResultCollShPtr& c);

      virtual bool getPointCollision(const int body_idx, 
                                      const int body_collision_idx, 
                                      Eigen::Vector3d &ptA, Eigen::Vector3d &ptB, 
                                      Eigen::Vector3d &normal);

      virtual bool getPairwiseCollision(const int bodyA_idx, 
                                               const int bodyB_idx, 
                                               Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB, 
                                               Eigen::MatrixXd& normals);
      
      virtual bool getPairwisePointCollision(const int bodyA_idx,
                                                  const int bodyB_idx, 
                                                  const int bodyA_collision_idx,
                                                  Eigen::Vector3d &ptA, Eigen::Vector3d &ptB, 
                                                  Eigen::Vector3d &normal);

      virtual bool getClosestPoints(const int bodyA_idx, const int bodyB_idx,
          Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal,
          double& distance);

      /* 
       * Returns one pair of points for each body that has collision elements.
       */
      virtual bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
                                               std::vector<int>& bodyB_idx, 
                                               Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB,
                                               Eigen::MatrixXd& normal, 
                                               Eigen::VectorXd& distance,
                                               const std::vector<int>& bodies_idx,
                                               const std::set<std::string>& active_element_groups);
      // END Required member functions

      virtual const Body& getBody(int body_idx) const
      {
        return bodies.at(body_idx);
      };
      
      virtual bool allCollisions(std::vector<int>& bodyA_idx,
              std::vector<int>& bodyB_idx,
              Eigen::MatrixXd& ptsA, Eigen::MatrixXd& ptsB);
      
      virtual bool collisionRaycast(const Eigen::Matrix3Xd &origins, 
              const Eigen::Matrix3Xd &ray_endpoints, Eigen::VectorXd &distances);
      
      // For getClosestPoints
      //btGjkEpaPenetrationDepthSolver epa;
      //btVoronoiSimplexSolver sGjkSimplexSolver;
      //TestbtCollisionWorld test;
      
    protected:
      virtual void updateElement(BulletElement& elem, 
          const Eigen::Matrix4d& T_link_to_world)
      {
        elem.updateWorldTransform(T_link_to_world);
      }

      virtual void setCollisionFilter(Body& body, const bitmask& group, 
          const bitmask& mask)
      {
        body.setGroup(group);
        body.setMask(mask);
      }
      bool findClosestPointsBtwElements(const int bodyA_idx, 
                                        const int bodyB_idx,
                                        const BulletElement& elemA, 
                                        const ElementVec& elem_vecB, 
                                        const std::set<std::string> active_element_groups,
                                        const ResultCollShPtr c)
      {
        for (BulletElement elemB : elem_vecB) {
          if ( active_element_groups.empty() || 
              ( (active_element_groups.find(elemA.getGroupName()) != active_element_groups.end()) &&
                (active_element_groups.find(elemB.getGroupName()) != active_element_groups.end()) ) ) {
            findClosestPointsBtwElements(bodyA_idx, bodyB_idx, elemA,elemB, active_element_groups,c);
          }
        }
        return (c->pts.size() > 0);
      }

       bool findClosestPointsBtwElements(const int bodyA_idx, 
                                         const int bodyB_idx,
                                         const ElementVec& elem_vecA, 
                                         const ElementVec& elem_vecB, 
                                         const std::set<std::string> active_element_groups,
                                         const ResultCollShPtr& c)
      {
        for (BulletElement elemA : elem_vecA) {
          findClosestPointsBtwElements(bodyA_idx,bodyB_idx,elemA, elem_vecB, active_element_groups, c);
        }
        return (c->pts.size() > 0);
      }

       bool findClosestPointsBtwBodies(const int bodyA_idx, 
          const int bodyB_idx, 
          const std::set<std::string> active_element_groups,
          const ResultCollShPtr& c)
      {
        bool result;
        //DEBUG
        //try {
        //END_DEBUG
        result = findClosestPointsBtwElements(bodyA_idx,bodyB_idx,
                                            bodies[bodyA_idx].getElements(),
                                            bodies[bodyB_idx].getElements(),
                                            active_element_groups, c);
        //DEBUG
        //} catch (std::exception& ex) {
          //std::cerr << "In ModelTemplate::findClosetPointsBtwBodies" << std::endl;
          //throw;
        //}
        //END_DEBUG
        return result;
      }

      bool findCollisionPointsBtwElements(const int bodyA_idx, 
                                          const int bodyB_idx,
                                          const BulletElement& elemA, 
                                          const ElementVec& elem_vecB, 
                                          const ResultCollShPtr& c)
      {
        for (BulletElement elemB : elem_vecB) {
          findCollisionPointsBtwElements(bodyA_idx,bodyB_idx,elemA,elemB,c);
        }
        return (c->pts.size() > 0);
      }

      bool findCollisionPointsBtwElements(const int bodyA_idx, 
                                          const int bodyB_idx,
                                          const ElementVec& elem_vecA, 
                                          const ElementVec& elem_vecB, 
                                          const ResultCollShPtr& c)
      {
        for (BulletElement elemA : elem_vecA) {
          findCollisionPointsBtwElements(bodyA_idx,bodyB_idx,elemA, elem_vecB,c);
        }
        return (c->pts.size() > 0);
      };

      bool findCollisionPointsBtwBodies(const int bodyA_idx, 
          const int bodyB_idx,
          const ResultCollShPtr& c)
      {
        return findCollisionPointsBtwElements(bodyA_idx,bodyB_idx,
                                              bodies[bodyA_idx].getElements(),
                                              bodies[bodyB_idx].getElements(),c);
      };

      btCollisionWorld* bt_collision_world;
      std::map< int, Body > bodies;
      
      btDefaultCollisionConfiguration bt_collision_configuration;
      btCollisionDispatcher* bt_collision_dispatcher;
      btDbvtBroadphase bt_collision_broadphase;
      OverlapFilterCallback filter_callback;
      std::vector< std::unique_ptr< ElementData > > element_data;

    protected:
      virtual const std::vector<int> bodyIndices() const;
  };
}
#endif
