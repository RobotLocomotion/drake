#ifndef __DrakeCollisionModelTemplate_H__
#define __DrakeCollisionModelTemplate_H__

#include "DrakeCollision.h"
#include "Model.h"
#include "Body.h"
#include "Element.h"
#include "ResultCollector.h"
#include "MinDistResultCollector.h"

namespace DrakeCollision
{
  template<typename ElementT>
  class ModelTemplate : public Model {
    typedef std::vector<ElementT> ElementTVec;
    public:
      //Required member functions for Model interface
      virtual void resize(int num_bodies)
      {
      };

      virtual void addElement(const int body_idx, const int parent_idx, 
          const Matrix4d& T_elem_to_link, Shape shape, 
          const std::vector<double>& params,
          bool is_static)
      {
        //DEBUG
        //try {
        //std::cout << "ModelTemplate::addElement: START" << std::endl;
        //std::cout << "ModelTemplate::addElement: Add element for body " << body_idx << std::endl;
        //END_DEBUG
        bodies[body_idx].addElement(body_idx, parent_idx, T_elem_to_link, 
                                       shape, params );
        //DEBUG
        //} catch (const std::out_of_range& oor) {
          //std::string msg("In ModelTemplate::addElement:\n");
          //throw std::out_of_range(msg + oor.what());
        //}
        //END_DEBUG
      };

       virtual bool updateElementsForBody(const int body_idx, 
          const Matrix4d& T_link_to_world)
      {
        //DEBUG
        //try {
        //END_DEBUG
          auto iter_for_body( bodies.find(body_idx) );
          if ( iter_for_body!=bodies.end() ) { // Then bodies[body_idx] exists
            iter_for_body->second.updateElements(T_link_to_world);
            return true;
          } else {
            return false;
          }
        //DEBUG
        //} catch (const std::out_of_range& oor) {
          //std::string msg("In ModelTemplate::updateElementsForBody:\n");
          //throw std::out_of_range(msg + oor.what());
        //}
        //END_DEBUG
        //DEBUG
        //std::cout << "ModelTemplate::addElement: END" << body_idx << std::endl;
        //END_DEBUG
      };
       
      virtual bool setCollisionFilter(const int body_idx, 
                                      const uint16_t group, 
                                      const uint16_t mask)
      {
          auto iter_for_body( bodies.find(body_idx) );
          if ( iter_for_body!=bodies.end() ) { // Then bodies[body_idx] exists
            setCollisionFilter(iter_for_body->second,group,mask);
            return true;
          } else {
            return false;
          }

      }
      
      virtual bool getPointCollision(const int body_idx, 
          const int body_collision_idx, 
          Vector3d &ptA, Vector3d &ptB, 
          Vector3d &normal)=0;

      virtual bool getPairwiseCollision(const int bodyA_idx, 
                                               const int bodyB_idx, 
                                               MatrixXd& ptsA, MatrixXd& ptsB, 
                                               MatrixXd& normals)
      {
        //DEBUG
        //try {
        //END_DEBUG
        ResultCollShPtr c = newResultCollector();
        findCollisionPointsBtwBodies(bodyA_idx,bodyB_idx,c);
        c->getResults(ptsA,ptsB,normals);

        return (c->pts.size() > 0);
        //DEBUG
        //} catch (const std::out_of_range& oor) {
          //std::string msg("In ModelTemplate::getPairwiseCollision:\n");
          //throw std::out_of_range(msg + oor.what());
        //}
        //END_DEBUG
      };

       virtual bool getPairwisePointCollision(const int bodyA_idx, 
                                                  const int bodyB_idx, 
                                                  const int bodyA_collision_idx,
                                                  Vector3d &ptA, Vector3d &ptB, 
                                                  Vector3d &normal)
      {
        //DEBUG
        //try {
        //END_DEBUG
        ResultCollShPtr c = newResultCollector();
        const ElementT& elemA = bodies.at(bodyA_idx).at(bodyA_collision_idx);
        findCollisionPointsBtwElements(bodyA_idx,bodyB_idx,elemA,bodies[bodyB_idx].getElements(),c);
        c->getResults(ptA,ptB,normal);

        return (c->pts.size() > 0);
        //DEBUG
        //} catch (const std::out_of_range& oor) {
          //std::string msg("In ModelTemplate::getPairwisePointCollision:\n");
          //throw std::out_of_range(msg + oor.what());
        //}
        //END_DEBUG
      };

      virtual bool getClosestPoints(const int bodyA_idx, const int bodyB_idx,
          Vector3d& ptA, Vector3d& ptB, Vector3d& normal,
          double& distance)
      {
        //DEBUG
        //try {
        //END_DEBUG
        ResultCollShPtr c(new MinDistResultCollector());
        findClosestPointsBtwBodies(bodyA_idx,bodyB_idx,c);
        c->pts.at(0).getResults(ptA, ptB, normal, distance);
        return (c->pts.size() > 0);
        //DEBUG
        //} catch (const std::out_of_range& oor) {
          //std::string msg("In ModelTemplate::getClosestPoints:\n");
          //throw std::out_of_range(msg + oor.what());
        //}
        //END_DEBUG
      }

      /* 
       * Returns one pair of points for each body that has collision elements.
       */
      virtual bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
                                               std::vector<int>& bodyB_idx, 
                                               MatrixXd& ptsA, MatrixXd& ptsB,
                                               MatrixXd& normal, 
                                               VectorXd& distance,
                                               std::vector<int>& bodies_idx)
      {
        bool has_result=false;
        //DEBUG
        //try {
        //END_DEBUG
          //ResultCollector c;
          ResultCollShPtr c = std::make_shared<ResultCollector>();
          if (bodies_idx.size() == 0) {
            for (auto it = bodies.begin(); it != bodies.end(); ++it) {
              bodies_idx.push_back(it->first);
            }
          }
          //DEBUG
          //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
          //std::cout << "Num active bodies: " << bodies_idx.size() << std::endl;
          //END_DEBUG
          //for (auto itA=bodies.begin(); itA!=bodies.end(); ++itA) {
          for (typename std::vector<int>::const_iterator itA=bodies_idx.begin(); itA!=bodies_idx.end(); ++itA) {
            if (bodies.count(*itA) > 0) {
              //DEBUG
              //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
              //std::cout << "Body A found" << std::endl;
              //END_DEBUG
              //for (typename std::map<int,Body<ElementT>>::iterator itB=itA; itB!=bodies.end(); ++itB) {
              for (typename std::vector<int>::const_iterator itB=itA; itB!=bodies_idx.end(); ++itB) {
                //for (auto itB=bodies.begin(); itB!=bodies.end(); ++itB) {
                if (bodies.count(*itB) > 0) {
                  //DEBUG
                  //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
                  //std::cout << "Body B found" << std::endl;
                  //END_DEBUG
                  Body<ElementT>& bodyA(bodies[*itA]);
                  Body<ElementT>& bodyB(bodies[*itB]);
                  //DEBUG
                  //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
                  //std::cout << "Body A idx:" << bodyA.getBodyIdx() << std::endl;
                  //std::cout << "Body B idx:" << bodyB.getBodyIdx() << std::endl;
                  //END_DEBUG
                  //ResultCollShPtr c_min_dist = std::make_shared<MinDistResultCollector>();
                  if ( bodyA.collidesWith(bodyB) ) {
                    //DEBUG
                    //std::cout << "ModelTemplate::closestPointsAllBodies: Body A: " << bodyA.getBodyIdx() << std::endl;
                    //std::cout << "ModelTemplate::closestPointsAllBodies: Body B: " << bodyB.getBodyIdx() << std::endl;
                    //END_DEBUG
                    has_result = findClosestPointsBtwBodies(bodyA.getBodyIdx(),
                        bodyB.getBodyIdx(),
                        c);
                  }
                }
              }
            } 
          }
          c->getResults(bodyA_idx, bodyB_idx, ptsA, ptsB,normal,distance);
          
        //DEBUG
          //std::cout << "ModelTemplate:closestPointsAllBodies:" << std::endl;
          //std::cout << "ptsA:" << std::endl;
          //std::cout << ptsA.transpose() << std::endl;
          //std::cout << "ptsB:" << std::endl;
          //std::cout << ptsB.transpose() << std::endl;
          //std::cout << "normal:" << std::endl;
          //std::cout << normal.transpose() << std::endl;
        //END_DEBUG
        //DEBUG
        //} catch (const std::out_of_range& oor) {
          //std::string msg("In ModelTemplate::closestPointsAllBodies:\n");
          //throw std::out_of_range(msg + oor.what());
        //}
        //END_DEBUG
	  return has_result;
      };
      // END Required member functions
      
     virtual const Body<ElementT>& getBody(int body_idx) const
     {
       return bodies.at(body_idx);
     };


    protected:
      virtual void updateElement(ElementT& elem, 
          const Matrix4d& T_link_to_world)
      {
        elem.updateWorldTransform(T_link_to_world);
      }

      virtual void setCollisionFilter(Body<ElementT>& body, const bitmask& group, 
          const bitmask& mask)
      {
        body.setGroup(group);
        body.setMask(mask);
      }

      virtual bool findClosestPointsBtwElements(const int bodyA_idx, 
                                                const int bodyB_idx,
                                                const ElementT& elemA, 
                                                const ElementT& elemB, 
                                                const ResultCollShPtr& c)=0;

      virtual bool findCollisionPointsBtwElements(const int bodyA_idx,
                                                  const int bodyB_idx,
                                                  const ElementT& elemA, 
                                                  const ElementT& elemB, 
                                                  const ResultCollShPtr& c)=0;

       //void addElement(const int body_idx, const int parent_idx, 
                             //const ElementT& new_element)
      //{
        //bodies[body_idx].addElement(body_idx, parent_idx, new_element);
        ////element_pool.at(body_idx).push_back(new_element);
      //};

      bool findClosestPointsBtwElements(const int bodyA_idx, 
                                        const int bodyB_idx,
                                        const ElementT& elemA, 
                                        const ElementTVec& elem_vecB, 
                                        const ResultCollShPtr c)
      {
        for (ElementT elemB : elem_vecB) {
          findClosestPointsBtwElements(bodyA_idx, bodyB_idx, elemA,elemB,c);
        }
        return (c->pts.size() > 0);
      }

       bool findClosestPointsBtwElements(const int bodyA_idx, 
                                         const int bodyB_idx,
                                         const ElementTVec& elem_vecA, 
                                         const ElementTVec& elem_vecB, 
                                         const ResultCollShPtr& c)
      {
        for (ElementT elemA : elem_vecA) {
          findClosestPointsBtwElements(bodyA_idx,bodyB_idx,elemA, elem_vecB,c);
        }
        return (c->pts.size() > 0);
      }

       bool findClosestPointsBtwBodies(const int bodyA_idx, 
          const int bodyB_idx, 
          const ResultCollShPtr& c)
      {
        bool result;
        //DEBUG
        //try {
        //END_DEBUG
        result = findClosestPointsBtwElements(bodyA_idx,bodyB_idx,
                                            bodies[bodyA_idx].getElements(),
                                            bodies[bodyB_idx].getElements(),c);
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
                                          const ElementT& elemA, 
                                          const ElementTVec& elem_vecB, 
                                          const ResultCollShPtr& c)
      {
        for (ElementT elemB : elem_vecB) {
          findCollisionPointsBtwElements(bodyA_idx,bodyB_idx,elemA,elemB,c);
        }
        return (c->pts.size() > 0);
      }

      bool findCollisionPointsBtwElements(const int bodyA_idx, 
                                          const int bodyB_idx,
                                          const ElementTVec& elem_vecA, 
                                          const ElementTVec& elem_vecB, 
                                          const ResultCollShPtr& c)
      {
        for (ElementT elemA : elem_vecA) {
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

    protected:
      std::map< int, Body<ElementT> > bodies;
  };
}
#endif
