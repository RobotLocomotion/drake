#define BOOST_TEST_MODULE Model test
#include <boost/test/unit_test.hpp>

#include <iostream>

#include "DrakeCollision.h"

using namespace DrakeCollision;
using namespace std;
using namespace Eigen;

/*
 * Three bodies (cube (1 m edges) and two spheres (0.5 m radii) arranged like this
 *
 *                             *****  
 *                           **     **   
 *                           *   3   * --+--
 *                           **     **   |
 *                             *****     |
 *                                       |
 *          ^                           2 m
 *        y |                            |
 *          |                            | 
 *      +---+---+              *****     | 
 *      |   |   |            **      *   | 
 *      |   +---+---->       *   2   * --+--
 *      | 1     |   x        **     ** 
 *      +-------+              *****   
 *          |                    |
 *          +------ 2 m ---------+
 *          |                    |
 *
 */
BOOST_AUTO_TEST_CASE(filter_by_body)
{
  vector<double> box_params {1,1,1};
  vector<double> sphere_params {0.5};
  Matrix4d T_body1_to_world, T_body2_to_world, T_body3_to_world;
  T_body1_to_world.setIdentity();
  T_body2_to_world.setIdentity();
  T_body3_to_world.setIdentity();
  T_body2_to_world.topRightCorner(3,1) << 2,0,0;
  T_body3_to_world.topRightCorner(3,1) << 2,2,0;

  shared_ptr<Model> model = newModel();
  model->addElement(1,0,Matrix4d::Identity(),BOX,box_params,"default",false);
  model->addElement(2,0,Matrix4d::Identity(),SPHERE,sphere_params,"default",false);
  model->addElement(3,0,Matrix4d::Identity(),SPHERE,sphere_params,"default",false);
  model->updateElementsForBody(1, T_body1_to_world);
  model->updateElementsForBody(2, T_body2_to_world);
  model->updateElementsForBody(3, T_body3_to_world);

  MatrixXd ptsA;
  MatrixXd ptsB;
  vector<int> bodyA_idx;
  vector<int> bodyB_idx;
  MatrixXd normal;
  VectorXd distance;
  vector<int> active_bodies_idx(2);

  // Bodies 1 and 2
  active_bodies_idx.at(0) = 1;
  active_bodies_idx.at(1) = 2;
  model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  normal,distance,active_bodies_idx);
  BOOST_CHECK_EQUAL(distance.size(), 1);
  BOOST_CHECK_EQUAL(distance(0), 1.0);
  BOOST_CHECK_EQUAL(bodyA_idx.at(0), 1);
  BOOST_CHECK_EQUAL(bodyB_idx.at(0), 2);
  
  // Bodies 3 and 1
  bodyA_idx.clear();
  bodyB_idx.clear();
  normal.resize(0,0);
  distance.resize(0,0);
  active_bodies_idx.at(0) = 3;
  active_bodies_idx.at(1) = 1;
  model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  normal,distance,active_bodies_idx);
  BOOST_CHECK_EQUAL(distance.size(), 1);
  BOOST_CHECK_CLOSE(distance(0), 1.6213203435596428,1e-8);
  BOOST_CHECK_EQUAL(bodyA_idx.at(0), 3);
  BOOST_CHECK_EQUAL(bodyB_idx.at(0), 1);
  
  // Bodies 2 and 3
  bodyA_idx.clear();
  bodyB_idx.clear();
  normal.resize(0,0);
  distance.resize(0,0);
  active_bodies_idx.at(0) = 2;
  active_bodies_idx.at(1) = 3;
  model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  normal,distance,active_bodies_idx);
  BOOST_CHECK_EQUAL(distance.size(), 1);
  BOOST_CHECK_EQUAL(distance(0), 1.0);
  BOOST_CHECK_EQUAL(bodyA_idx.at(0), 2);
  BOOST_CHECK_EQUAL(bodyB_idx.at(0), 3);

  // Bodies 1, 2, and 3
  bodyA_idx.clear();
  bodyB_idx.clear();
  normal.resize(0,0);
  distance.resize(0,0);
  active_bodies_idx.resize(3);
  active_bodies_idx.at(0) = 2;
  active_bodies_idx.at(1) = 1;
  active_bodies_idx.at(2) = 3;
  model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  normal,distance,active_bodies_idx);
  BOOST_CHECK_EQUAL(distance.size(), 3);
  BOOST_CHECK_EQUAL(distance(0), 1.0);
  BOOST_CHECK_EQUAL(distance(1), 1.0);
  BOOST_CHECK_CLOSE(distance(2), 1.6213203435596428,1e-8);
  BOOST_CHECK_EQUAL(bodyA_idx.at(0), 2);
  BOOST_CHECK_EQUAL(bodyA_idx.at(1), 2);
  BOOST_CHECK_EQUAL(bodyA_idx.at(2), 1);
  BOOST_CHECK_EQUAL(bodyB_idx.at(0), 1);
  BOOST_CHECK_EQUAL(bodyB_idx.at(1), 3);
  BOOST_CHECK_EQUAL(bodyB_idx.at(2), 3);
}

/*
 * Three geometries (cube (1 m edges) and two spheres (0.5 m radii), attached
 * to two rigid bodies, arranged like this
 *
 *                         + - - - - - +
 *                         |   *****   |  
 *                           **     **      
 *                         | *   2b  * |  --+--
 *                           **     **      |
 *                         |   *****   |    |
 *                                          |
 *          ^              |  Body 2   |   2 m
 *        y |                               |
 *          |              |           |    | 
 *      +---+---+              *****        | 
 *      |   |   |          | **      * |    | 
 *      |   +---+---->       *   2a  *    --+--
 *      | 1     |   x      | **     ** |  
 *      +-------+              *****      
 *                         +- - - - - -+  
 *
 *          |                    |
 *          +------ 2 m ---------+
 *          |                    |
 *
 */
BOOST_AUTO_TEST_CASE(filter_by_group)
{
  vector<double> box_params {1,1,1};
  vector<double> sphere_params {0.5};
  Matrix4d T_body1_to_world, T_body2_to_world, T_geom2b_to_body2;
  T_body1_to_world.setIdentity();
  T_body2_to_world.setIdentity();
  T_geom2b_to_body2.setIdentity();
  T_body2_to_world.topRightCorner(3,1) << 2,0,0;
  T_geom2b_to_body2.topRightCorner(3,1) << 0,2,0;

  shared_ptr<Model> model = newModel();
  model->addElement(1,0,Matrix4d::Identity(),BOX,box_params,"default",false);
  model->addElement(2,0,Matrix4d::Identity(),SPHERE,sphere_params,"groupA",false);
  model->addElement(2,0,T_geom2b_to_body2,SPHERE,sphere_params,"groupB",false);
  model->updateElementsForBody(1, T_body1_to_world);
  model->updateElementsForBody(2, T_body2_to_world);

  MatrixXd ptsA;
  MatrixXd ptsB;
  vector<int> bodyA_idx;
  vector<int> bodyB_idx;
  MatrixXd normal;
  VectorXd distance;
  set<string> active_group_names;
  vector<int> active_bodies_idx(2);

  // Elements 1 and 2a
  active_group_names.insert("default");
  active_group_names.insert("groupA");
  model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  normal,distance,active_group_names);
  BOOST_CHECK_EQUAL(distance.size(), 1);
  BOOST_CHECK_EQUAL(distance(0), 1.0);
  BOOST_CHECK_EQUAL(bodyA_idx.at(0), 1);
  BOOST_CHECK_EQUAL(bodyB_idx.at(0), 2);
  
  // Elements 2b and 1
  bodyA_idx.clear();
  bodyB_idx.clear();
  normal.resize(0,0);
  distance.resize(0,0);
  active_group_names.clear();
  active_group_names.insert("default");
  active_group_names.insert("groupB");
  model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  normal,distance,active_group_names);
  BOOST_CHECK_EQUAL(distance.size(), 1);
  BOOST_CHECK_CLOSE(distance(0), 1.6213203435596428,1e-8);
  BOOST_CHECK_EQUAL(bodyA_idx.at(0), 1);
  BOOST_CHECK_EQUAL(bodyB_idx.at(0), 2);
  
  // Bodies 1 and 2
  bodyA_idx.clear();
  bodyB_idx.clear();
  normal.resize(0,0);
  distance.resize(0,0);
  active_bodies_idx.resize(3);
  active_bodies_idx.at(0) = 2;
  active_bodies_idx.at(1) = 1;
  model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  normal,distance,active_bodies_idx);
  BOOST_CHECK_EQUAL(distance.size(), 2);
  BOOST_CHECK_EQUAL(distance(0), 1.0);
  BOOST_CHECK_CLOSE(distance(1), 1.6213203435596428,1e-8);
  BOOST_CHECK_EQUAL(bodyA_idx.at(0), 2);
  BOOST_CHECK_EQUAL(bodyA_idx.at(1), 2);
  BOOST_CHECK_EQUAL(bodyB_idx.at(0), 1);
  BOOST_CHECK_EQUAL(bodyB_idx.at(1), 1);
}
