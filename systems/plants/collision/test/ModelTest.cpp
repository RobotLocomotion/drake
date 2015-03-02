#include <iostream>
#include <math.h>

#include "DrakeCollision.h"

using namespace DrakeCollision;
using namespace std;
using namespace Eigen;

int testPointPair(PointPair point, double distance, Vector3d normal, ElementId idA, ElementId idB)
{
  if (point.distance != distance) {
    cerr << "Wrong distance:" << endl;
    cerr << "  Expected " << distance << ", got " << point.distance << endl;
    return 1;
  }
  if (point.normal != normal) {
    cerr << "  Expected " << normal.transpose() << ", got " << point.normal.transpose() << endl;
    cerr << "Wrong normal:" << endl;
    return 1;
  }
  if (point.idA != idA) {
    cerr << "Wrong idA:" << endl;
    cerr << "  Expected " << idA << ", got " << point.idA << endl;
    return 1;
  }
  if (point.idB != idB) {
    cerr << "Wrong idB:" << endl;
    cerr << "  Expected " << idB << ", got " << point.idB << endl;
    return 1;
  }
  return 0;
}

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
int main()
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
  ElementId id1, id2, id3;
  id1 = model->addElement(unique_ptr<Element>(new Element(unique_ptr<Geometry>(new Box(Vector3d(1,1,1))))));
  id2 = model->addElement(unique_ptr<Element>(new Element(unique_ptr<Geometry>(new Sphere(0.5)))));
  id3 = model->addElement(unique_ptr<Element>(new Element(unique_ptr<Geometry>(new Sphere(0.5)))));
  model->updateElementWorldTransform(id1, T_body1_to_world);
  model->updateElementWorldTransform(id2, T_body2_to_world);
  model->updateElementWorldTransform(id3, T_body3_to_world);

  vector<PointPair> points;
  vector<ElementId> ids_to_check;
  ids_to_check.push_back(id1);
  ids_to_check.push_back(id2);
  ids_to_check.push_back(id3);

  model->closestPointsAllToAll(ids_to_check, true, points);
  if (points.size() != 3) {
    cerr << "Wrong number of points: " << endl;
    cerr << "  Expected 3, got " << points.size() << endl;
    return 1;
  }
  int out;
  cout << "Check points[0] ..." << endl;
  out = testPointPair(points[0], 1.0, Vector3d(-1, 0, 0), id1, id2);
  if (out == 0) {
    cout << "Check points[1] ..." << endl;
    out = testPointPair(points[1], 1.6213203435596428, Vector3d(-sqrt(2)/2, -sqrt(2)/2, 0), id1, id3);
  }
  if (out == 0) {
    cout << "Check points[2] ..." << endl;
    out = testPointPair(points[2], 1.0, Vector3d(0, -1, 0), id2, id3);
  }
  cout << out << endl;
  return out;
  //if (points[0].distance != 1.0) {
    //cerr << "Wrong distance:" << endl;
    //cerr << "  Expected 1.0, got " << points[0].distance << endl;
    //return 1;
  //}
  //if (points[0].idA != id1) {
    //cerr << "Wrong idA:" << endl;
    //cerr << "  Expected " << id1 << ", got " << points[0].idA << endl;
    //return 1;
  //}
  //if (points[0].idB != id1) {
    //cerr << "Wrong idB:" << endl;
    //cerr << "  Expected " << id2 << ", got " << points[0].idB << endl;
    //return 1;
  //}

  //return 0;
};
  
   //Bodies 3 and 1
  //bodyA_idx.clear();
  //bodyB_idx.clear();
  //normal.resize(0,0);
  //distance.resize(0,0);
  //active_bodies_idx.at(0) = 3;
  //active_bodies_idx.at(1) = 1;
  //model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  //normal,distance,active_bodies_idx);
  //BOOST_CHECK_EQUAL(distance.size(), 1);
  //BOOST_CHECK_CLOSE(distance(0), 1.6213203435596428,1e-8);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(0), 3);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(0), 1);
  
   //Bodies 2 and 3
  //bodyA_idx.clear();
  //bodyB_idx.clear();
  //normal.resize(0,0);
  //distance.resize(0,0);
  //active_bodies_idx.at(0) = 2;
  //active_bodies_idx.at(1) = 3;
  //model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  //normal,distance,active_bodies_idx);
  //BOOST_CHECK_EQUAL(distance.size(), 1);
  //BOOST_CHECK_EQUAL(distance(0), 1.0);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(0), 2);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(0), 3);

   //Bodies 1, 2, and 3
  //bodyA_idx.clear();
  //bodyB_idx.clear();
  //normal.resize(0,0);
  //distance.resize(0,0);
  //active_bodies_idx.resize(3);
  //active_bodies_idx.at(0) = 2;
  //active_bodies_idx.at(1) = 1;
  //active_bodies_idx.at(2) = 3;
  //model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  //normal,distance,active_bodies_idx);
  //BOOST_CHECK_EQUAL(distance.size(), 3);
  //BOOST_CHECK_EQUAL(distance(0), 1.0);
  //BOOST_CHECK_EQUAL(distance(1), 1.0);
  //BOOST_CHECK_CLOSE(distance(2), 1.6213203435596428,1e-8);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(0), 2);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(1), 2);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(2), 1);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(0), 1);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(1), 3);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(2), 3);
//}

 //Three geometries (cube (1 m edges) and two spheres (0.5 m radii), attached
 //to two rigid bodies, arranged like this

                         //+ - - - - - +
                         //|   *****   |  
                           //**     **      
                         //| *   2b  * |  --+--
                           //**     **      |
                         //|   *****   |    |
                                          //|
          //^              |  Body 2   |   2 m
        //y |                               |
          //|              |           |    | 
      //+---+---+              *****        | 
      //|   |   |          | **      * |    | 
      //|   +---+---->       *   2a  *    --+--
      //| 1     |   x      | **     ** |  
      //+-------+              *****      
                         //+- - - - - -+  

          //|                    |
          //+------ 2 m ---------+
          //|                    |

//BOOST_AUTO_TEST_CASE(filter_by_group)
//{
  //vector<double> box_params {1,1,1};
  //vector<double> sphere_params {0.5};
  //Matrix4d T_body1_to_world, T_body2_to_world, T_geom2b_to_body2;
  //T_body1_to_world.setIdentity();
  //T_body2_to_world.setIdentity();
  //T_geom2b_to_body2.setIdentity();
  //T_body2_to_world.topRightCorner(3,1) << 2,0,0;
  //T_geom2b_to_body2.topRightCorner(3,1) << 0,2,0;

  //shared_ptr<Model> model = newModel();
  //model->addElement(1,0,Matrix4d::Identity(),BOX,box_params,"default",false);
  //model->addElement(2,0,Matrix4d::Identity(),SPHERE,sphere_params,"groupA",false);
  //model->addElement(2,0,T_geom2b_to_body2,SPHERE,sphere_params,"groupB",false);
  //model->updateElementsForBody(1, T_body1_to_world);
  //model->updateElementsForBody(2, T_body2_to_world);

  //MatrixXd ptsA;
  //MatrixXd ptsB;
  //vector<int> bodyA_idx;
  //vector<int> bodyB_idx;
  //MatrixXd normal;
  //VectorXd distance;
  //set<string> active_group_names;
  //vector<int> active_bodies_idx(2);

   //Elements 1 and 2a
  //active_group_names.insert("default");
  //active_group_names.insert("groupA");
  //model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  //normal,distance,active_group_names);
  //BOOST_CHECK_EQUAL(distance.size(), 1);
  //BOOST_CHECK_EQUAL(distance(0), 1.0);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(0), 1);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(0), 2);
  
   //Elements 2b and 1
  //bodyA_idx.clear();
  //bodyB_idx.clear();
  //normal.resize(0,0);
  //distance.resize(0,0);
  //active_group_names.clear();
  //active_group_names.insert("default");
  //active_group_names.insert("groupB");
  //model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  //normal,distance,active_group_names);
  //BOOST_CHECK_EQUAL(distance.size(), 1);
  //BOOST_CHECK_CLOSE(distance(0), 1.6213203435596428,1e-8);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(0), 1);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(0), 2);
  
   //Bodies 1 and 2
  //bodyA_idx.clear();
  //bodyB_idx.clear();
  //normal.resize(0,0);
  //distance.resize(0,0);
  //active_bodies_idx.resize(3);
  //active_bodies_idx.at(0) = 2;
  //active_bodies_idx.at(1) = 1;
  //model->closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB, 
                                  //normal,distance,active_bodies_idx);
  //BOOST_CHECK_EQUAL(distance.size(), 2);
  //BOOST_CHECK_EQUAL(distance(0), 1.0);
  //BOOST_CHECK_CLOSE(distance(1), 1.6213203435596428,1e-8);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(0), 2);
  //BOOST_CHECK_EQUAL(bodyA_idx.at(1), 2);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(0), 1);
  //BOOST_CHECK_EQUAL(bodyB_idx.at(1), 1);
//}
