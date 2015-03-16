#include <iostream>
#include <math.h>

#include "DrakeCollision.h"

using namespace DrakeCollision;
using namespace std;
using namespace Eigen;

int testPointPair(PointPair point, double distance, Vector3d normal, ElementId idA, ElementId idB)
{
  if (point.getDistance() != distance) {
    cerr << "Wrong distance:" << endl;
    cerr << "  Expected " << distance << ", got " << point.getDistance() << endl;
    return 1;
  }
  if (point.getNormal() != normal) {
    cerr << "  Expected " << normal.transpose() << ", got " << point.getNormal().transpose() << endl;
    cerr << "Wrong normal:" << endl;
    return 1;
  }
  if (point.getIdA() != idA) {
    cerr << "Wrong idA:" << endl;
    cerr << "  Expected " << idA << ", got " << point.getIdA() << endl;
    return 1;
  }
  if (point.getIdB() != idB) {
    cerr << "Wrong idB:" << endl;
    cerr << "  Expected " << idB << ", got " << point.getIdB() << endl;
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
  id1 = model->addElement(Element(DrakeShapes::Box(Vector3d(1,1,1))));
  id2 = model->addElement(Element(DrakeShapes::Sphere(0.5)));
  id3 = model->addElement(Element(DrakeShapes::Sphere(0.5)));
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
}
