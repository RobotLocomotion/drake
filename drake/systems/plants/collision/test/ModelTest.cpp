#include <iostream>
#include <math.h>

#include "drake/systems/plants/collision/DrakeCollision.h"

using namespace DrakeCollision;
using namespace std;
using namespace Eigen;

int testPointPair(PointPair point, double distance, Vector3d normal,
                  ElementId idA, ElementId idB, Vector3d ptA, Vector3d ptB) {
  if (point.getDistance() != distance) {
    cerr << "Wrong distance:" << endl;
    cerr << "  Expected " << distance << ", got " << point.getDistance()
         << endl;
    return 1;
  }
  if (point.getNormal() != normal) {
    cerr << "  Expected " << normal.transpose() << ", got "
         << point.getNormal().transpose() << endl;
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
  if (!(point.getPtA() - ptA).isZero()) {
    cerr << "Wrong ptA:" << endl;
    cerr << "  Expected " << ptA.transpose() << ", got "
         << point.getPtA().transpose() << endl;
    return 1;
  }
  if (!(point.getPtB() - ptB).isZero()) {
    cerr << "Wrong ptB:" << endl;
    cerr << "  Expected " << ptB.transpose() << ", got "
         << point.getPtB().transpose() << endl;
    return 1;
  }
  return 0;
}

/*
 * Three bodies (cube (1 m edges) and two spheres (0.5 m radii) arranged like
 *this
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
int main() {
  Isometry3d T_body1_to_world, T_body2_to_world, T_body3_to_world,
      T_elem2_to_body;
  T_body1_to_world.setIdentity();
  T_body2_to_world.setIdentity();
  T_body3_to_world.setIdentity();
  T_body2_to_world.translation() << 1, 0, 0;
  T_elem2_to_body.setIdentity();
  T_elem2_to_body.translation() << 1, 0, 0;
  T_body3_to_world.translation() << 2, 2, 0;
  T_body3_to_world.matrix().block<2, 2>(0, 0) << 0, -1, 1,
      0;  // rotate 90 deg in z

  shared_ptr<Model> model = newModel();
  ElementId id1, id2, id3;
  id1 = model->addElement(Element(DrakeShapes::Box(Vector3d(1, 1, 1))));
  id2 = model->addElement(Element(DrakeShapes::Sphere(0.5), T_elem2_to_body));
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
  out = testPointPair(points[0], 1.0, Vector3d(-1, 0, 0), id1, id2,
                      Vector3d(0.5, 0, 0), Vector3d(.5, 0, 0));
  if (out == 0) {
    cout << "Check points[1] ..." << endl;
    out = testPointPair(
        points[1], 1.6213203435596428, Vector3d(-sqrt(2) / 2, -sqrt(2) / 2, 0),
        id1, id3, Vector3d(.5, .5, 0), Vector3d(-sqrt(2) / 4, sqrt(2) / 4, 0));
  }
  if (out == 0) {
    cout << "Check points[2] ..." << endl;
    out = testPointPair(points[2], 1.0, Vector3d(0, -1, 0), id2, id3,
                        Vector3d(1, .5, 0), Vector3d(-.5, 0, 0));
  }
  cout << out << endl;
  return out;
}
