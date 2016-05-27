// Get an example of FCL Collision detection working compare it with the
// BulletModel.

#include <iostream>
#include <cmath>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision_data.h"
#include "fcl/distance.h"
#include "fcl/collision.h"
#include "fcl/collision_node.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"

#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/collision/fcl_model.h"

using test_func_t =
    bool (*)(const fcl::Transform3f&,
             const std::vector<fcl::Vec3f>&, const std::vector<fcl::Triangle>&,
             const std::vector<fcl::Vec3f>&, const std::vector<fcl::Triangle>&,
             fcl::SplitMethodType, bool);

// FCL only stuff to explore its behavior.

// Basic shapes (tested box-box)
// distance goes negative when in collision, but closest points are wrong.
// Collision point and penetration depth are wrong/ not computed.
// Distance works with negative depth, but closest point is wrong when in
// collision.
// Closest point computed individually. The two are not the closest pair.

// RSS: collide has similar problems to basic shapes.
// OBB: collide works.
// OBB: distance works.

void TestBoxFCLDistance(double tx, double ty, double tz) {
  fcl::Box s1(20, 40, 60);
  fcl::Box s2(10, 10, 10);

  fcl::BVHModel<fcl::OBBRSS> s1_rss;
  fcl::BVHModel<fcl::OBBRSS> s2_rss;

  fcl::generateBVHModel(s1_rss, s1, fcl::Transform3f());
  fcl::generateBVHModel(s2_rss, s2, fcl::Transform3f());

  fcl::DistanceRequest request;
  request.enable_nearest_points = true;
  fcl::DistanceResult res;

  fcl::Transform3f pose;

  pose.setTranslation(fcl::Vec3f(tx, ty, tz));

  res.clear();
  fcl::distance(&s1_rss, fcl::Transform3f(), &s2_rss, pose, request, res);

  std::cout << "distance" << std::endl;
  std::cout << "translate (" << tx << "," << ty << "," << tz
            << "): " << res.min_distance << std::endl;
  std::cout << "     ("
            << res.nearest_points[0][0] << ", "
            << res.nearest_points[0][1] << ", "
            << res.nearest_points[0][2] << "), ";
  std::cout << "("
            << res.nearest_points[1][0] << ", "
            << res.nearest_points[1][1] << ", "
            << res.nearest_points[1][2] << ")" << std::endl;
}

template <typename BV>
void TestBoxFCLCollideNode(double tx, double ty, double tz,
                           fcl::SplitMethodType split_method) {
  std::cout << "testBoxFCLCollideNode " << tx << ", " << ty << ", " << tz
            << std::endl;

  fcl::Box s1(20, 20, 20);
  fcl::Box s2(10, 10, 10);

  fcl::BVHModel<BV> s1_bv;
  fcl::BVHModel<BV> s2_bv;

  fcl::generateBVHModel(s1_bv, s1, fcl::Transform3f());
  fcl::generateBVHModel(s2_bv, s2, fcl::Transform3f());
  s1_bv.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));
  s2_bv.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));

  fcl::Transform3f pose1;
  fcl::Transform3f pose2;
  pose2.setTranslation(fcl::Vec3f(tx, ty, tz));

  fcl::CollisionResult res;
  int num_max_contacts = std::numeric_limits<int>::max();
  bool enable_contact = true;
  fcl::CollisionRequest request(num_max_contacts, enable_contact);
  fcl::MeshCollisionTraversalNode<BV> node;

  if (!fcl::initialize<BV>(
           node, s1_bv, pose1, s2_bv, pose2,
           fcl::CollisionRequest(num_max_contacts, enable_contact), res)) {
    std::cout << "initialize error" << std::endl;
  }

  node.enable_statistics = true;

  fcl::collide(&node);

  if (res.numContacts() > 0) {
    std::cout << "in collision " << res.numContacts() << ": " << std::endl;
    std::cout << "  ";
    for (int i = 0; i < res.numContacts(); ++i) {
      fcl::Contact c = res.getContact(i);
      std::cout << i << ":p=(" << c.pos[0] << "," << c.pos[1] << "," << c.pos[2]
                << ")";
      std::cout << "n=(" << c.normal[0] << "," << c.normal[1] << ","
                << c.normal[2] << ")";
      std::cout << "d=" << c.penetration_depth << ",  " << std::endl;
    }
    std::cout << "\n";
  } else {
    std::cout << "collision free " << std::endl;
  }
}

void TestBoxFCLCollide(double tx, double ty, double tz) {
  fcl::Box s1(10, 10, 10);
  fcl::Box s2(10, 10, 10);

  fcl::BVHModel<fcl::OBBRSS> s1_rss;
  fcl::BVHModel<fcl::OBBRSS> s2_rss;

  fcl::generateBVHModel(s1_rss, s1, fcl::Transform3f());
  fcl::generateBVHModel(s2_rss, s2, fcl::Transform3f());

  fcl::CollisionRequest request(4, true);
  fcl::CollisionResult res;

  fcl::Transform3f pose;

  pose.setTranslation(fcl::Vec3f(tx, ty, tz));

  res.clear();
  fcl::collide(&s1_rss, fcl::Transform3f(), &s2_rss, pose, request, res);

  std::cout << "collide" << std::endl;
  int num = res.numContacts();
  std::cout << "translate (" << tx << "," << ty << "," << tz << "): " << num
            << std::endl;
  for (int i = 0; i < num; ++i) {
    fcl::Contact c = res.getContact(i);
    // Let's see if we can get both collision points out of this structure?
    std::cout << "(" << c.pos[0] << "," << c.pos[1] << "," << c.pos[2] << "),"
              << "(" << c.normal[0] << "," << c.normal[1] << "," << c.normal[2]
              << ") : " << c.penetration_depth << std::endl;
  }
}

template <typename BV>
bool FCLCollideTest(const fcl::Transform3f& tf,
                    const std::vector<fcl::Vec3f>& vertices1,
                    const std::vector<fcl::Triangle>& triangles1,
                    const std::vector<fcl::Vec3f>& vertices2,
                    const std::vector<fcl::Triangle>& triangles2,
                    fcl::SplitMethodType split_method, bool verbose) {
  fcl::BVHModel<BV> m1;
  fcl::BVHModel<BV> m2;
  m1.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  fcl::Transform3f pose1(tf), pose2;

  fcl::CollisionResult local_result;
  fcl::MeshCollisionTraversalNode<BV> node;

  int num_max_contacts = std::numeric_limits<int>::max();
  bool enable_contact = true;

  if (!fcl::initialize<BV>(
           node, m1, pose1, m2, pose2,
           fcl::CollisionRequest(num_max_contacts, enable_contact),
           local_result)) {
    std::cout << "initialize error" << std::endl;
  }

  node.enable_statistics = verbose;

  fcl::collide(&node);

  if (local_result.numContacts() > 0) {

    if (verbose) {
      std::cout << "in collision " << local_result.numContacts() << ": "
                << std::endl;
      std::cout << "  ";
      for (int i = 0; i < local_result.numContacts(); ++i) {
        fcl::Contact c = local_result.getContact(i);
        std::cout << "p=(" << c.pos[0] << "," << c.pos[1] << "," << c.pos[2]
                  << ")";
        std::cout << "n=(" << c.normal[0] << "," << c.normal[1] << ","
                  << c.normal[2] << ")";
        std::cout << "d=" << c.penetration_depth << ",  ";
      }
      std::cout << "\n";
    }

    return true;
  } else {
    if (verbose) std::cout << "collision free " << std::endl;
    return false;
  }
}

void TestFCLMultiPoint(test_func_t test_func) {
  std::cout << "------------------------------------------------\n";
  std::vector<fcl::Vec3f> p1(8), p2(6);
  std::vector<fcl::Triangle> t1(12), t2(8);

  // A box z = 0 surface
  p1[0] = fcl::Vec3f(-100, -100, 0);
  p1[1] = fcl::Vec3f(+100, -100, 0);
  p1[2] = fcl::Vec3f(-100, +100, 0);
  p1[3] = fcl::Vec3f(+100, +100, 0);
  p1[4] = fcl::Vec3f(-100, -100, -20);
  p1[5] = fcl::Vec3f(+100, -100, -20);
  p1[6] = fcl::Vec3f(-100, +100, -20);
  p1[7] = fcl::Vec3f(+100, +100, -20);
  // xy
  t1[0] = fcl::Triangle(0, 1, 3);
  t1[1] = fcl::Triangle(0, 3, 2);
  t1[2] = fcl::Triangle(4, 7, 5);
  t1[3] = fcl::Triangle(4, 6, 7);
  // xz
  t1[4] = fcl::Triangle(0, 5, 1);
  t1[5] = fcl::Triangle(0, 4, 5);
  t1[6] = fcl::Triangle(2, 3, 7);
  t1[7] = fcl::Triangle(2, 7, 6);
  // yz
  t1[8] = fcl::Triangle(0, 2, 6);
  t1[9] = fcl::Triangle(0, 6, 4);
  t1[10] = fcl::Triangle(1, 7, 3);
  t1[11] = fcl::Triangle(1, 5, 7);

  // Double pyramid pointing down. (tips at (-1,0,0), (1,0,0))
  p2[0] = fcl::Vec3f(-1, 0, 0);
  p2[1] = fcl::Vec3f(+1, 0, 0);
  p2[2] = fcl::Vec3f(-5, 0, 10);
  p2[3] = fcl::Vec3f(+5, 0, 10);
  p2[4] = fcl::Vec3f(0, -5, 10);
  p2[5] = fcl::Vec3f(0, +5, 10);
  // base
  t2[0] = fcl::Triangle(2, 4, 5);
  t2[1] = fcl::Triangle(3, 5, 4);
  // pyramid 1
  t2[2] = fcl::Triangle(0, 2, 5);
  t2[3] = fcl::Triangle(0, 5, 4);
  t2[4] = fcl::Triangle(0, 4, 3);
  t2[5] = fcl::Triangle(1, 3, 4);
  t2[6] = fcl::Triangle(1, 4, 5);
  t2[7] = fcl::Triangle(1, 5, 3);

  bool verbose = true;

  // collision
  fcl::Transform3f transform1;
  fcl::Matrix3f R;
  R.setValue(1, 0, 0, 0, 1, 0, 0, 0, 1);
  // Vec3f T(0, 0, -1); // collision free
  fcl::Vec3f T(0, 0, 0.25);  // works but reports 6 contacts
  // Vec3f T(0, 0, 0.5); // bug
  transform1.setTransform(R, T);

  test_func(transform1, p1, t1, p2, t2, fcl::SPLIT_METHOD_MEAN, verbose);
  test_func(transform1, p1, t1, p2, t2, fcl::SPLIT_METHOD_BV_CENTER, verbose);
  test_func(transform1, p1, t1, p2, t2, fcl::SPLIT_METHOD_MEDIAN, verbose);
}

void TestFCLMultiPoint() {
  std::cout << "------------------------------------------------\n";
  TestFCLMultiPoint(&FCLCollideTest<fcl::OBB>);
  TestFCLMultiPoint(&FCLCollideTest<fcl::RSS>);
  TestFCLMultiPoint(&FCLCollideTest<fcl::AABB>);
  TestFCLMultiPoint(&FCLCollideTest<fcl::KDOP<24> >);
  TestFCLMultiPoint(&FCLCollideTest<fcl::KDOP<18> >);
  TestFCLMultiPoint(&FCLCollideTest<fcl::KDOP<16> >);
  TestFCLMultiPoint(&FCLCollideTest<fcl::kIOS>);
  TestFCLMultiPoint(&FCLCollideTest<fcl::OBBRSS>);
}

// DrakeCollision stuff

// Pass in either a BulletModel or an FCLModel
void TestBoxModel(DrakeCollision::Model& model,
                  double tx, double ty, double tz) {
  std::cout << "Box Model " << tx << ", " << ty << ", " << tz
            << "   -------------\n";

  Eigen::Isometry3d T_body1_to_world, T_body2_to_world, T_body3_to_world,
      T_elem2_to_body;
  T_body1_to_world.setIdentity();
  T_body1_to_world.translation() << 0, 0, 100;
  T_body2_to_world.setIdentity();
  T_body2_to_world.translation() << tx, ty, (100 + tz);

  DrakeCollision::ElementId id1, id2;
  id1 = model.addElement(
      DrakeCollision::Element(DrakeShapes::Box(Eigen::Vector3d(20, 20, 20))));
  id2 = model.addElement(
      DrakeCollision::Element(DrakeShapes::Box(Eigen::Vector3d(10, 10, 10))));
  model.updateElementWorldTransform(id1, T_body1_to_world);
  model.updateElementWorldTransform(id2, T_body2_to_world);

  std::vector<DrakeCollision::PointPair> points;
  std::vector<DrakeCollision::ElementId> ids_to_check;
  ids_to_check.push_back(id1);
  ids_to_check.push_back(id2);

  model.closestPointsAllToAll(ids_to_check, true, points);
  int num = points.size();
  std::cout << "NumPoints: " << num << std::endl;
  for (int i = 0; i < num; ++i) {
    auto const& p = points[i];
    std::cout << "  distance: " << p.getDistance() << std::endl;
    Eigen::Vector3d pta = p.getPtA();
    std::cout << "  pointA : \n" << pta << std::endl;
    Eigen::Vector3d ptb = p.getPtB();
    std::cout << "  pointB : \n" << ptb << std::endl;
    Eigen::Vector3d normal = p.getNormal();
    std::cout << "  normal : \n" << normal << std::endl;
  }
}

// Pass in either a BulletModel or an FCLModel
void TestSphereModel(DrakeCollision::Model& model,
                     double tx, double ty, double tz) {
  std::cout << "Box Model " << tx << ", " << ty << ", " << tz
            << "   -------------" << std::endl;

  Eigen::Isometry3d T_body1_to_world, T_body2_to_world, T_body3_to_world,
      T_elem2_to_body;
  T_body1_to_world.setIdentity();
  T_body1_to_world.translation() << 0, 0, 100;
  T_body2_to_world.setIdentity();
  T_body2_to_world.translation() << tx, ty, (100 + tz);

  DrakeCollision::ElementId id1, id2;
  id1 = model.addElement(DrakeCollision::Element(DrakeShapes::Sphere(20)));
  id2 = model.addElement(DrakeCollision::Element(DrakeShapes::Sphere(10)));
  model.updateElementWorldTransform(id1, T_body1_to_world);
  model.updateElementWorldTransform(id2, T_body2_to_world);

  std::vector<DrakeCollision::PointPair> points;
  std::vector<DrakeCollision::ElementId> ids_to_check;
  ids_to_check.push_back(id1);
  ids_to_check.push_back(id2);

  model.closestPointsAllToAll(ids_to_check, true, points);
  int num = points.size();
  std::cout << "NumPoints: " << num << std::endl;
  for (int i = 0; i < num; ++i) {
    auto const& p = points[i];
    std::cout << "  distance: " << p.getDistance() << std::endl;
    Eigen::Vector3d pta = p.getPtA();
    std::cout << "  pointA : \n" << pta << std::endl;
    Eigen::Vector3d ptb = p.getPtB();
    std::cout << "  pointB : \n" << ptb << std::endl;
    Eigen::Vector3d normal = p.getNormal();
    std::cout << "  normal : \n" << normal << std::endl;
  }
}

int main() {
  using DrakeCollision::newModel;

  std::unique_ptr<DrakeCollision::Model> bullet_model{
      newModel(DrakeCollision::BULLET)};
  std::unique_ptr<DrakeCollision::Model> fcl_model{
      newModel(DrakeCollision::FCL)};
  // testBoxModel(model, 14,14,14);
  // Just out of collision
  TestSphereModel(*bullet_model, 18, 18, 18);
  TestSphereModel(*fcl_model, 18, 18, 18);
  // Just in collision.
  TestSphereModel(*bullet_model, 16, 16, 16);
  TestSphereModel(*fcl_model, 16, 16, 16);

  // Compare bullet and FCL
  // TestBoxBulletModel(14, 14, 14);
  // TestBoxFCLModel(14, 14, 14);
  // TestBoxFCLCollideNode<OBBRSS>(14, 14, 14, SPLIT_METHOD_MEAN);

  /*
  TestBoxFCLDistance(10, 20, 30);
  TestBoxFCLDistance(14, 24, 34);
  TestBoxFCLDistance(16, 26, 36);
  TestBoxFCLDistance(20, 30, 40);
  */

  // compare the two
  // TestBoxBulletModel(10, 20, 30);
  // TestBoxBulletModel(19, 19, 19);
  // TestBoxBulletModel(15, 25, 35);
  // TestBoxBulletModel(16, 26, 36);
  // TestBoxBulletModel(20, 30, 40);
  /*
  cout << "FCLModel\n";
  // TestBoxFCLModel(10, 20, 30);
  TestBoxFCLModel(14, 24, 34);
  TestBoxFCLModel(15, 25, 35);
  TestBoxFCLModel(16, 26, 36);
  // TestBoxFCLModel(20, 30, 40);
  */

  /* I cannot get the second collision point of the pair
     and the normal / depth is not the max.
  TestBoxFCLCollide(10, 20, 30);
  TestBoxFCLCollide(14, 24, 34);
  TestBoxFCLCollide(16, 26, 36);
  TestBoxFCLCollide(20, 30, 40);
  */

  // TestBoxFCLCollide2<OBBRSS>(10, 20, 30, SPLIT_METHOD_MEAN);

  // TestFCLMultiPoint();
}
