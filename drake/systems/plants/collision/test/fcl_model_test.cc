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

using namespace DrakeCollision;
using namespace std;
using namespace fcl;
using namespace Eigen;

using test_func_t =
    bool (*)(const Transform3f&,
             const std::vector<Vec3f>&, const std::vector<Triangle>&,
             const std::vector<Vec3f>&, const std::vector<Triangle>&,
             SplitMethodType, bool);

// FCL only stuff to explore its behavior.

// Basic shapes (tested box-box)
// distance goes negative when in collision, but closest points are wrong.
// Collision point and penetration depth are wrong/ not computed.
// Distance works with negative depth, but closest point is wrond when in
// collision.
// Closest point computed individually. The two are not the closest pair.

// RSS: collide has similar problems to basic shapes.
// OBB: collide works.
// OBB: distance works.

void testBoxFCLDistance(float tx, float ty, float tz) {
  Box s1(20, 40, 60);
  Box s2(10, 10, 10);

  BVHModel<OBBRSS> s1_rss;
  BVHModel<OBBRSS> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3f());
  generateBVHModel(s2_rss, s2, Transform3f());

  DistanceRequest request;
  request.enable_nearest_points = true;
  DistanceResult res;

  Transform3f pose;

  pose.setTranslation(Vec3f(tx, ty, tz));

  res.clear();
  distance(&s1_rss, Transform3f(), &s2_rss, pose, request, res);

  cout << "distance" << endl;
  cout << "translate (" << tx << "," << ty << "," << tz
       << "): " << res.min_distance << endl;
  cout << "     (" << res.nearest_points[0][0] << ", "
       << res.nearest_points[0][1] << ", " << res.nearest_points[0][2] << "), ";
  cout << "(" << res.nearest_points[1][0] << ", " << res.nearest_points[1][1]
       << ", " << res.nearest_points[1][2] << ")\n";
}

template <typename BV>
void testBoxFCLCollideNode(float tx, float ty, float tz,
                           SplitMethodType split_method) {
  cout << "testBoxFCLCollideNode " << tx << ", " << ty << ", " << tz << endl;

  Box s1(20, 20, 20);
  Box s2(10, 10, 10);

  BVHModel<BV> s1_bv;
  BVHModel<BV> s2_bv;

  generateBVHModel(s1_bv, s1, Transform3f());
  generateBVHModel(s2_bv, s2, Transform3f());
  s1_bv.bv_splitter.reset(new BVSplitter<BV>(split_method));
  s2_bv.bv_splitter.reset(new BVSplitter<BV>(split_method));

  Transform3f pose1;
  Transform3f pose2;
  pose2.setTranslation(Vec3f(tx, ty, tz));

  CollisionResult res;
  int num_max_contacts = std::numeric_limits<int>::max();
  bool enable_contact = true;
  CollisionRequest request(num_max_contacts, enable_contact);
  MeshCollisionTraversalNode<BV> node;

  if (!initialize<BV>(node, s1_bv, pose1, s2_bv, pose2,
                      CollisionRequest(num_max_contacts, enable_contact),
                      res)) {
    std::cout << "initialize error" << std::endl;
  }

  node.enable_statistics = true;

  fcl::collide(&node);

  if (res.numContacts() > 0) {
    std::cout << "in collision " << res.numContacts() << ": " << std::endl;
    std::cout << "  ";
    for (int i = 0; i < res.numContacts(); ++i) {
      Contact c = res.getContact(i);
      std::cout << i << ":p=(" << c.pos[0] << "," << c.pos[1] << "," << c.pos[2]
                << ")";
      std::cout << "n=(" << c.normal[0] << "," << c.normal[1] << ","
                << c.normal[2] << ")";
      std::cout << "d=" << c.penetration_depth << ",  " << endl;
    }
    std::cout << "\n";
  } else {
    std::cout << "collision free " << std::endl;
  }
}

void testBoxFCLCollide(float tx, float ty, float tz) {
  Box s1(10, 10, 10);
  Box s2(10, 10, 10);

  BVHModel<OBBRSS> s1_rss;
  BVHModel<OBBRSS> s2_rss;

  generateBVHModel(s1_rss, s1, Transform3f());
  generateBVHModel(s2_rss, s2, Transform3f());

  CollisionRequest request(4, true);
  CollisionResult res;

  Transform3f pose;

  pose.setTranslation(Vec3f(tx, ty, tz));

  res.clear();
  collide(&s1_rss, Transform3f(), &s2_rss, pose, request, res);

  cout << "collide" << endl;
  int num = res.numContacts();
  cout << "translate (" << tx << "," << ty << "," << tz << "): " << num << endl;
  for (int i = 0; i < num; ++i) {
    Contact c = res.getContact(i);
    // Let's see if we can get bothe collision points out of this structure?
    cout << "(" << c.pos[0] << "," << c.pos[1] << "," << c.pos[2] << "),"
         << "(" << c.normal[0] << "," << c.normal[1] << "," << c.normal[2]
         << ") : " << c.penetration_depth << endl;
  }
}

template <typename BV>
bool fcl_collide_test(const Transform3f& tf,
                      const std::vector<Vec3f>& vertices1,
                      const std::vector<Triangle>& triangles1,
                      const std::vector<Vec3f>& vertices2,
                      const std::vector<Triangle>& triangles2,
                      SplitMethodType split_method, bool verbose) {
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3f pose1(tf), pose2;

  CollisionResult local_result;
  MeshCollisionTraversalNode<BV> node;

  int num_max_contacts = std::numeric_limits<int>::max();
  bool enable_contact = true;

  if (!initialize<BV>(node, m1, pose1, m2, pose2,
                      CollisionRequest(num_max_contacts, enable_contact),
                      local_result)) {
    std::cout << "initialize error" << std::endl;
  }

  node.enable_statistics = verbose;

  fcl::collide(&node);

  if (local_result.numContacts() > 0) {
    // std::vector<Contact> global_pairs;
    // global_pairs.clear();

    // local_result.getContacts(global_pairs);
    // std::sort(global_pairs.begin(), global_pairs.end());
    // for(std::size_t j = 0; j < global_pairs.size(); ++j) {
    //   cout << global_pairs[j].b1;
    //   cout << global_pairs[j].b2;
    // }

    if (verbose) {
      std::cout << "in collision " << local_result.numContacts() << ": "
                << std::endl;
      std::cout << "  ";
      for (int i = 0; i < local_result.numContacts(); ++i) {
        Contact c = local_result.getContact(i);
        std::cout << "p=(" << c.pos[0] << "," << c.pos[1] << "," << c.pos[2]
                  << ")";
        std::cout << "n=(" << c.normal[0] << "," << c.normal[1] << ","
                  << c.normal[2] << ")";
        std::cout << "d=" << c.penetration_depth << ",  ";
      }
      std::cout << "\n";
    }

    // if(verbose) {
    //   std::cout << node.num_bv_tests << " " << node.num_leaf_tests
    //             << std::endl;
    // }
    return true;
  } else {
    if (verbose) std::cout << "collision free " << std::endl;
    // if(verbose) {
    //   std::cout << node.num_bv_tests << " " << node.num_leaf_tests
    //             << std::endl;
    // }
    return false;
  }
}

void testFCLMultiPoint(test_func_t test_func) {
  std::cout << "------------------------------------------------\n";
  std::vector<Vec3f> p1(8), p2(6);
  std::vector<Triangle> t1(12), t2(8);

  // A box z = 0 surface
  p1[0] = Vec3f(-100, -100, 0);
  p1[1] = Vec3f(+100, -100, 0);
  p1[2] = Vec3f(-100, +100, 0);
  p1[3] = Vec3f(+100, +100, 0);
  p1[4] = Vec3f(-100, -100, -20);
  p1[5] = Vec3f(+100, -100, -20);
  p1[6] = Vec3f(-100, +100, -20);
  p1[7] = Vec3f(+100, +100, -20);
  // xy
  t1[0] = Triangle(0, 1, 3);
  t1[1] = Triangle(0, 3, 2);
  t1[2] = Triangle(4, 7, 5);
  t1[3] = Triangle(4, 6, 7);
  // xz
  t1[4] = Triangle(0, 5, 1);
  t1[5] = Triangle(0, 4, 5);
  t1[6] = Triangle(2, 3, 7);
  t1[7] = Triangle(2, 7, 6);
  // yz
  t1[8] = Triangle(0, 2, 6);
  t1[9] = Triangle(0, 6, 4);
  t1[10] = Triangle(1, 7, 3);
  t1[11] = Triangle(1, 5, 7);

  // Double pyramid pointing down. (tips at (-1,0,0), (1,0,0))
  p2[0] = Vec3f(-1, 0, 0);
  p2[1] = Vec3f(+1, 0, 0);
  p2[2] = Vec3f(-5, 0, 10);
  p2[3] = Vec3f(+5, 0, 10);
  p2[4] = Vec3f(0, -5, 10);
  p2[5] = Vec3f(0, +5, 10);
  // base
  t2[0] = Triangle(2, 4, 5);
  t2[1] = Triangle(3, 5, 4);
  // pyramid 1
  t2[2] = Triangle(0, 2, 5);
  t2[3] = Triangle(0, 5, 4);
  t2[4] = Triangle(0, 4, 3);
  t2[5] = Triangle(1, 3, 4);
  t2[6] = Triangle(1, 4, 5);
  t2[7] = Triangle(1, 5, 3);

  // boost::filesystem::path path(TEST_RESOURCES_DIR);
  // loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  // loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);
  // std::vector<Transform3f> transforms;
  // FCL_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  // std::size_t n = 10;
  bool verbose = true;

  // collision
  fcl::Transform3f transform1;
  fcl::Matrix3f R;
  // eulerToMatrix(0, 0, 0, R);
  R.setValue(1, 0, 0, 0, 1, 0, 0, 0, 1);
  // Vec3f T(0, 0, -1); // collision free
  Vec3f T(0, 0, 0.25);  // works but reports 6 contacts
  // Vec3f T(0, 0, 0.5); // bug
  transform1.setTransform(R, T);

  test_func(transform1, p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
  test_func(transform1, p1, t1, p2, t2, SPLIT_METHOD_BV_CENTER, verbose);
  test_func(transform1, p1, t1, p2, t2, SPLIT_METHOD_MEDIAN, verbose);
}

void testFCLMultiPoint() {
  std::cout << "------------------------------------------------\n";
  testFCLMultiPoint(&fcl_collide_test<OBB>);
  testFCLMultiPoint(&fcl_collide_test<RSS>);
  testFCLMultiPoint(&fcl_collide_test<AABB>);
  testFCLMultiPoint(&fcl_collide_test<KDOP<24> >);
  testFCLMultiPoint(&fcl_collide_test<KDOP<18> >);
  testFCLMultiPoint(&fcl_collide_test<KDOP<16> >);
  testFCLMultiPoint(&fcl_collide_test<kIOS>);
  testFCLMultiPoint(&fcl_collide_test<OBBRSS>);
}

// DrakeCollision stuff

// Pass in either a BulletModel or an FCLModel
void testBoxModel(shared_ptr<Model> model, float tx, float ty, float tz) {
  cout << "Box Model " << tx << ", " << ty << ", " << tz
       << "   -------------\n";

  Isometry3d T_body1_to_world, T_body2_to_world, T_body3_to_world,
      T_elem2_to_body;
  T_body1_to_world.setIdentity();
  T_body1_to_world.translation() << 0, 0, 100;
  T_body2_to_world.setIdentity();
  T_body2_to_world.translation() << tx, ty, (100 + tz);

  ElementId id1, id2;
  id1 = model->addElement(Element(DrakeShapes::Box(Vector3d(20, 20, 20))));
  id2 = model->addElement(Element(DrakeShapes::Box(Vector3d(10, 10, 10))));
  model->updateElementWorldTransform(id1, T_body1_to_world);
  model->updateElementWorldTransform(id2, T_body2_to_world);

  vector<PointPair> points;
  vector<ElementId> ids_to_check;
  ids_to_check.push_back(id1);
  ids_to_check.push_back(id2);

  model->closestPointsAllToAll(ids_to_check, true, points);
  int num = points.size();
  cout << "NumPoints: " << num << endl;
  for (int i = 0; i < num; ++i) {
    PointPair p = points[i];
    cout << "  distance: " << p.getDistance() << endl;
    Vector3d pta = p.getPtA();
    cout << "  pointA : \n" << pta << endl;
    Vector3d ptb = p.getPtB();
    cout << "  pointB : \n" << ptb << endl;
    Vector3d normal = p.getNormal();
    cout << "  normal : \n" << normal << endl;
  }
}

// Pass in either a BulletModel or an FCLModel
void testSphereModel(shared_ptr<Model> model, float tx, float ty, float tz) {
  cout << "Box Model " << tx << ", " << ty << ", " << tz
       << "   -------------\n";

  Isometry3d T_body1_to_world, T_body2_to_world, T_body3_to_world,
      T_elem2_to_body;
  T_body1_to_world.setIdentity();
  T_body1_to_world.translation() << 0, 0, 100;
  T_body2_to_world.setIdentity();
  T_body2_to_world.translation() << tx, ty, (100 + tz);

  ElementId id1, id2;
  id1 = model->addElement(Element(DrakeShapes::Sphere(20)));
  id2 = model->addElement(Element(DrakeShapes::Sphere(10)));
  model->updateElementWorldTransform(id1, T_body1_to_world);
  model->updateElementWorldTransform(id2, T_body2_to_world);

  vector<PointPair> points;
  vector<ElementId> ids_to_check;
  ids_to_check.push_back(id1);
  ids_to_check.push_back(id2);

  model->closestPointsAllToAll(ids_to_check, true, points);
  int num = points.size();
  cout << "NumPoints: " << num << endl;
  for (int i = 0; i < num; ++i) {
    PointPair p = points[i];
    cout << "  distance: " << p.getDistance() << endl;
    Vector3d pta = p.getPtA();
    cout << "  pointA : \n" << pta << endl;
    Vector3d ptb = p.getPtB();
    cout << "  pointB : \n" << ptb << endl;
    Vector3d normal = p.getNormal();
    cout << "  normal : \n" << normal << endl;
  }
}

int main() {
  // Default is BulletModel
  shared_ptr<Model> bullet_model = newModel();
  shared_ptr<Model> fcl_model(new FCLModel);
  // testBoxModel(model, 14,14,14);
  // Just out of collision
  testSphereModel(bullet_model, 18, 18, 18);
  testSphereModel(fcl_model, 18, 18, 18);
  // Just in collision.
  testSphereModel(bullet_model, 16, 16, 16);
  testSphereModel(fcl_model, 16, 16, 16);

  // Compare bullet and FCL
  // testBoxBulletModel(14, 14, 14);
  // testBoxFCLModel(14, 14, 14);
  // testBoxFCLCollideNode<OBBRSS>(14, 14, 14, SPLIT_METHOD_MEAN);

  /*
  testBoxFCLDistance(10, 20, 30);
  testBoxFCLDistance(14, 24, 34);
  testBoxFCLDistance(16, 26, 36);
  testBoxFCLDistance(20, 30, 40);
  */

  // compare the two
  // testBoxBulletModel(10, 20, 30);
  // testBoxBulletModel(19, 19, 19);
  // testBoxBulletModel(15, 25, 35);
  // testBoxBulletModel(16, 26, 36);
  // testBoxBulletModel(20, 30, 40);
  /*
  cout << "FCLModel\n";
  // testBoxFCLModel(10, 20, 30);
  testBoxFCLModel(14, 24, 34);
  testBoxFCLModel(15, 25, 35);
  testBoxFCLModel(16, 26, 36);
  // testBoxFCLModel(20, 30, 40);
  */

  /* I cannot get the second collision point of the pair
     and the normal / depth is not the max.
  testBoxFCLCollide(10, 20, 30);
  testBoxFCLCollide(14, 24, 34);
  testBoxFCLCollide(16, 26, 36);
  testBoxFCLCollide(20, 30, 40);
  */

  // testBoxFCLCollide2<OBBRSS>(10, 20, 30, SPLIT_METHOD_MEAN);

  // testFCLMultiPoint();
}
