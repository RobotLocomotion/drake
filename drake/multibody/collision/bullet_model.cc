#include "drake/multibody/collision/bullet_model.h"

#include <iostream>
#include <limits>
#include <utility>

#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/collision/drake_collision.h"

using Eigen::Isometry3d;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Vector3i;
using Eigen::Matrix3Xi;

namespace DrakeCollision {

// Helper method to convert a btVector3 to an Eigen vector representation.
// Using Eigen::Map avoids unnecessary (expensive) copies.
Eigen::Map<const Vector3d> toVector3d(const btVector3& bt_vec) {
  return Eigen::Map<const Vector3d>(bt_vec.m_floats);
}

static const int kPerturbationIterations = 8;
static const int kMinimumPointsPerturbationThreshold = 8;

namespace {
// Converts between two representations of a pose.
btTransform convert(const Isometry3d &T) {
  btTransform btT;
  btMatrix3x3 rot;
  btVector3 pos;

  rot.setValue(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0),
               T(2, 1), T(2, 2));
  btT.setBasis(rot);
  pos.setValue(T(0, 3), T(1, 3), T(2, 3));
  btT.setOrigin(pos);
  return btT;
}
}  // namespace

struct BinaryContactResultCallback
    : public btCollisionWorld::ContactResultCallback {
 public:
  BinaryContactResultCallback() { in_collision = false; }

  bool isInCollision() { return in_collision; }

  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual btScalar addSingleResult(btManifoldPoint& cp,
                                   const btCollisionObjectWrapper* colObj0Wrap,
                                   int partId0, int index0,
                                   const btCollisionObjectWrapper* colObj1Wrap,
                                   int partId1, int index1) {
    in_collision = true;
    return 0;
  }

 private:
  bool in_collision;
};

bool OverlapFilterCallback::needBroadphaseCollision(
    btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const {
  bool collides =
      (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
  collides = collides &&
             (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

  // add some additional logic here that modified 'collides'
  if (collides) {
    btCollisionObject* bt_collision_object0 =
        reinterpret_cast<btCollisionObject*>(proxy0->m_clientObject);
    btCollisionObject* bt_collision_object1 =
        reinterpret_cast<btCollisionObject*>(proxy1->m_clientObject);
    if ((bt_collision_object0->getUserPointer() != NULL) &&
        (bt_collision_object1->getUserPointer() != NULL)) {
      auto element0 =
          static_cast<Element*>(bt_collision_object0->getUserPointer());
      auto element1 =
          static_cast<Element*>(bt_collision_object1->getUserPointer());
      collides = collides && element0->CanCollideWith(element1);
    }
  }
  return collides;
}

BulletCollisionWorldWrapper::BulletCollisionWorldWrapper()
    : bt_collision_configuration(),
      bt_collision_broadphase(),
      filter_callback() {
  bt_collision_configuration.setConvexConvexMultipointIterations(0, 0);
  bt_collision_configuration.setPlaneConvexMultipointIterations(0, 0);
  bt_collision_dispatcher = std::unique_ptr<btCollisionDispatcher>(
      new btCollisionDispatcher(&bt_collision_configuration));
  bt_collision_world = std::unique_ptr<btCollisionWorld>(new btCollisionWorld(
      bt_collision_dispatcher.get(), &bt_collision_broadphase,
      &bt_collision_configuration));

  bt_collision_world->getPairCache()->setOverlapFilterCallback(
      &filter_callback);
}

std::unique_ptr<btCollisionShape> BulletModel::newBulletBoxShape(
    const DrakeShapes::Box& geometry, bool use_margins) {
  std::unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
  btBoxShape bt_box(btVector3(geometry.size(0) / 2, geometry.size(1) / 2,
                              geometry.size(2) / 2));
  /* Strange things happen to the collision-normals when we use the
   * convex interface to the btBoxShape. Instead, we'll explicitly create
   * a btConvexHullShape.
   */
  if (use_margins)
    bt_shape->setMargin(kLargeMargin);
  else
    bt_shape->setMargin(kSmallMargin);
  for (int i = 0; i < 8; ++i) {
    btVector3 vtx;
    bt_box.getVertex(i, vtx);
    dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(vtx);
  }

  return bt_shape;
}

std::unique_ptr<btCollisionShape> BulletModel::newBulletSphereShape(
    const DrakeShapes::Sphere& geometry, bool use_margins) {
  std::unique_ptr<btCollisionShape> bt_shape(
      new btSphereShape(geometry.radius));
  return bt_shape;
}

std::unique_ptr<btCollisionShape> BulletModel::newBulletCylinderShape(
    const DrakeShapes::Cylinder& geometry, bool use_margins) {
  std::unique_ptr<btCollisionShape> bt_shape(new btCylinderShapeZ(
      btVector3(geometry.radius, geometry.radius, geometry.length / 2)));
  return bt_shape;
}

std::unique_ptr<btCollisionShape> BulletModel::newBulletCapsuleShape(
    const DrakeShapes::Capsule& geometry, bool use_margins) {
  std::unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
  dynamic_cast<btConvexHullShape*>(bt_shape.get())
      ->addPoint(btVector3(0, 0, -geometry.length / 2));
  dynamic_cast<btConvexHullShape*>(bt_shape.get())
      ->addPoint(btVector3(0, 0, geometry.length / 2));
  bt_shape->setMargin(geometry.radius);
  return bt_shape;
}

std::unique_ptr<btCollisionShape> BulletModel::newBulletMeshShape(
    const DrakeShapes::Mesh& geometry, bool use_margins) {
  std::unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
  Matrix3Xd vertices;
  if (geometry.extractMeshVertices(vertices)) {
    if (use_margins)
      bt_shape->setMargin(kLargeMargin);
    else
      bt_shape->setMargin(kSmallMargin);
    auto bt_convex_hull_shape =
        dynamic_cast<btConvexHullShape*>(bt_shape.get());
    for (int i = 0; i < vertices.cols(); i++) {
      bt_convex_hull_shape->addPoint(
          btVector3(vertices(0, i), vertices(1, i), vertices(2, i)), false);
    }
    bt_convex_hull_shape->recalcLocalAabb();

    return bt_shape;
  } else {
    return nullptr;
  }
}

std::unique_ptr<btCollisionShape> BulletModel::newBulletStaticMeshShape(
    const DrakeShapes::Mesh& geometry, bool use_margins) {

  // Gathers vertices and triangles from the mesh's file.
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;
  geometry.LoadObjFile(&vertices, &triangles,
                       DrakeShapes::Mesh::TriangulatePolicy::kTry);

  btTriangleMesh* mesh = new btTriangleMesh();
  // BulletModel takes ownership of the mesh because Bullet does not.
  bt_triangle_meshes_.emplace_back(mesh);

  // Preallocates memory.
  int num_triangles = static_cast<int>(triangles.size());
  int num_vertices = static_cast<int>(vertices.size());

  mesh->preallocateIndices(num_triangles);
  mesh->preallocateVertices(num_vertices);

  // Loads individual triangles.
  for (int itri = 0; itri <  num_triangles; ++itri) {
    Vector3i tri = triangles[itri];
    btVector3 vertex0(
        vertices[tri(0)](0), vertices[tri(0)](1), vertices[tri(0)](2));
    btVector3 vertex1(
        vertices[tri(1)](0), vertices[tri(1)](1), vertices[tri(1)](2));
    btVector3 vertex2(
        vertices[tri(2)](0), vertices[tri(2)](1), vertices[tri(2)](2));
    mesh->addTriangle(vertex0, vertex1, vertex2);
  }

  // Instantiates a Bullet collision object with a btBvhTriangleMeshShape shape.
  // btBvhTriangleMeshShape is a static-triangle mesh shape with
  // Bounding Volume Hierarchy optimization.
  bool useQuantizedAabbCompression = true;
  btBvhTriangleMeshShape* bvh_mesh_shape =
      new btBvhTriangleMeshShape(mesh, useQuantizedAabbCompression);
  std::unique_ptr<btCollisionShape> bt_shape(bvh_mesh_shape);

  // Sets margins.
  if (use_margins)
    bt_shape->setMargin(kLargeMargin);
  else
    bt_shape->setMargin(kSmallMargin);

  return bt_shape;
}

std::unique_ptr<btCollisionShape> BulletModel::newBulletMeshPointsShape(
    const DrakeShapes::MeshPoints& geometry, bool use_margins) {
  std::unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
  if (use_margins)
    bt_shape->setMargin(kLargeMargin);
  else
    bt_shape->setMargin(kSmallMargin);
  auto bt_convex_hull_shape = dynamic_cast<btConvexHullShape*>(bt_shape.get());
  for (int i = 0; i < geometry.points.cols(); i++) {
    bt_convex_hull_shape->addPoint(
        btVector3(geometry.points(0, i), geometry.points(1, i),
                  geometry.points(2, i)),
        false);
  }
  bt_convex_hull_shape->recalcLocalAabb();
  return bt_shape;
}

void BulletModel::DoAddElement(const Element& element) {
  ElementId id = element.getId();

  if (id != 0) {
    std::unique_ptr<btCollisionShape> bt_shape;
    std::unique_ptr<btCollisionShape> bt_shape_no_margin;
    switch (element.getShape()) {
      case DrakeShapes::BOX: {
        const auto box =
            static_cast<const DrakeShapes::Box&>(element.getGeometry());
        bt_shape = newBulletBoxShape(box, true);
        bt_shape_no_margin = newBulletBoxShape(box, false);
      } break;
      case DrakeShapes::SPHERE: {
        const auto sphere = static_cast<const DrakeShapes::Sphere&>(
            element.getGeometry());
        bt_shape = newBulletSphereShape(sphere, true);
        bt_shape_no_margin = newBulletSphereShape(sphere, false);
      } break;
      case DrakeShapes::CYLINDER: {
        const auto cylinder = static_cast<const DrakeShapes::Cylinder&>(
            element.getGeometry());
        bt_shape = newBulletCylinderShape(cylinder, true);
        bt_shape_no_margin = newBulletCylinderShape(cylinder, false);
      } break;
      case DrakeShapes::MESH: {
        const auto mesh =
            static_cast<const DrakeShapes::Mesh&>(element.getGeometry());
        if (element.is_anchored()) {
          // Meshes are only allowed for anchored geometry.
          bt_shape = newBulletStaticMeshShape(mesh, true);
          bt_shape_no_margin = newBulletStaticMeshShape(mesh, false);
        } else {  // A convex hull representation of the mesh points.
          bt_shape = newBulletMeshShape(mesh, true);
          bt_shape_no_margin = newBulletMeshShape(mesh, false);
        }
      } break;
      case DrakeShapes::MESH_POINTS: {
        const auto mesh = static_cast<const DrakeShapes::MeshPoints&>(
            element.getGeometry());
        bt_shape = newBulletMeshPointsShape(mesh, true);
        bt_shape_no_margin = newBulletMeshPointsShape(mesh, false);
      } break;
      case DrakeShapes::CAPSULE: {
        const auto capsule = static_cast<const DrakeShapes::Capsule&>(
            element.getGeometry());
        bt_shape = newBulletCapsuleShape(capsule, true);
        bt_shape_no_margin = newBulletCapsuleShape(capsule, false);
      } break;
      default:
        std::cerr << "Warning: Collision elements[id] has an unknown type "
                  << element.getShape() << std::endl;
        throw UnknownShapeException(element.getShape());
        break;
    }
    if (bt_shape) {
      // Create the actual Bullet collision objects.
      std::unique_ptr<btCollisionObject> bt_obj(new btCollisionObject());
      std::unique_ptr<btCollisionObject> bt_obj_no_margin(
          new btCollisionObject());
      bt_obj->setCollisionShape(bt_shape.get());
      bt_obj_no_margin->setCollisionShape(bt_shape_no_margin.get());
      bt_obj->setUserPointer(elements[id].get());
      bt_obj_no_margin->setUserPointer(elements[id].get());

      // Here bit masks are set so that static collision elements are not even
      // checked for collisions between them.
      //
      // From Bullet's user manual, Ch. 5:
      // Bullet provides three easy ways to ensure that only certain objects
      // collide with each other: masks, broadphase filter callbacks and
      // nearcallbacks. It is worth noting that mask-based collision selection
      // happens a lot further up the toolchain than the callbacks do. In short,
      // if masks are sufficient for your purposes, use them; they perform
      // better and are a lot simpler to use.
      //
      // Bullet's collision filtering model assigns two independent sets of
      // "groups" to each body:
      //  - the set of groups the body belongs to, and
      //  - the set of groups the body can collide with.
      // A pair of bodies A and B are checked for possible collision only if A
      // belongs to a group B can collide with and B belongs to a group A can
      // collide with.
      // Groups are identified with small integers. The sets are represented as
      // bitsets using the group number as a bit position. Using short provides
      // up to 16 groups.
      // A body thus has two shorts:
      //   - 'group' has a bit set for each group the body belongs to, and
      //   - 'mask' has a bit set for each group the body can collide with.
      // So A and B can collide if A.group & B.mask and B.group & A.mask are
      // both non-zero.
      //
      // Notes:
      //   1. In general it is not true that group = ~mask.
      //   2. The exclusive or operator (^) is an easy way to turn on/off
      //      specific bits (since A^0 = A and A^1 = ~A).

      bool is_dynamic = !element.is_anchored();
      short collision_filter_group =  is_dynamic?    // NOLINT(runtime/int)
          // NOLINTNEXTLINE(runtime/int)
          static_cast<short>(btBroadphaseProxy::DefaultFilter) :
          // NOLINTNEXTLINE(runtime/int)
          static_cast<short>(btBroadphaseProxy::StaticFilter);
      short collision_filter_mask = is_dynamic?  // NOLINT(runtime/int)
          // NOLINTNEXTLINE(runtime/int)
          static_cast<short>(btBroadphaseProxy::AllFilter) :
          // The exclusive or flips the one bit position corresponding to the
          // StaticFilter group.
          // NOLINTNEXTLINE(runtime/int)
          static_cast<short>(
             btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

      // NOTE: The bullet collision object is assigned the Drake element's
      // world transform.  This will be the *only* time that anchored collision
      // objects will have their world transform set.  This code assumes that
      // the world transform on the corresponding input Drake element has
      // already been properly set. (See RigidBodyTree::CompileCollisionState.)
      btTransform btT = convert(element.getWorldTransform());
      bt_obj->setWorldTransform(btT);
      bt_obj_no_margin->setWorldTransform(btT);
      bullet_world_.bt_collision_world->
          addCollisionObject(bt_obj.get(),
                             collision_filter_group, collision_filter_mask);

      bullet_world_no_margin_.bt_collision_world->addCollisionObject(
          bt_obj_no_margin.get(), collision_filter_group,
          collision_filter_mask);

      // Take ownership of the Bullet collision objects.
      bullet_world_.bt_collision_objects.insert(
          std::make_pair(id, move(bt_obj)));
      bullet_world_no_margin_.bt_collision_objects.insert(
          std::make_pair(id, move(bt_obj_no_margin)));

      // Take ownership of the Bullet collision shapes too, because Bullet
      // does no cleanup.
      bt_collision_shapes_.push_back(move(bt_shape));
      bt_collision_shapes_.push_back(move(bt_shape_no_margin));
    }
  }
}

std::vector<PointPair> BulletModel::potentialCollisionPoints(bool use_margins) {
  if (dispatch_method_in_use_ == kNotYetDecided) {
    dispatch_method_in_use_ = kPotentialCollisionPoints;
  } else if (dispatch_method_in_use_ != kPotentialCollisionPoints) {
    throw std::runtime_error(
        "Calling BulletModel::potentialCollisionPoints after previously using"
            " another dispatch method will result in undefined behavior.");
  }

  BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);
  bt_world.bt_collision_configuration.setConvexConvexMultipointIterations(
      kPerturbationIterations, kMinimumPointsPerturbationThreshold);
  bt_world.bt_collision_configuration.setPlaneConvexMultipointIterations(
      kPerturbationIterations, kMinimumPointsPerturbationThreshold);
  std::vector<PointPair> point_pairs;
  bt_world.bt_collision_world->performDiscreteCollisionDetection();
  int numManifolds =
      bt_world.bt_collision_world->getDispatcher()->getNumManifolds();

  for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contact_manifold =
        bt_world.bt_collision_world->getDispatcher()
            ->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = contact_manifold->getBody0();
    const btCollisionObject* obB = contact_manifold->getBody1();

    auto elementA = static_cast<Element*>(obA->getUserPointer());
    auto elementB = static_cast<Element*>(obB->getUserPointer());

    DrakeShapes::Shape shapeA = elementA->getShape();
    DrakeShapes::Shape shapeB = elementB->getShape();

    ElementId idA = elementA->getId();
    ElementId idB = elementB->getId();

    double marginA = 0;
    double marginB = 0;

    if (shapeA == DrakeShapes::MESH || shapeA == DrakeShapes::BOX) {
      marginA = obA->getCollisionShape()->getMargin();
    }
    if (shapeB == DrakeShapes::MESH || shapeB == DrakeShapes::BOX) {
      marginB = obB->getCollisionShape()->getMargin();
    }
    int num_contacts = contact_manifold->getNumContacts();

    for (int j = 0; j < num_contacts; j++) {
      btManifoldPoint& pt = contact_manifold->getContactPoint(j);
      const btVector3& normal_on_B = pt.m_normalWorldOnB;
      const btVector3& point_on_A_in_world =
          pt.getPositionWorldOnA() + normal_on_B * marginA;
      const btVector3& point_on_B_in_world =
          pt.getPositionWorldOnB() - normal_on_B * marginB;
      const btVector3 point_on_elemA =
          obA->getWorldTransform().invXform(point_on_A_in_world);
      const btVector3 point_on_elemB =
          obB->getWorldTransform().invXform(point_on_B_in_world);

      auto point_on_A =
          elements[idA]->getLocalTransform() * toVector3d(point_on_elemA);
      auto point_on_B =
          elements[idB]->getLocalTransform() * toVector3d(point_on_elemB);

      point_pairs.emplace_back(
          elements[idA].get(), elements[idB].get(),
          point_on_A, point_on_B, toVector3d(normal_on_B),
          static_cast<double>(pt.getDistance()) + marginA + marginB);
    }
  }

  bt_world.bt_collision_configuration.setConvexConvexMultipointIterations(0, 0);
  bt_world.bt_collision_configuration.setPlaneConvexMultipointIterations(0, 0);
  return point_pairs;
}

bool BulletModel::collidingPointsCheckOnly(
    const std::vector<Vector3d>& input_points, double collision_threshold) {
  // Create sphere geometry
  btSphereShape bt_shape(collision_threshold);

  // Create Bullet collision object
  btCollisionObject bt_obj;
  bt_obj.setCollisionShape(static_cast<btCollisionShape*>(&bt_shape));

  btTransform btT;
  btT.setIdentity();
  BulletCollisionWorldWrapper& bt_world = getBulletWorld(false);

  for (size_t i = 0; i < input_points.size(); ++i) {
    BinaryContactResultCallback c;

    btVector3 pos(static_cast<btScalar>(input_points[i](0)),
                  static_cast<btScalar>(input_points[i](1)),
                  static_cast<btScalar>(input_points[i](2)));
    btT.setOrigin(pos);
    bt_obj.setWorldTransform(btT);

    bt_world.bt_collision_world->contactTest(&bt_obj, c);

    if (c.isInCollision()) {
      return true;
    }
  }

  return false;
}

std::vector<size_t> BulletModel::collidingPoints(
    const std::vector<Vector3d>& input_points, double collision_threshold) {
  // Create sphere geometry
  btSphereShape bt_shape(collision_threshold);

  // Create Bullet collision object
  btCollisionObject bt_obj;
  bt_obj.setCollisionShape(static_cast<btCollisionShape*>(&bt_shape));

  btTransform btT;
  btT.setIdentity();
  BulletCollisionWorldWrapper& bt_world = getBulletWorld(false);
  std::vector<size_t> in_collision_indices;

  for (size_t i = 0; i < input_points.size(); ++i) {
    BinaryContactResultCallback c;

    btVector3 pos(static_cast<btScalar>(input_points[i](0)),
                  static_cast<btScalar>(input_points[i](1)),
                  static_cast<btScalar>(input_points[i](2)));
    btT.setOrigin(pos);
    bt_obj.setWorldTransform(btT);

    bt_world.bt_collision_world->contactTest(&bt_obj, c);

    if (c.isInCollision()) {
      in_collision_indices.push_back(i);
    }
  }

  return in_collision_indices;
}

bool BulletModel::updateElementWorldTransform(
    ElementId id, const Isometry3d& T_local_to_world) {
  const bool element_exists(
      Model::updateElementWorldTransform(id, T_local_to_world));
  if (element_exists) {
    btTransform btT = convert(elements[id]->getWorldTransform());

    auto bt_obj_iter = bullet_world_.bt_collision_objects.find(id);
    auto bt_obj_no_margin_iter =
        bullet_world_no_margin_.bt_collision_objects.find(id);
    if (bt_obj_iter != bullet_world_.bt_collision_objects.end()) {
      bullet_world_.bt_collision_objects.at(id)->setWorldTransform(btT);
    }

    if (bt_obj_no_margin_iter !=
        bullet_world_no_margin_.bt_collision_objects.end()) {
      bullet_world_no_margin_.bt_collision_objects.at(id)
          ->setWorldTransform(btT);
    }
  }
  return element_exists;
}

void BulletModel::updateModel() {
  bullet_world_.bt_collision_world->updateAabbs();
  bullet_world_no_margin_.bt_collision_world->updateAabbs();
}

PointPair BulletModel::findClosestPointsBetweenElements(
    ElementId idA, ElementId idB, bool use_margins) {
  // special case: two spheres (because we need to handle the zero-radius sphere
  // case)
  if (elements[idA]->getShape() == DrakeShapes::SPHERE &&
      elements[idB]->getShape() == DrakeShapes::SPHERE) {
    const Isometry3d& TA_world = elements[idA]->getWorldTransform();
    const Isometry3d& TB_world = elements[idB]->getWorldTransform();
    auto xA_world = TA_world.translation();
    auto xB_world = TB_world.translation();
    double radiusA =
        dynamic_cast<const DrakeShapes::Sphere&>(elements[idA]->getGeometry())
            .radius;
    double radiusB =
        dynamic_cast<const DrakeShapes::Sphere&>(elements[idB]->getGeometry())
            .radius;
    double distance = (xA_world - xB_world).norm();
    return PointPair(
        elements[idA].get(), elements[idB].get(),
        elements[idA]->getLocalTransform() * TA_world.inverse() *
            (xA_world +
             (xB_world - xA_world) * radiusA /
                 distance),  // ptA (in body A coords)
        elements[idB]->getLocalTransform() * TB_world.inverse() *
            (xB_world +
             (xA_world - xB_world) * radiusB /
                 distance),  // ptB (in body B coords)
        (xA_world - xB_world) / distance,
        distance - radiusA - radiusB);
  }

  btConvexShape* shapeA;
  btConvexShape* shapeB;
  btGjkPairDetector::ClosestPointInput input;
  btPointCollector gjkOutput;

  BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);

  auto bt_objA_iter = bt_world.bt_collision_objects.find(idA);
  DRAKE_DEMAND(bt_objA_iter != bt_world.bt_collision_objects.end() &&
      "In BulletModel::findClosestPointsBetweenElements: "
          "invalid ElementId for body A.");

  auto bt_objB_iter = bt_world.bt_collision_objects.find(idB);
  DRAKE_DEMAND(bt_objB_iter != bt_world.bt_collision_objects.end() &&
        "In BulletModel::findClosestPointsBetweenElements: "
            "invalid ElementId for body B.");

  std::unique_ptr<btCollisionObject>& bt_objA = bt_objA_iter->second;
  std::unique_ptr<btCollisionObject>& bt_objB = bt_objB_iter->second;

  shapeA = dynamic_cast<btConvexShape*>(bt_objA->getCollisionShape());
  shapeB = dynamic_cast<btConvexShape*>(bt_objB->getCollisionShape());

  if (shapeA == nullptr || shapeB == nullptr) {
    throw std::logic_error(
        "Attempting to compute distance between two collision "
        "elements, at least one of which is non-convex.");
  }

  btGjkEpaPenetrationDepthSolver epa;
  btVoronoiSimplexSolver sGjkSimplexSolver;
  sGjkSimplexSolver.setEqualVertexThreshold(0.f);

  btGjkPairDetector convexConvex(shapeA, shapeB, &sGjkSimplexSolver, &epa);

  input.m_transformA = bt_objA->getWorldTransform();
  input.m_transformB = bt_objB->getWorldTransform();

  convexConvex.getClosestPoints(input, gjkOutput, 0);

  btVector3 pointOnAinWorld;
  btVector3 pointOnBinWorld;
  if (elements[idA]->getShape() == DrakeShapes::MESH ||
      elements[idA]->getShape() == DrakeShapes::MESH_POINTS ||
      elements[idA]->getShape() == DrakeShapes::BOX) {
    pointOnAinWorld = gjkOutput.m_pointInWorld +
                      gjkOutput.m_normalOnBInWorld *
                          (gjkOutput.m_distance + shapeA->getMargin());
  } else {
    pointOnAinWorld = gjkOutput.m_pointInWorld +
                      gjkOutput.m_normalOnBInWorld * gjkOutput.m_distance;
  }
  if (elements[idB]->getShape() == DrakeShapes::MESH ||
      elements[idB]->getShape() == DrakeShapes::MESH_POINTS ||
      elements[idB]->getShape() == DrakeShapes::BOX) {
    pointOnBinWorld = gjkOutput.m_pointInWorld -
                      gjkOutput.m_normalOnBInWorld * shapeB->getMargin();
  } else {
    pointOnBinWorld = gjkOutput.m_pointInWorld;
  }

  btVector3 point_on_elemA = input.m_transformA.invXform(pointOnAinWorld);
  btVector3 point_on_elemB = input.m_transformB.invXform(pointOnBinWorld);

  auto point_on_A =
      elements[idA]->getLocalTransform() * toVector3d(point_on_elemA);
  auto point_on_B =
      elements[idB]->getLocalTransform() * toVector3d(point_on_elemB);

  btScalar distance =
      gjkOutput.m_normalOnBInWorld.dot(pointOnAinWorld - pointOnBinWorld);

  if (gjkOutput.m_hasResult) {
    return PointPair(elements[idA].get(), elements[idB].get(),
                     point_on_A, point_on_B,
                     toVector3d(gjkOutput.m_normalOnBInWorld),
                     static_cast<double>(distance));
  } else {
    throw std::runtime_error(
        "In BulletModel::findClosestPointsBetweenElements: "
        "No closest point found between " +
        std::to_string(idA) + " and " + std::to_string(idB));
  }
}

void BulletModel::collisionDetectFromPoints(
    const Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>& closest_points) {
  closest_points.resize(points.cols());
  VectorXd phi(points.cols());

  btSphereShape shapeA(0.0);
  btConvexShape* shapeB;
  btGjkPairDetector::ClosestPointInput input;

  BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);

  // do collision check against all bodies for each point using bullet's
  // internal getclosestpoints solver
  for (int i = 0; i < points.cols(); i++) {
    bool got_one = false;
    for (auto bt_objB_iter = bt_world.bt_collision_objects.begin();
         bt_objB_iter != bt_world.bt_collision_objects.end(); bt_objB_iter++) {
      std::unique_ptr<btCollisionObject>& bt_objB = bt_objB_iter->second;
      btPointCollector gjkOutput;

      shapeB = dynamic_cast<btConvexShape*>(bt_objB->getCollisionShape());

      if (shapeB == nullptr) {
        // TODO(SeanCurtis-TRI): Eventually implement a solution to this for
        //  non-convex geometry.
        // This passes 0 as a dummy argument to disambiguate the overloaded
        // info logging method.
        drake::log()->info(
            "Attempting to compute distance between a point and a non-convex "
            "shape.", 0);
        continue;
      } else {
        btGjkEpaPenetrationDepthSolver epa;
        btVoronoiSimplexSolver sGjkSimplexSolver;
        sGjkSimplexSolver.setEqualVertexThreshold(0.f);
        btGjkPairDetector
            convexConvex(&shapeA, shapeB, &sGjkSimplexSolver, &epa);

        input.m_transformA =
            btTransform(btQuaternion(0, 0, 0, 1),
                        btVector3(points(0, i), points(1, i), points(2, i)));
        input.m_transformB = bt_objB->getWorldTransform();

        convexConvex.getClosestPoints(input, gjkOutput, 0);

        btVector3 pointOnAinWorld(points(0, i), points(1, i), points(2, i));
        btVector3 pointOnBinWorld = gjkOutput.m_pointInWorld;

        btScalar distance =
            gjkOutput.m_normalOnBInWorld.dot(pointOnAinWorld - pointOnBinWorld);

        if (gjkOutput.m_hasResult && (!got_one || distance < phi[i])) {
          btVector3 pointOnElemB = input.m_transformB.invXform(pointOnBinWorld);
          phi[i] = distance;
          got_one = true;
          Element *collision_element =
              static_cast<Element *>(bt_objB->getUserPointer());
          closest_points[i] =
              PointPair(collision_element, collision_element,
                        toVector3d(pointOnElemB), toVector3d(pointOnBinWorld),
                        toVector3d(gjkOutput.m_normalOnBInWorld), distance);
        }
      }
    }
    if (!got_one) {
      // Values used in the degenerate case of no closest points.
      constexpr double inf = std::numeric_limits<double>::infinity();
      const Vector3d inf_vector(0, 0, inf);
      const Vector3d default_norm(0, 0, 1);

      // In case there are no other objects found, we report a null object
      // infinitely far away.
      phi[i] = inf;
      closest_points[i] = PointPair();
      closest_points[i].distance = inf;
      closest_points[i].normal = default_norm;
      closest_points[i].ptA = inf_vector;
      closest_points[i].ptB = inf_vector;
    }
  }
}

bool BulletModel::collisionRaycast(const Matrix3Xd& origins,
                                   const Matrix3Xd& ray_endpoints,
                                   bool use_margins, VectorXd& distances,
                                   Matrix3Xd& normals) {
  distances.resize(ray_endpoints.cols());
  normals.resize(3, ray_endpoints.cols());

  BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);

  for (int i = 0; i < ray_endpoints.cols(); i++) {
    int origin_col = (origins.cols() > 1 ? i : 0);  // if a single origin is
                                                    // passed in, then use it
                                                    // for all raycasts
    btVector3 ray_from_world(origins(0, origin_col), origins(1, origin_col),
                             origins(2, origin_col));
    btVector3 ray_to_world(ray_endpoints(0, i), ray_endpoints(1, i),
                           ray_endpoints(2, i));

    // ClosestRayResultCallback inherits from RayResultCallback.
    btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from_world,
                                                            ray_to_world);

    // The user can specify options by setting the flag
    // RayResultCallback::m_flags.
    // This is demonstrated in the RaytestDemo
    // (bullet3/examples/Raycast/RaytestDemo.cpp) part of Bullet's
    // ExampleBrowser.
    //
    // Possible options are defined in the enum
    // btTriangleRaycastCallback::EFlags:
    // 1. kF_UseSubSimplexConvexCastRaytest
    // 2. kF_UseGjkConvexCastRaytest
    //
    // From the comments in this enum (btRaycastCallback.h) we know
    // (quoting those comments):
    // - SubSimplexConvexCastRaytest is the default.
    // - SubSimplexConvexCastRaytest uses an approximate but faster ray versus
    //   convex intersection algorithm.
    // We now know that SubSimplexConvexCastRaytest is an iterative algorithm
    // with a very large hardcoded tolerance for convergence, see discussions
    // on Drake's github repository issue #1712 and fix in PR #2354.
    // Erwin Coumans himself comments about this in Bullet's issue #34.
    //
    // When using SubSimplexConvexCastRaytest the ray test finally gets resolved
    // with the call to btSubsimplexConvexCast::calcTimeOfImpact() (this is the
    // iterative method with the hardcoded tolerance).
    // A nice discussion (and probably the only one out there) is referenced at
    // the top of the file, quote:
    // Typically the conservative advancement reaches solution in a few
    // iterations, clip it to 32 for degenerate cases. See discussion about this
    // here http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=565
    ray_callback.m_flags |=
        btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;

    bt_world.bt_collision_world->rayTest(ray_from_world, ray_to_world,
                                         ray_callback);

    if (ray_callback.hasHit()) {
      // compute distance to hit

      btVector3 end = ray_callback.m_hitPointWorld;

      Vector3d end_eigen(end.getX(), end.getY(), end.getZ());

      distances(i) = (end_eigen - origins.col(origin_col)).norm();

      btVector3 normal = ray_callback.m_hitNormalWorld;
      normals(0, i) = normal.getX();
      normals(1, i) = normal.getY();
      normals(2, i) = normal.getZ();
    } else {
      distances(i) = -1.;
      normals(0, i) = 0.;
      normals(1, i) = 0.;
      normals(2, i) = 0.;
    }
  }

  return true;
}

bool BulletModel::closestPointsAllToAll(
    const std::vector<ElementId>& ids_to_check, bool use_margins,
    std::vector<PointPair>& closest_points) {
  if (dispatch_method_in_use_ == kNotYetDecided)
    dispatch_method_in_use_ = kClosestPointsAllToAll;

  std::vector<ElementIdPair> id_pairs;
  for (size_t i = 0; i < ids_to_check.size(); ++i) {
    ElementId id_a = ids_to_check[i];
    const Element* element_a = FindElement(id_a);
    if (element_a != nullptr) {
      for (size_t j = i + 1; j < ids_to_check.size(); ++j) {
        ElementId id_b = ids_to_check[j];
        const Element* element_b = FindElement(id_b);
        if (element_b != nullptr && element_a->CanCollideWith(element_b)) {
          id_pairs.push_back(std::make_pair(id_a, id_b));
        }
      }
    }
  }
  return closestPointsPairwise(id_pairs, use_margins, closest_points);
}

bool BulletModel::closestPointsPairwise(
    const std::vector<ElementIdPair>& id_pairs, bool use_margins,
    std::vector<PointPair>& closest_points) {
  closest_points.clear();
  for (const ElementIdPair& pair : id_pairs) {
    try {
      closest_points.push_back(findClosestPointsBetweenElements(
          pair.first, pair.second, use_margins));
    } catch (std::logic_error& e) {
      drake::log()->warn(e.what());
    }
  }
  return closest_points.size() > 0;
}

bool BulletModel::ComputeMaximumDepthCollisionPoints(
    bool use_margins, std::vector<PointPair> &collision_points) {
  if (dispatch_method_in_use_ == kNotYetDecided)
    dispatch_method_in_use_ = kCollisionPointsAllToAll;

  collision_points.clear();
  MatrixXd normals;
  std::vector<double> distance;
  BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);

  // This removes the "persistent" behavior of Bullet's manifolds allowing to
  // perform a clean, from scratch, collision dispatch.
  ClearCachedResults(use_margins);

  // Internally updates AABB's calling btCollisionWorld::updateAabbs();
  // TODO(amcastro-tri): analyze if the call to BulletModel::updateModel() is
  // redundant (since all it does is to call btCollisionWorld::updateAabbs()).
  bt_world.bt_collision_world->performDiscreteCollisionDetection();
  int numManifolds =
      bt_world.bt_collision_world->getDispatcher()->getNumManifolds();
  for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold =
        bt_world.bt_collision_world->getDispatcher()
            ->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = contactManifold->getBody0();
    const btCollisionObject* obB = contactManifold->getBody1();
    auto elementA = static_cast<Element*>(obA->getUserPointer());
    auto elementB = static_cast<Element*>(obB->getUserPointer());
    DrakeShapes::Shape shapeA = elementA->getShape();
    DrakeShapes::Shape shapeB = elementB->getShape();
    double marginA = 0;
    double marginB = 0;
    if (shapeA == DrakeShapes::MESH || shapeA == DrakeShapes::MESH_POINTS ||
        shapeA == DrakeShapes::BOX) {
      marginA = obA->getCollisionShape()->getMargin();
    }
    if (shapeB == DrakeShapes::MESH || shapeB == DrakeShapes::MESH_POINTS ||
        shapeB == DrakeShapes::BOX) {
      marginB = obB->getCollisionShape()->getMargin();
    }
    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; j++) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      if (pt.getDistance() + marginA + marginB < 0.f) {
        const btVector3& normalOnB = pt.m_normalWorldOnB;
        const btVector3& ptA = pt.getPositionWorldOnA() + normalOnB * marginA;
        const btVector3& ptB = pt.getPositionWorldOnB() - normalOnB * marginB;

        collision_points.emplace_back(
            elementA, elementB,
            toVector3d(ptA), toVector3d(ptB), toVector3d(normalOnB),
            static_cast<double>(pt.getDistance() + marginA + marginB));
      }
    }
  }
  return collision_points.size() > 0;
}

void BulletModel::ClearCachedResults(bool use_margins) {
  BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);

  int numManifolds =
      bt_world.bt_collision_world->getDispatcher()->getNumManifolds();

  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold =
        bt_world.bt_collision_world->getDispatcher()
            ->getManifoldByIndexInternal(i);
    contactManifold->clearManifold();
  }
}

BulletCollisionWorldWrapper& BulletModel::getBulletWorld(bool use_margins) {
  if (use_margins) {
    return bullet_world_;
  } else {
    return bullet_world_no_margin_;
  }
}

UnknownShapeException::UnknownShapeException(DrakeShapes::Shape shape)
    : runtime_error("") {
  std::ostringstream ostr;
  ostr << shape;
  this->shape_name_ = ostr.str();
}

const char* UnknownShapeException::what() const throw() {
  return ("Unknown collision shape: " + shape_name_ +
          ". Ignoring this collision element")
      .c_str();
}

}  // namespace DrakeCollision
