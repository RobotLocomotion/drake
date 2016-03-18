#include <iostream>

//#include "drake/systems/plants/collision/DrakeCollision.h"
#include "FCLModel.h"
//#include "fcl/shape/geometric_shapes.h"
//#include "fcl/shape/geometric_shape_to_BVH_model.h"

#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision_data.h"
#include "fcl/distance.h"
#include "fcl/collision.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"

#include "drake/systems/plants/collision/DrakeCollision.h"

using namespace std;
using namespace fcl;
using namespace Eigen;

namespace DrakeCollision {

BVHModel<OBBRSS>* FCLModel::newFCLBoxShape(const DrakeShapes::Box& geometry,
                                           bool use_margins) {
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Box shape(geometry.size(0), geometry.size(1), geometry.size(2));
  fcl::BVHModel<OBBRSS>* bvh_shape = new fcl::BVHModel<OBBRSS>();
  fcl::generateBVHModel(*bvh_shape, shape, Transform3f());

  return bvh_shape;
}

BVHModel<OBBRSS>* FCLModel::newFCLSphereShape(
    const DrakeShapes::Sphere& geometry, bool use_margins) {
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Sphere shape(geometry.radius);
  fcl::BVHModel<OBBRSS>* bvh_shape = new fcl::BVHModel<OBBRSS>();
  int numLong = 10;
  int numLat = 8;
  fcl::generateBVHModel(*bvh_shape, shape, Transform3f(), numLong, numLat);

  return bvh_shape;
}

BVHModel<OBBRSS>* FCLModel::newFCLCylinderShape(
    const DrakeShapes::Cylinder& geometry, bool use_margins) {
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Cylinder shape(geometry.radius, geometry.length);
  fcl::BVHModel<OBBRSS>* bvh_shape = new fcl::BVHModel<OBBRSS>();
  int numLength = 1;
  int numTheta = 10;
  fcl::generateBVHModel(*bvh_shape, shape, Transform3f(), numTheta, numLength);

  return bvh_shape;
}

BVHModel<OBBRSS>* FCLModel::newFCLCapsuleShape(
    const DrakeShapes::Capsule& geometry, bool use_margins) {
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Capsule shape(geometry.radius, geometry.length);
  fcl::BVHModel<OBBRSS>* bvh_shape = new fcl::BVHModel<OBBRSS>();

  // fcl does not have a method to generate a capsule mesh.
  // TODO: Make one.
  // fcl::generateBVHModel(*bvh_shape, shape, Transform3f(), numTheta,
  // numLength);

  return bvh_shape;
}

// TODO: Figure out how to convert mesh verts to fcl mesh.
BVHModel<OBBRSS>* FCLModel::newFCLMeshShape(const DrakeShapes::Mesh& geometry,
                                            bool use_margins) {
  Matrix3Xd vertices;
  if (geometry.extractMeshVertices(vertices)) {
    // geometry.getPoints(Eigen::Matrix3Xd &points) const;
  }
  // What about traingle indexes?
  // TODL........
  cerr << "Warning: FCLModel::newMeshShape is not implemented." << endl;
  return 0;
}

ElementId FCLModel::addElement(const Element& element) {
  ElementId id = Model::addElement(element);

  if (id != 0) {
    BVHModel<OBBRSS>* bvh_shape = 0;
    switch (elements[id]->getShape()) {
      case DrakeShapes::BOX: {
        const auto box =
            static_cast<const DrakeShapes::Box&>(elements[id]->getGeometry());
        bvh_shape = newFCLBoxShape(box, true);
      } break;
      case DrakeShapes::SPHERE: {
        const auto sphere = static_cast<const DrakeShapes::Sphere&>(
            elements[id]->getGeometry());
        bvh_shape = newFCLSphereShape(sphere, true);
      } break;
      case DrakeShapes::CYLINDER: {
        const auto cylinder = static_cast<const DrakeShapes::Cylinder&>(
            elements[id]->getGeometry());
        bvh_shape = newFCLCylinderShape(cylinder, true);
      } break;
      case DrakeShapes::MESH: {
        const auto mesh =
            static_cast<const DrakeShapes::Mesh&>(elements[id]->getGeometry());
        // I only see points.  I cannot find triangles.
        // bt_shape = newBulletMeshShape(mesh, true);
        // bt_shape_no_margin = newBulletMeshShape(mesh, false);
        cerr << "Warning: FCLMesh not implemented." << endl;
      } break;
      case DrakeShapes::MESH_POINTS: {
        const auto mesh = static_cast<const DrakeShapes::MeshPoints&>(
            elements[id]->getGeometry());
        // bt_shape = newBulletMeshPointsShape(mesh, true);
        // bt_shape_no_margin = newBulletMeshPointsShape(mesh, false);
        cerr << "Warning: FCLMeshPoints not implemented." << endl;
      } break;
      case DrakeShapes::CAPSULE: {
        const auto capsule = static_cast<const DrakeShapes::Capsule&>(
            elements[id]->getGeometry());
        bvh_shape = newFCLCapsuleShape(capsule, true);
        cerr << "Warning: FCLCapsule not implemented." << endl;
      } break;
      default:
        cerr << "Warning: Collision elements[id] has an unknown type "
             << elements[id]->getShape() << endl;
        throw unknownShapeException(elements[id]->getShape());
        break;
    }
    if (bvh_shape) {
      this->fclElements.insert(
          make_pair(id, unique_ptr<BVHModel<OBBRSS>>(bvh_shape)));
    }
  }
  return id;
}

vector<PointPair> FCLModel::potentialCollisionPoints(bool use_margins) {
  ResultCollector c;
  // TODO: Not Implemented
  cerr << "Warning: FCLModel::potentialCollisionPoints not implemented."
       << endl;
  return c.getResults();
}

bool FCLModel::collidingPointsCheckOnly(const vector<Vector3d>& points,
                                        double collision_threshold) {
  // TODO: Not Implemented
  cerr << "Warning: FCLModel::collisionPointsCheckOnly not implemented."
       << endl;
  // if (c.isInCollision()) {
  //  return true;
  //}

  return false;
}

vector<size_t> FCLModel::collidingPoints(const vector<Vector3d>& points,
                                         double collision_threshold) {
  cerr << "Warning: FCLModel::collidingPoints not implemented." << endl;
  vector<size_t> in_collision_indices;

  for (size_t i = 0; i < points.size(); ++i) {
    // if (c.isInCollision()) {
    //  in_collision_indices.push_back(i);
    //}
  }

  return in_collision_indices;
}

bool FCLModel::updateElementWorldTransform(const ElementId id,
                                           const Isometry3d& T_local_to_world) {
  // Since the transform is applide during collision, this method may not need
  // to be
  // implemented.
  cerr << "Warning: FCLModel::updateElementWorldTransform not implemented."
       << endl;

  const bool element_exists(
      Model::updateElementWorldTransform(id, T_local_to_world));
  if (element_exists) {
    // const Isometry3d& T = elements[id]->getWorldTransform();
    // btMatrix3x3 rot;
    // btVector3 pos;
    // btTransform btT;

    // rot.setValue( T(0,0), T(0,1), T(0,2),
    //              T(1,0), T(1,1), T(1,2),
    //              T(2,0), T(2,1), T(2,2) );
    // btT.setBasis(rot);
    // pos.setValue( T(0,3), T(1,3), T(2,3) );
    // btT.setOrigin(pos);
    // ...
  }
  return element_exists;
}

void FCLModel::updateModel() {
  // I am not sure if this needs to be implemented.
  cerr << "Warning: FCLModel::updateModel not implemented." << endl;
  // NOTE: ccl: I am not sure what cahnges I have to accomadate.
  // Should I rebuild the fclModels?
  // TODO: Test this.
}

bool FCLModel::findClosestPointsBtwElements(
    const ElementId idA, const ElementId idB, const bool use_margins,
    std::unique_ptr<ResultCollector>& c) {
  // For FCL we are converting basic shapes into meshes.
  // This is the same special case in BulletModel.
  // TODO: Consider sharing code.
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
    c->addSingleResult(idA, idB,
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
    return true;
  }

  DistanceRequest request;
  request.enable_nearest_points = true;
  DistanceResult res;

  const Isometry3d& TA_world = elements[idA]->getWorldTransform();
  const Isometry3d& TB_world = elements[idB]->getWorldTransform();

  // Convert the Eigen transform into the fcl transform.
  Transform3f tfa(fcl::Matrix3f(TA_world(0, 0), TA_world(0, 1), TA_world(0, 2),
                                TA_world(1, 0), TA_world(1, 1), TA_world(1, 2),
                                TA_world(2, 0), TA_world(2, 1), TA_world(2, 2)),
                  fcl::Vec3f(TA_world(0, 3), TA_world(1, 3), TA_world(2, 3)));
  Transform3f tfb(fcl::Matrix3f(TB_world(0, 0), TB_world(0, 1), TB_world(0, 2),
                                TB_world(1, 0), TB_world(1, 1), TB_world(1, 2),
                                TB_world(2, 0), TB_world(2, 1), TB_world(2, 2)),
                  fcl::Vec3f(TB_world(0, 3), TB_world(1, 3), TB_world(2, 3)));

  fcl::distance(fclElements[idA].get(), tfa, fclElements[idB].get(), tfb,
                request, res);
  auto dist = res.min_distance;
  fcl::Vec3f fclPt;

  if (dist > 0) {
    // FCL reports closest points in world coordinates.
    // drake returns them in element coordinates.

    tfa.inverse();
    fclPt = tfa.transform(res.nearest_points[0]);
    Vector3d ptA(fclPt[0], fclPt[1], fclPt[2]);

    tfb.inverse();
    fclPt = tfb.transform(res.nearest_points[1]);
    Vector3d ptB(fclPt[0], fclPt[1], fclPt[2]);

    c->addSingleResult(idA, idB, ptA, ptB, (ptA - ptB) / dist, dist);
  } else {
    // fcl::distance does work when elements are in collision.
    int num_max_contacts = std::numeric_limits<int>::max();
    bool enable_contact = true;
    CollisionRequest cRequest(num_max_contacts, enable_contact);
    CollisionResult cRes;
    fcl::collide(fclElements[idA].get(), tfa, fclElements[idB].get(), tfb,
                 cRequest, cRes);

    tfa.inverse();
    tfb.inverse();

    int num = cRes.numContacts();
    for (int i = 0; i < num; ++i) {
      Contact contact = cRes.getContact(i);
      // if (c.o1 == fclElements[idA].get()) {
      fclPt = tfa.transform(contact.pos);
      Vector3d ptA(fclPt[0], fclPt[1], fclPt[2]);
      auto dist = contact.penetration_depth;
      fclPt = tfb.transform(contact.pos - (contact.normal * dist));
      Vector3d ptB(fclPt[0], fclPt[1], fclPt[2]);
      Vector3d normal(contact.normal[0], contact.normal[1], contact.normal[2]);
      c->addSingleResult(idA, idB, ptA, ptB, normal, dist);
    }
  }

  return (c->pts.size() > 0);
}

void FCLModel::collisionDetectFromPoints(
    const Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>& closest_points) {
  closest_points.resize(
      points.cols(), PointPair(0, 0, Vector3d(), Vector3d(), Vector3d(), 0.0));
  VectorXd phi(points.cols());

  cerr << "Warning: FCLModel::collisionDetectFromPoints not implemented."
       << endl;
  //...

  // do collision check against all bodies for each point using bullet's
  // internal getclosestpoitns solver
  for (int i = 0; i < points.cols(); i++) {
    // bool got_one = false;
    //...

    // closest_points[i] = PointPair(
    //          bt_objB_iter->first, bt_objB_iter->first,
    //          toVector3d(pointOnElemB), toVector3d(pointOnBinWorld),
    //          toVector3d(gjkOutput.m_normalOnBInWorld), distance);
  }
}

bool FCLModel::collisionRaycast(const Matrix3Xd& origins,
                                const Matrix3Xd& ray_endpoints,
                                bool use_margins, VectorXd& distances,
                                Matrix3Xd& normals) {
  distances.resize(origins.cols());
  normals.resize(3, origins.cols());

  cerr << "Warning: FCLModel::collisionRaycast not implemented." << endl;

  //...
  for (int i = 0; i < origins.cols(); i++) {
    //...
  }

  return true;
}

// This is the same implementation as in bullet.
// TODO: consider sharing code.
bool FCLModel::closestPointsAllToAll(const vector<ElementId>& ids_to_check,
                                     const bool use_margins,
                                     vector<PointPair>& closest_points) {
  vector<ElementIdPair> id_pairs;
  const auto elements_end = elements.end();
  for (auto idA_iter = ids_to_check.begin(); idA_iter != ids_to_check.end();
       ++idA_iter) {
    auto elementA_iter = elements.find(*idA_iter);
    if (elementA_iter != elements_end) {
      for (auto idB_iter = idA_iter + 1; idB_iter != ids_to_check.end();
           ++idB_iter) {
        auto elementB_iter = elements.find(*idB_iter);
        if (elementB_iter != elements_end) {
          if (elements[*idA_iter]->CollidesWith(elements[*idB_iter].get())) {
            id_pairs.push_back(make_pair(*idA_iter, *idB_iter));
          }
        }
      }
    }
  }
  return closestPointsPairwise(id_pairs, use_margins, closest_points);
}

// Copy and paste identicalcode to BulletModel.
// TODO: Consider sharing this code.
bool FCLModel::closestPointsPairwise(const vector<ElementIdPair>& id_pairs,
                                     const bool use_margins,
                                     vector<PointPair>& closest_points) {
  unique_ptr<ResultCollector> c(new ResultCollector());
  for (auto id_pair_iter = id_pairs.begin(); id_pair_iter != id_pairs.end();
       ++id_pair_iter) {
    findClosestPointsBtwElements(id_pair_iter->first, id_pair_iter->second,
                                 use_margins, c);
  }

  closest_points = c->getResults();
  return closest_points.size() > 0;
}

bool FCLModel::collisionPointsAllToAll(const bool use_margins,
                                       vector<PointPair>& collision_points) {
  ResultCollector c;
  MatrixXd normals;
  vector<double> distance;

  //...
  collision_points = c.getResults();
  return c.pts.size() > 0;
}

FCLModel::unknownShapeException::unknownShapeException(
    DrakeShapes::Shape shape) {
  std::ostringstream ostr;
  ostr << shape;
  this->shape_str = ostr.str();
}

const char* FCLModel::unknownShapeException::what() const throw() {
  return ("Unknown collision shape: " + shape_str +
          ". Ignoring this collision element")
      .c_str();
}
}
