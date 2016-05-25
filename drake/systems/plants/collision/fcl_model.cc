#include <iostream>

#include "fcl_model.h"

#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision_data.h"
#include "fcl/distance.h"
#include "fcl/collision.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"

#include "drake/systems/plants/collision/DrakeCollision.h"

namespace DrakeCollision {

std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> FCLModel::newFCLBoxShape(
    const DrakeShapes::Box& geometry, bool use_margins) {
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Box shape(geometry.size(0), geometry.size(1), geometry.size(2));
  std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_shape(
    new fcl::BVHModel<fcl::OBBRSS>());
  fcl::generateBVHModel(*bvh_shape, shape, fcl::Transform3f());

  return std::move(bvh_shape);
}

std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> FCLModel::newFCLSphereShape(
    const DrakeShapes::Sphere& geometry, bool use_margins) {
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Sphere shape(geometry.radius);
  std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_shape(
    new fcl::BVHModel<fcl::OBBRSS>());
  int numLong = 10;
  int numLat = 8;
  fcl::generateBVHModel(*bvh_shape, shape, fcl::Transform3f(), numLong, numLat);

  return std::move(bvh_shape);
}

std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> FCLModel::newFCLCylinderShape(
    const DrakeShapes::Cylinder& geometry, bool use_margins) {
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Cylinder shape(geometry.radius, geometry.length);
  std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_shape(
    new fcl::BVHModel<fcl::OBBRSS>());
  int numLength = 1;
  int numTheta = 10;
  fcl::generateBVHModel(*bvh_shape, shape, fcl::Transform3f(), numTheta,
                        numLength);

  return std::move(bvh_shape);
}

std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> FCLModel::newFCLCapsuleShape(
    const DrakeShapes::Capsule& geometry, bool use_margins) {
  /*
  // NOTE:  Ignore margins for now.
  use_margins = use_margins;

  fcl::Capsule shape(geometry.radius, geometry.length);
  std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_shape(
    new fcl::BVHModel<fcl::OBBRSS>());

  // fcl does not have a method to generate a capsule mesh.
  // TODO(law12019): Make one.
  // fcl::generateBVHModel(*bvh_shape, shape, Transform3f(), numTheta,
  // numLength);

  return std::move(bvh_shape);
  */
  throw std::logic_error("FCLModel::newFCLCapsuleShape is not implemented yet");
}

// TODO(law12019): Figure out how to convert mesh verts to fcl mesh.
std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> FCLModel::newFCLMeshShape(
    const DrakeShapes::Mesh& geometry, bool use_margins) {
  Eigen::Matrix3Xd vertices;
  if (geometry.extractMeshVertices(vertices)) {
    // geometry.getPoints(Eigen::Matrix3Xd &points) const;
  }
  // TODO(law12019): What about traingle indexes?
  throw std::logic_error("FCLModel::newFCLMeshShape is not implemented yet");
}

ElementId FCLModel::addElement(const Element& element) {
  ElementId id = Model::addElement(element);

  if (id != 0) {
    std::unique_ptr<fcl::BVHModel<fcl::OBBRSS>> bvh_shape;
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
      case DrakeShapes::MESH:         // not implemented yet
      case DrakeShapes::MESH_POINTS:  // not implemented yet
      case DrakeShapes::CAPSULE:      // not implemented yet
      default:
        std::ostringstream ostr;
        ostr << "Unknown collision shape: " << elements[id]->getShape();
        throw std::logic_error(ostr.str());
        break;
    }
    if (bvh_shape) {
      this->fclElements.insert(std::make_pair(id, std::move(bvh_shape)));
    }
  }
  return id;
}

std::vector<PointPair> FCLModel::potentialCollisionPoints(bool use_margins) {
  ResultCollector c;
  // TODO(law12019): Not Implemented
  throw std::logic_error(
    "FCLModel::potentialCollisionPoints is not implemented yet");
}

bool FCLModel::collidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>& points, double collision_threshold) {
  // TODO(law12019): Not Implemented
  throw std::logic_error(
    "FCLModel::collisionPointsCheckOnly is not implemented yet");
}

std::vector<size_t> FCLModel::collidingPoints(
    const std::vector<Eigen::Vector3d>& points, double collision_threshold) {
  throw std::logic_error("FCLModel::collidingPoints is not implemented yet");
}

bool FCLModel::updateElementWorldTransform(
    const ElementId id, const Eigen::Isometry3d& T_local_to_world) {
  // Since the transform is applied during collision, this method may not need
  // to be implemented.
  std::cerr << "Warning: FCLModel::updateElementWorldTransform not implemented."
            << std::endl;
  // throw std::runtime_error("not implemented yet");
  return false;
}

void FCLModel::updateModel() {
  // I am not sure if this needs to be implemented.
  // NOTE: ccl: I am not sure what changes I have to accommodate.
  // Should I rebuild the fclModels?
  // TODO(law12019): Test this.
  throw std::logic_error("FCLModel::updateModel is not implemented yet");
}

bool FCLModel::findClosestPointsBtwElements(
    const ElementId idA, const ElementId idB, const bool use_margins,
    std::unique_ptr<ResultCollector>& c) {
  // For FCL we are converting basic shapes into meshes.
  // This is the same special case in BulletModel.
  // TODO(law12019): Consider sharing code.
  // special case: two spheres (because we need to handle the zero-radius sphere
  // case)
  if (elements[idA]->getShape() == DrakeShapes::SPHERE &&
      elements[idB]->getShape() == DrakeShapes::SPHERE) {
    const Eigen::Isometry3d& TA_world = elements[idA]->getWorldTransform();
    const Eigen::Isometry3d& TB_world = elements[idB]->getWorldTransform();
    auto xA_world = TA_world.translation();
    auto xB_world = TB_world.translation();
    double radiusA = dynamic_cast<const DrakeShapes::Sphere&>(
        elements[idA]->getGeometry()).radius;
    double radiusB = dynamic_cast<const DrakeShapes::Sphere&>(
        elements[idB]->getGeometry()).radius;
    double distance = (xA_world - xB_world).norm();
    c->addSingleResult(idA, idB,
                       elements[idA]->getLocalTransform() * TA_world.inverse() *
                           (xA_world + (xB_world - xA_world) * radiusA /
                                           distance),  // ptA (in body A coords)
                       elements[idB]->getLocalTransform() * TB_world.inverse() *
                           (xB_world + (xA_world - xB_world) * radiusB /
                                           distance),  // ptB (in body B coords)
                       (xA_world - xB_world) / distance,
                       distance - radiusA - radiusB);
    return true;
  }

  fcl::DistanceRequest request;
  request.enable_nearest_points = true;
  fcl::DistanceResult res;

  const Eigen::Isometry3d& TA_world = elements[idA]->getWorldTransform();
  const Eigen::Isometry3d& TB_world = elements[idB]->getWorldTransform();

  // Convert the Eigen transform into the fcl transform.
  fcl::Matrix3f ma(TA_world(0, 0), TA_world(0, 1), TA_world(0, 2),
                   TA_world(1, 0), TA_world(1, 1), TA_world(1, 2),
                   TA_world(2, 0), TA_world(2, 1), TA_world(2, 2));
  fcl::Vec3f va(TA_world(0, 3), TA_world(1, 3), TA_world(2, 3));

  fcl::Matrix3f mb(TB_world(0, 0), TB_world(0, 1), TB_world(0, 2),
                   TB_world(1, 0), TB_world(1, 1), TB_world(1, 2),
                   TB_world(2, 0), TB_world(2, 1), TB_world(2, 2));
  fcl::Vec3f vb(TB_world(0, 3), TB_world(1, 3), TB_world(2, 3));

  fcl::Transform3f tfa(ma, va);
  fcl::Transform3f tfb(mb, vb);

  fcl::distance(fclElements[idA].get(), tfa, fclElements[idB].get(), tfb,
                request, res);
  auto dist = res.min_distance;
  fcl::Vec3f fclPt;

  if (dist > 0) {
    // FCL reports closest points in world coordinates.
    // drake returns them in element coordinates.

    tfa.inverse();
    fclPt = tfa.transform(res.nearest_points[0]);
    Eigen::Vector3d ptA(fclPt[0], fclPt[1], fclPt[2]);

    tfb.inverse();
    fclPt = tfb.transform(res.nearest_points[1]);
    Eigen::Vector3d ptB(fclPt[0], fclPt[1], fclPt[2]);

    c->addSingleResult(idA, idB, ptA, ptB, (ptA - ptB) / dist, dist);
  } else {
    // fcl::distance does work when elements are in collision.
    int num_max_contacts = std::numeric_limits<int>::max();
    bool enable_contact = true;
    fcl::CollisionRequest cRequest(num_max_contacts, enable_contact);
    fcl::CollisionResult cRes;
    fcl::collide(fclElements[idA].get(), tfa, fclElements[idB].get(), tfb,
                 cRequest, cRes);

    tfa.inverse();
    tfb.inverse();

    int num = cRes.numContacts();
    for (int i = 0; i < num; ++i) {
      fcl::Contact contact = cRes.getContact(i);
      // if (c.o1 == fclElements[idA].get()) {
      fclPt = tfa.transform(contact.pos);
      Eigen::Vector3d ptA(fclPt[0], fclPt[1], fclPt[2]);
      auto dist = contact.penetration_depth;
      fclPt = tfb.transform(contact.pos - (contact.normal * dist));
      Eigen::Vector3d ptB(fclPt[0], fclPt[1], fclPt[2]);
      Eigen::Vector3d normal(contact.normal[0], contact.normal[1],
                             contact.normal[2]);
      c->addSingleResult(idA, idB, ptA, ptB, normal, dist);
    }
  }

  return (c->pts.size() > 0);
}

void FCLModel::collisionDetectFromPoints(
    const Eigen::Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>& closest_points) {
  closest_points.resize(points.cols(),
                        PointPair(0, 0, Eigen::Vector3d(), Eigen::Vector3d(),
                                  Eigen::Vector3d(), 0.0));
  Eigen::VectorXd phi(points.cols());

  throw std::logic_error(
    "FCLModel::collisionDetectFromPoints is not implemented yet");
}

bool FCLModel::collisionRaycast(const Eigen::Matrix3Xd& origins,
                                const Eigen::Matrix3Xd& ray_endpoints,
                                bool use_margins, Eigen::VectorXd& distances,
                                Eigen::Matrix3Xd& normals) {
  distances.resize(origins.cols());
  normals.resize(3, origins.cols());

  throw std::logic_error("FCLModel::collisionRaycast is not implemented yet");
}

// This is the same implementation as in bullet.
// TODO(law12019): consider sharing code.
bool FCLModel::closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     const bool use_margins,
                                     std::vector<PointPair>& closest_points) {
  std::vector<ElementIdPair> id_pairs;
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
            id_pairs.push_back(std::make_pair(*idA_iter, *idB_iter));
          }
        }
      }
    }
  }
  return closestPointsPairwise(id_pairs, use_margins, closest_points);
}

// Copy and paste identicalcode to BulletModel.
// TODO(law12019): Consider sharing this code.
bool FCLModel::closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                                     const bool use_margins,
                                     std::vector<PointPair>& closest_points) {
  std::unique_ptr<ResultCollector> c(new ResultCollector());
  for (auto id_pair_iter = id_pairs.begin(); id_pair_iter != id_pairs.end();
       ++id_pair_iter) {
    findClosestPointsBtwElements(id_pair_iter->first, id_pair_iter->second,
                                 use_margins, c);
  }

  closest_points = c->getResults();
  return closest_points.size() > 0;
}

bool FCLModel::collisionPointsAllToAll(
    const bool use_margins, std::vector<PointPair>& collision_points) {
  ResultCollector c;
  Eigen::MatrixXd normals;
  std::vector<double> distance;

  //...
  collision_points = c.getResults();
  return c.pts.size() > 0;
}
}
