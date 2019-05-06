#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>

#include <fcl/fcl.h>
#include <tiny_obj_loader.h>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/distance_to_point.h"
#include "drake/geometry/proximity/distance_to_point_with_gradient.h"
#include "drake/geometry/proximity/distance_to_shape.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using std::make_shared;
using std::make_unique;
using std::move;
using std::shared_ptr;
using std::unique_ptr;

namespace {

// TODO(SeanCurtis-TRI): Swap all Isometry3 for RigidTransforms.

// Struct for use in SingleCollisionCallback(). Contains the collision request
// and accumulates results in a drake::multibody::collision::PointPair vector.
struct CollisionData {
  CollisionData(const std::vector<GeometryId>* geometry_map_in,
                const CollisionFilterLegacy* collision_filter_in)
      : geometry_map(*geometry_map_in),
        collision_filter(*collision_filter_in) {}
  // Maps so the penetration call back can map from engine index to geometry id.
  const std::vector<GeometryId>& geometry_map;
  const CollisionFilterLegacy& collision_filter;

  // Collision request
  fcl::CollisionRequestd request;

  // Vector of distance results
  std::vector<PenetrationAsPointPair<double>>* contacts{};
};

// Callback function for FCL's collide() function for retrieving a *single*
// contact.
bool SingleCollisionCallback(fcl::CollisionObjectd* fcl_object_A_ptr,
                             fcl::CollisionObjectd* fcl_object_B_ptr,
                             void* callback_data) {
  // NOTE: Although this function *takes* non-const pointers to satisfy the
  // fcl api, it should not exploit the non-constness to modify the collision
  // objects. We insure this by immediately assigning to a const version and
  // not directly using the provided parameters.
  const fcl::CollisionObjectd& fcl_object_A = *fcl_object_A_ptr;
  const fcl::CollisionObjectd& fcl_object_B = *fcl_object_B_ptr;

  auto& collision_data = *static_cast<CollisionData*>(callback_data);

  // Extract the collision filter keys from the fcl collision objects. These
  // keys will also be used to map the fcl collision object back to the Drake
  // GeometryId for colliding geometries.
  EncodedData encoding_A(fcl_object_A);
  EncodedData encoding_B(fcl_object_B);

  const bool can_collide = collision_data.collision_filter.CanCollideWith(
      encoding_A.encoding(), encoding_B.encoding());

  // NOTE: Here and below, false is returned regardless of whether collision
  // is detected or not because true tells the broadphase manager to terminate.
  // Since we want *all* collisions, we return false.
  if (!can_collide) return false;

  // Unpack the callback data
  const fcl::CollisionRequestd& request = collision_data.request;

  // This callback only works for a single contact, this confirms a request
  // hasn't been made for more contacts.
  DRAKE_ASSERT(request.num_max_contacts == 1);
  fcl::CollisionResultd result;

  // Perform nearphase collision detection
  fcl::collide(&fcl_object_A, &fcl_object_B, request, result);

  if (!result.isCollision()) return false;

  // Process the contact points
  // NOTE: This assumes that the request is configured to use a single
  // contact.
  const fcl::Contactd& contact = result.getContact(0);

  // Signed distance is negative when penetration depth is positive.
  const double depth = contact.penetration_depth;

  // TODO(SeanCurtis-TRI): Remove this test when FCL issue 375 is fixed.
  // FCL returns osculation as contact but doesn't guarantee a non-zero
  // normal. Drake isn't really in a position to define that normal from the
  // geometry or contact results so, if the geometry is sufficiently close
  // to osculation, we consider the geometries to be non-penetrating.
  if (depth <= std::numeric_limits<double>::epsilon()) return false;

  // By convention, Drake requires the contact normal to point out of B
  // and into A. FCL uses the opposite convention.
  Vector3d drake_normal = -contact.normal;

  // FCL returns a single contact point centered between the two
  // penetrating surfaces. PenetrationAsPointPair expects
  // two, one on the surface of body A (Ac) and one on the surface of body
  // B (Bc). Choose points along the line defined by the contact point and
  // normal, equidistant to the contact point. Recall that signed_distance
  // is strictly non-positive, so signed_distance * drake_normal points
  // out of A and into B.
  Vector3d p_WAc = contact.pos - 0.5 * depth * drake_normal;
  Vector3d p_WBc = contact.pos + 0.5 * depth * drake_normal;

  PenetrationAsPointPair<double> penetration;
  penetration.depth = depth;
  // The engine doesn't know geometry ids; it returns engine indices. The
  // caller must map engine indices to geometry ids.
  const std::vector<GeometryId>& geometry_map = collision_data.geometry_map;
  penetration.id_A = encoding_A.id(geometry_map);
  penetration.id_B = encoding_B.id(geometry_map);
  penetration.p_WCa = p_WAc;
  penetration.p_WCb = p_WBc;
  penetration.nhat_BA_W = drake_normal;
  // Guarantee fixed ordering of pair (A, B). Swap the ids and points on
  // surfaces and then flip the normal.
  if (penetration.id_B < penetration.id_A) {
    std::swap(penetration.id_A, penetration.id_B);
    std::swap(penetration.p_WCa, penetration.p_WCb);
    penetration.nhat_BA_W = -penetration.nhat_BA_W;
  }
  collision_data.contacts->emplace_back(std::move(penetration));

  return false;
}

// Returns a copy of the given fcl collision geometry; throws an exception for
// unsupported collision geometry types. This supplements the *missing* cloning
// functionality in FCL. Issue has been submitted to FCL:
// https://github.com/flexible-collision-library/fcl/issues/246
shared_ptr<fcl::ShapeBased> CopyShapeOrThrow(
    const fcl::CollisionGeometryd& geometry) {
  // NOTE: Returns a shared pointer because of the FCL API in assigning
  // collision geometry to collision objects.
  switch (geometry.getNodeType()) {
    case fcl::GEOM_SPHERE: {
      const auto& sphere = dynamic_cast<const fcl::Sphered&>(geometry);
      return make_shared<fcl::Sphered>(sphere.radius);
    }
    case fcl::GEOM_CYLINDER: {
      const auto& cylinder = dynamic_cast<const fcl::Cylinderd&>(geometry);
      return make_shared<fcl::Cylinderd>(cylinder.radius, cylinder.lz);
    }
    case fcl::GEOM_HALFSPACE:
      // All half spaces are defined exactly the same.
      return make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    case fcl::GEOM_BOX: {
      const auto& box = dynamic_cast<const fcl::Boxd&>(geometry);
      return make_shared<fcl::Boxd>(box.side);
    }
    case fcl::GEOM_CONVEX: {
      const auto& convex = dynamic_cast<const fcl::Convexd&>(geometry);
      // TODO(DamrongGuoy): Change to the copy constructor Convex(other) when
      //  we figure out why "Convex(const Convex& other) = default" created
      //  link errors for Xenial Debug build.  For now we do deep copy of the
      //  vertices and faces instead of simply copying the shared pointer.
      return make_shared<fcl::Convexd>(
          make_shared<const std::vector<Vector3d>>(convex.getVertices()),
          convex.getFaceCount(),
          make_shared<const std::vector<int>>(convex.getFaces()));
    }
    case fcl::GEOM_ELLIPSOID:
    case fcl::GEOM_CAPSULE:
    case fcl::GEOM_CONE:
    case fcl::GEOM_PLANE:
    case fcl::GEOM_TRIANGLE:
      throw std::logic_error(
          "Trying to copy fcl::CollisionGeometry of unsupported GEOM_* type");
    default:
      throw std::logic_error(
          "Trying to copy fcl::CollisionGeometry of non GEOM_* type");
  }
}

// Helper function that creates a *deep* copy of the given collision object.
unique_ptr<fcl::CollisionObjectd> CopyFclObjectOrThrow(
    const fcl::CollisionObjectd& object) {
  shared_ptr<fcl::ShapeBased> geometry_copy =
      CopyShapeOrThrow(*object.collisionGeometry());
  auto copy = make_unique<fcl::CollisionObjectd>(geometry_copy);
  copy->setUserData(object.getUserData());
  copy->setTransform(object.getTransform());
  copy->computeAABB();
  return copy;
}

// Helper function that creates a deep copy of a vector of collision objects.
// Assumes the input vector has already been cleared. The `copy_map` parameter
// serves as a mapping from each source object to its corresponding copy. Used
// to facilitate copying broadphase culling data structures (see
// ProximityEngine::operator=()).
void CopyFclObjectsOrThrow(
    const std::vector<unique_ptr<fcl::CollisionObjectd>>& source_objects,
    std::vector<unique_ptr<fcl::CollisionObjectd>>* target_objects,
    std::unordered_map<const fcl::CollisionObjectd*, fcl::CollisionObjectd*>*
        copy_map) {
  DRAKE_ASSERT(target_objects->size() == 0);
  target_objects->reserve(source_objects.size());
  for (const unique_ptr<fcl::CollisionObjectd>& source_object :
       source_objects) {
    target_objects->emplace_back(CopyFclObjectOrThrow(*source_object));
    copy_map->insert({source_object.get(), target_objects->back().get()});
  }
}

// Builds into the target AABB tree manager based on the reference "other"
// manager and the lookup table from other's collision objects to the target's
// collision objects (the map populated by CopyFclObjectsOrThrow()).
void BuildTreeFromReference(
    const fcl::DynamicAABBTreeCollisionManager<double>& other,
    const std::unordered_map<const fcl::CollisionObjectd*,
                             fcl::CollisionObjectd*>& copy_map,
    fcl::DynamicAABBTreeCollisionManager<double>* target) {
  std::vector<fcl::CollisionObjectd*> other_objects;
  other.getObjects(other_objects);
  for (auto* other_object : other_objects) {
    target->registerObject(copy_map.at(other_object));
  }
  target->update();
}

}  // namespace

// The implementation class for the fcl engine. Each of these functions
// mirrors a method on the ProximityEngine (unless otherwise indicated.
// See ProximityEngine for documentation.
template <typename T>
class ProximityEngine<T>::Impl : public ShapeReifier {
 public:
  Impl() = default;

  Impl(const Impl& other) : ShapeReifier(other) {
    dynamic_tree_.clear();
    dynamic_objects_.clear();
    anchored_tree_.clear();
    anchored_objects_.clear();

    // Copy all of the geometry to ensure that the engine index values stay
    // aligned.
    std::unordered_map<const fcl::CollisionObjectd*, fcl::CollisionObjectd*>
        object_map;
    CopyFclObjectsOrThrow(other.anchored_objects_, &anchored_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(other.dynamic_objects_, &dynamic_objects_,
                          &object_map);

    // Build new AABB trees from the input AABB trees.
    BuildTreeFromReference(other.dynamic_tree_, object_map, &dynamic_tree_);
    BuildTreeFromReference(other.anchored_tree_, object_map, &anchored_tree_);
    collision_filter_ = other.collision_filter_;
  }

  // Only the copy constructor is used to facilitate copying of the parent
  // ProximityEngine class.
  Impl& operator=(const Impl&) = delete;
  Impl(Impl&& other) = delete;
  Impl& operator=(Impl&&) = delete;

  std::unique_ptr<ProximityEngine<AutoDiffXd>::Impl> ToAutoDiff() const {
    auto engine = make_unique<ProximityEngine<AutoDiffXd>::Impl>();

    // TODO(SeanCurtis-TRI): When AutoDiff is fully supported in the internal
    // types, modify this map to the appropriate scalar and modify consuming
    // functions accordingly.
    // Copy all of the geometry to ensure that the engine index values stay
    // aligned.
    std::unordered_map<const fcl::CollisionObjectd*, fcl::CollisionObjectd*>
        object_map;
    CopyFclObjectsOrThrow(anchored_objects_, &engine->anchored_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(dynamic_objects_, &engine->dynamic_objects_,
                          &object_map);
    engine->collision_filter_ = this->collision_filter_;

    // Build new AABB trees from the input AABB trees.
    BuildTreeFromReference(dynamic_tree_, object_map, &engine->dynamic_tree_);
    BuildTreeFromReference(anchored_tree_, object_map, &engine->anchored_tree_);

    return engine;
  }

  ProximityIndex AddDynamicGeometry(const Shape& shape,
                                    GeometryIndex index) {
    // The collision object gets instantiated in the reification process and
    // placed in this unique pointer.
    std::unique_ptr<fcl::CollisionObject<double>> fcl_object;
    shape.Reify(this, &fcl_object);
    dynamic_tree_.registerObject(fcl_object.get());
    ProximityIndex proximity_index(static_cast<int>(dynamic_objects_.size()));
    // Encode the *global* pose index so that the geometry id can be looked up
    // in the event of a collision.
    EncodedData encoding(index, true /* is dynamic */);
    encoding.write_to(fcl_object.get());
    dynamic_objects_.emplace_back(std::move(fcl_object));

    collision_filter_.AddGeometry(encoding.encoding());

    return proximity_index;
  }

  ProximityIndex AddAnchoredGeometry(const Shape& shape,
                                     const Isometry3<double>& X_WG,
                                     GeometryIndex index) {
    // The collision object gets instantiated in the reification process and
    // placed in this unique pointer.
    std::unique_ptr<fcl::CollisionObject<double>> fcl_object;
    shape.Reify(this, &fcl_object);
    fcl_object->setTransform(X_WG);
    fcl_object->computeAABB();
    anchored_tree_.registerObject(fcl_object.get());
    anchored_tree_.update();
    ProximityIndex proximity_index(static_cast<int>(anchored_objects_.size()));
    EncodedData encoding(index, false /* is dynamic */);
    encoding.write_to(fcl_object.get());
    anchored_objects_.emplace_back(std::move(fcl_object));

    collision_filter_.AddGeometry(encoding.encoding());

    return proximity_index;
  }

  void UpdateGeometryIndex(ProximityIndex proximity_index, bool is_dynamic,
                           GeometryIndex geometry_index) {
    if (is_dynamic) {
      EncodedData::encode_dynamic(geometry_index)
          .write_to(dynamic_objects_[proximity_index].get());
    } else {
      EncodedData::encode_anchored(geometry_index)
          .write_to(anchored_objects_[proximity_index].get());
    }
  }

  optional<GeometryIndex> RemoveGeometry(ProximityIndex index,
                                         bool is_dynamic) {
    if (is_dynamic) {
      return RemoveGeometry(index, &dynamic_tree_, &dynamic_objects_);
    } else {
      return RemoveGeometry(index, &anchored_tree_, &anchored_objects_);
    }
  }

  int num_geometries() const {
    return static_cast<int>(dynamic_objects_.size() + anchored_objects_.size());
  }

  int num_dynamic() const { return static_cast<int>(dynamic_objects_.size()); }

  int num_anchored() const {
    return static_cast<int>(anchored_objects_.size());
  }

  void set_distance_tolerance(double tol) { distance_tolerance_ = tol; }

  double distance_tolerance() const { return distance_tolerance_; }

  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG,
                        const std::vector<GeometryIndex>& indices) {
    DRAKE_DEMAND(indices.size() == dynamic_objects_.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      // The FCL broadphase requires double-valued poses; so we use ADL to
      // efficiently get double-valued poses out of arbitrary T-valued poses.
      dynamic_objects_[i]->setTransform(convert_to_double(X_WG[indices[i]]));
      dynamic_objects_[i]->computeAABB();
    }
    dynamic_tree_.update();
  }

  // Implementation of ShapeReifier interface

  void ImplementGeometry(const Sphere& sphere, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_sphere = make_shared<fcl::Sphered>(sphere.get_radius());
    TakeShapeOwnership(fcl_sphere, user_data);
  }

  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_cylinder = make_shared<fcl::Cylinderd>(cylinder.get_radius(),
                                                    cylinder.get_length());
    TakeShapeOwnership(fcl_cylinder, user_data);
  }

  void ImplementGeometry(const HalfSpace&, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_half_space = make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    TakeShapeOwnership(fcl_half_space, user_data);
  }

  void ImplementGeometry(const Box& box, void* user_data) override {
    auto fcl_box = make_shared<fcl::Boxd>(box.size());
    TakeShapeOwnership(fcl_box, user_data);
  }

  void ImplementGeometry(const Mesh&, void*) override {
    throw std::domain_error("The proximity engine does not support meshes yet");
  }

  //
  // Convert vertices from tinyobj format to FCL format.
  //
  // Vertices from tinyobj are in a vector of floating-points like this:
  //     attrib.vertices = {c0,c1,c2, c3,c4,c5, c6,c7,c8,...}
  //                     = {x, y, z,  x, y, z,  x, y, z,...}
  // We will convert to a vector of Vector3d for FCL like this:
  //     vertices = {{c0,c1,c2}, {c3,c4,c5}, {c6,c7,c8},...}
  //              = {    v0,         v1,         v2,    ...}
  //
  // The size of `attrib.vertices` is three times the number of vertices.
  //
  std::vector<Vector3d> TinyObjToFclVertices(const tinyobj::attrib_t& attrib,
                                             const double scale) const {
    int num_coords = attrib.vertices.size();
    DRAKE_DEMAND(num_coords % 3 == 0);
    std::vector<Vector3d> vertices;
    vertices.reserve(num_coords / 3);

    auto iter = attrib.vertices.begin();
    while (iter != attrib.vertices.end()) {
      // We increment `iter` three times for x, y, and z coordinates.
      double x = *(iter++) * scale;
      double y = *(iter++) * scale;
      double z = *(iter++) * scale;
      vertices.emplace_back(x, y, z);
    }

    return vertices;
  }

  //
  // Convert faces from tinyobj to FCL.
  //
  //
  // A tinyobj mesh has an integer array storing the number of vertices of
  // each polygonal face.
  //     mesh.num_face_vertices = {n0,n1,n2,...}
  //         face0 has n0 vertices.
  //         face1 has n1 vertices.
  //         face2 has n2 vertices.
  //         ...
  // A tinyobj mesh has a vector of vertices that belong to the faces.
  //     mesh.indices = {v0_0, v0_1,..., v0_n0-1,
  //                     v1_0, v1_1,..., v1_n1-1,
  //                     v2_0, v2_1,..., v2_n2-1,
  //                     ...}
  //         face0 has vertices v0_0, v0_1,...,v0_n0-1.
  //         face1 has vertices v1_0, v1_1,...,v1_n1-1.
  //         face2 has vertices v2_0, v2_1,...,v2_n2-1.
  //         ...
  // For fcl::Convex, the `faces` is as an array of integers in this format.
  //     faces = { n0, v0_0,v0_1,...,v0_n0-1,
  //               n1, v1_0,v1_1,...,v1_n1-1,
  //               n2, v2_0,v2_1,...,v2_n2-1,
  //               ...}
  // where n_i is the number of vertices of face_i.
  //
  int TinyObjToFclFaces(const tinyobj::mesh_t& mesh,
                        std::vector<int>* faces) const {
    auto iter = mesh.indices.begin();
    for (const int& num : mesh.num_face_vertices) {
      faces->push_back(num);
      std::for_each(iter, iter + num, [faces](const tinyobj::index_t& index) {
        faces->push_back(index.vertex_index);
      });
      iter += num;
    }

    return mesh.num_face_vertices.size();
  }

  void ImplementGeometry(const Convex& convex, void* user_data) override {
    // We use tiny_obj_loader to read the .obj file of the convex shape.
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    // We keep polygonal faces without triangulating them. Some algorithms for
    // convex geometry perform better with fewer faces.
    bool do_tinyobj_triangulation = false;
    // We use default value (NULL) for the base directory of .mtl file (material
    // description), so it will be searched from the working directory.
    const char* mtl_basedir = nullptr;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
        convex.filename().c_str(), mtl_basedir, do_tinyobj_triangulation);
    if (!ret || !err.empty()) {
      throw std::runtime_error("Error parsing file '" + convex.filename() +
          "' : " + err);
    }

    // TODO(DamrongGuoy) Check that the input is a valid convex polyhedron.
    // 1. Each face is a planar polygon.
    // 2. Each face is a convex polygon.
    // 3. The polyhedron is convex.

    //
    // Now we convert tinyobj data for fcl::Convex.
    //

    if (shapes.size() != 1) {
      throw std::runtime_error("For Convex geometry, the .obj file must have "
                               "one and only one object defined in it.");
    }

    auto vertices = std::make_shared<std::vector<Vector3d>>(
        TinyObjToFclVertices(attrib, convex.scale()));

    const tinyobj::mesh_t& mesh = shapes[0].mesh;

    // We will have `faces.size()` larger than the number of faces. For each
    // face_i, the vector `faces` contains both the number and indices of its
    // vertices:
    //     faces = { n0, v0_0,v0_1,...,v0_n0-1,
    //               n1, v1_0,v1_1,...,v1_n1-1,
    //               n2, v2_0,v2_1,...,v2_n2-1,
    //               ...}
    // where n_i is the number of vertices of face_i.
    //
    auto faces = std::make_shared<std::vector<int>>();
    int num_faces = TinyObjToFclFaces(mesh, faces.get());

    // Create fcl::Convex.
    auto fcl_convex = make_shared<fcl::Convexd>(
        vertices, num_faces, faces);
    TakeShapeOwnership(fcl_convex, user_data);

    // TODO(DamrongGuoy): Per f2f with SeanCurtis-TRI, we want ProximityEngine
    // to own vertices and face by a map from filename.  This way we won't have
    // to read the same file again and again when we create multiple Convex
    // objects from the same file.
  }

  std::vector<SignedDistancePair<T>> ComputeSignedDistancePairwiseClosestPoints(
      const std::vector<GeometryId>& geometry_map,
      const std::vector<Isometry3<T>>& X_WGs) const {
    std::vector<SignedDistancePair<T>> witness_pairs;
    // All these quantities are aliased in the callback data.
    shape_distance::CallbackData<T> data{&geometry_map, &collision_filter_,
                                         &X_WGs, &witness_pairs};
    data.request.enable_nearest_points = true;
    data.request.enable_signed_distance = true;
    data.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    data.request.distance_tolerance = distance_tolerance_;

    dynamic_tree_.distance(&data, shape_distance::Callback<T>);
    dynamic_tree_.distance(
        const_cast<fcl::DynamicAABBTreeCollisionManager<double>*>(
            &anchored_tree_),
        &data, shape_distance::Callback<T>);
    return witness_pairs;
  }

  std::vector<SignedDistanceToPoint<T>> ComputeSignedDistanceToPoint(
      const Vector3<T>& p_WQ,
      const std::vector<GeometryId>& geometry_map,
      const std::vector<Isometry3<T>>& X_WGs,
      const double threshold) const {
    // We create a sphere of zero radius centered at the query point and put
    // it into a fcl::CollisionObject.
    auto fcl_sphere = make_shared<fcl::Sphered>(0.0);  // sphere of zero radius
    fcl::CollisionObjectd query_point(fcl_sphere);
    // The FCL broadphase requires double-valued poses; so we use ADL to
    // efficiently get double-valued poses out of arbitrary T-valued poses.
    query_point.setTranslation(convert_to_double(p_WQ));
    query_point.computeAABB();

    std::vector<SignedDistanceToPoint<T>> distances;

    point_distance::CallbackData<T> data{
        &query_point, &geometry_map, threshold, p_WQ, &X_WGs, &distances};

    anchored_tree_.distance(&query_point, &data, point_distance::Callback<T>);
    dynamic_tree_.distance(&query_point, &data, point_distance::Callback<T>);

    return distances;
  }

  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration(
      const std::vector<GeometryId>& geometry_map) const {
    std::vector<PenetrationAsPointPair<double>> contacts;
    // CollisionData stores references to the provided data structures.
    CollisionData collision_data{&geometry_map, &collision_filter_};
    collision_data.contacts = &contacts;
    collision_data.request.num_max_contacts = 1;
    collision_data.request.enable_contact = true;
    // NOTE: As of 5/1/2018 the GJK implementation of Libccd appears to be
    // superior to FCL's "independent" implementation. Furthermore, libccd
    // appears to behave badly if its gjk tolerance is much tighter than
    // 2e-12. Until this changes, we explicitly specify these parameters rather
    // than relying on FCL's defaults.
    collision_data.request.gjk_tolerance = 2e-12;
    collision_data.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    dynamic_tree_.collide(&collision_data, SingleCollisionCallback);

    // NOTE: The interface to DynamicAABBTreeCollisionManager::collide
    // requires the input collision manager pointer to be *non* const.
    // As of 02/06/2018, it appears the only opportunity for modification
    // of the AABB tree (and its contents) occurs in the callback provided.
    // See the definition of SingleCollisionCallback above to see that no
    // modification takes place.
    dynamic_tree_.collide(
        const_cast<fcl::DynamicAABBTreeCollisionManager<double>*>(
            &anchored_tree_),
        &collision_data, SingleCollisionCallback);
    return contacts;
  }

  // TODO(SeanCurtis-TRI): Update this with the new collision filter method.
  void ExcludeCollisionsWithin(
      const std::unordered_set<GeometryIndex>& dynamic,
      const std::unordered_set<GeometryIndex>& anchored) {
    // Preventing collision between members in a single set is simple: assign
    // every geometry to the same clique.

    // There are no collisions between anchored geometries. So, to meaningfully
    // add collisions, there must be dynamic geometry. Only perform work if:
    //   1. There are multiple dynamic geometries, or
    //   2. There are non-zero numbers of dynamic *and* anchored geometries.
    //
    // NOTE: Given the set of geometries G, if the pair (gᵢ, gⱼ), gᵢ, gⱼ ∈ G
    // is already filtered, this *will* add a redundant clique.
    // TODO(SeanCurtis-TRI): If redundant cliques proves to have a performance
    // impact before the alternate filtering mechanism is in place, revisit this
    // algorithm to prevent redundancy.

    if (dynamic.size() > 1 || (dynamic.size() > 0 && anchored.size() > 0)) {
      int clique = collision_filter_.next_clique_id();
      for (auto index : dynamic) {
        EncodedData encoding(index, true /* is dynamic */);
        collision_filter_.AddToCollisionClique(encoding.encoding(), clique);
      }
      for (auto index : anchored) {
        EncodedData encoding(index, false /* is dynamic */);
        collision_filter_.AddToCollisionClique(encoding.encoding(), clique);
      }
    }
  }

  void ExcludeCollisionsBetween(
      const std::unordered_set<GeometryIndex>& dynamic1,
      const std::unordered_set<GeometryIndex>& anchored1,
      const std::unordered_set<GeometryIndex>& dynamic2,
      const std::unordered_set<GeometryIndex>& anchored2) {
    // TODO(SeanCurtis-TRI): Update this with the new collision filter method.

    // NOTE: This is a brute-force implementation. It does not claim to be
    // optimal in any way. It merely guarantees the semantics given. If the
    // two sets of geometries are in fact the *same* set (i.e., this is used
    // to create the same effect as ExcludeCollisionsWithin), it will work, but
    // be horribly inefficient (with each geometry picking up N cliques).
    using std::transform;
    using std::back_inserter;
    using std::vector;

    // There are no collisions between anchored geometries. So, there must be
    // dynamic geometry for work to be necessary.
    if (dynamic1.size() > 0 || dynamic2.size() > 0) {
      vector<EncodedData> group1;
      transform(dynamic1.begin(), dynamic1.end(), back_inserter(group1),
                EncodedData::encode_dynamic);
      transform(anchored1.begin(), anchored1.end(), back_inserter(group1),
                EncodedData::encode_anchored);
      vector<EncodedData> group2;
      transform(dynamic2.begin(), dynamic2.end(), back_inserter(group2),
                EncodedData::encode_dynamic);
      transform(anchored2.begin(), anchored2.end(), back_inserter(group2),
                EncodedData::encode_anchored);

      // O(N²) process which generates O(N²) cliques. For the two collision
      // groups G and H, each pair (g, h), g ∈ G, h ∈ H, requires a unique
      // clique. If the cliques were not unique, then there would be a pair
      // (a, b) where a, b ∈ G or a, b ∈ H where a and b have the same clique.
      // And that would lead to removal of self-collision within one of the
      // collision groups.
      //
      // NOTE: Using cliques for this purpose is *horribly* inefficient. This is
      // exactly what collision filter groups are best at.
      for (auto encoding1 : group1) {
        for (auto encoding2 : group2) {
          if ((encoding1.is_dynamic() || encoding2.is_dynamic()) &&
              collision_filter_.CanCollideWith(encoding1.encoding(),
                                               encoding2.encoding())) {
            int clique = collision_filter_.next_clique_id();
            collision_filter_.AddToCollisionClique(encoding2.encoding(),
                                                   clique);
            collision_filter_.AddToCollisionClique(encoding1.encoding(),
                                                   clique);
          }
        }
      }
    }
  }

  bool CollisionFiltered(GeometryIndex index1, bool is_dynamic_1,
                         GeometryIndex index2, bool is_dynamic_2) const {
    // Collisions between anchored geometries are implicitly filtered.
    if (!is_dynamic_1 && !is_dynamic_2) return true;
    EncodedData encoding1(index1, is_dynamic_1);
    EncodedData encoding2(index2, is_dynamic_2);
    return !collision_filter_.CanCollideWith(encoding1.encoding(),
                                             encoding2.encoding());
  }

  int get_next_clique() { return collision_filter_.next_clique_id(); }

  void set_clique(GeometryIndex index, int clique) {
    EncodedData encoding(index, true /* is dynamic */);
    collision_filter_.AddToCollisionClique(encoding.encoding(), clique);
  }

  // Testing utilities

  bool IsDeepCopy(const Impl& other) const {
    if (this != &other) {
      // Function for validating that two objects are different objects with
      // "identical" data. The test isn't exhaustive (for example, the
      // parameters of the particular geometric shape are not compared--instead,
      // we compare the AABBs).
      auto ValidateObject = [](const fcl::CollisionObject<double>& test,
                               const fcl::CollisionObject<double>& ref) {
        return test.getUserData() == ref.getUserData() && &test != &ref &&
               test.getNodeType() == ref.getNodeType() &&
               test.getObjectType() == ref.getObjectType() &&
               test.getAABB().center() == ref.getAABB().center() &&
               test.getAABB().width() == ref.getAABB().width() &&
               test.getAABB().height() == ref.getAABB().height() &&
               test.getAABB().depth() == ref.getAABB().depth();
      };
      bool is_copy = true;
      is_copy = is_copy &&
                this->dynamic_objects_.size() == other.dynamic_objects_.size();
      is_copy =
          is_copy &&
          this->anchored_objects_.size() == other.anchored_objects_.size();
      if (is_copy) {
        for (size_t i = 0; i < this->dynamic_objects_.size(); ++i) {
          const fcl::CollisionObject<double>& test_object =
              *this->dynamic_objects_.at(i);
          const fcl::CollisionObject<double>& ref_object =
              *other.dynamic_objects_.at(i);
          is_copy = is_copy && ValidateObject(test_object, ref_object);
        }
        for (size_t i = 0; i < this->anchored_objects_.size(); ++i) {
          const fcl::CollisionObject<double>& test_object =
              *this->anchored_objects_.at(i);
          const fcl::CollisionObject<double>& ref_object =
              *other.anchored_objects_.at(i);
          is_copy = is_copy && ValidateObject(test_object, ref_object);
        }
      }
      return is_copy;
    }

    return false;
  }

  int peek_next_clique() const { return collision_filter_.peek_next_clique(); }

  const Isometry3<double>& GetX_WG(ProximityIndex index,
                                   bool is_dynamic) const {
    if (is_dynamic) {
      return dynamic_objects_[index]->getTransform();
    } else {
      return anchored_objects_[index]->getTransform();
    }
  }

  GeometryIndex GetGeometryIndex(ProximityIndex index, bool is_dynamic) const {
    if (is_dynamic) {
      return EncodedData(*dynamic_objects_[index]).index();
    } else {
      return EncodedData(*anchored_objects_[index]).index();
    }
  }

 private:
  // Engine on one scalar can see the members of other engines.
  friend class ProximityEngineTester;
  template <typename>
  friend class ProximityEngine;

  // Removes the geometry with the given proximity index from the given tree. It
  // potentially moves another object to take its slot in the vector of objects
  // to maintain a contiguous memory block.
  optional<GeometryIndex> RemoveGeometry(
      ProximityIndex index, fcl::DynamicAABBTreeCollisionManager<double>* tree,
      std::vector<std::unique_ptr<fcl::CollisionObject<double>>>* geometries) {
    std::vector<unique_ptr<fcl::CollisionObject<double>>>& typed_geometries =
        *geometries;
    fcl::CollisionObjectd* fcl_object = typed_geometries[index].get();
    const size_t old_size = tree->size();
    tree->unregisterObject(fcl_object);
    // NOTE: The FCL API provides no other mechanism for confirming the
    // unregistration was successful.
    DRAKE_DEMAND(old_size == tree->size() + 1);
    optional<GeometryIndex> moved{};
    DRAKE_DEMAND(typed_geometries.size() > 0);
    const size_t last = typed_geometries.size() - 1;
    if (index < last) {
      // Removed geometry that *isn't* the last in the geometries. Swap last
      // into empty slot.
      const fcl::CollisionObjectd* move_object = typed_geometries[last].get();
      EncodedData encoding(*move_object);
      moved = encoding.index();
      typed_geometries[index].swap(typed_geometries.back());
    }
    typed_geometries.pop_back();
    return moved;
  }

  // TODO(SeanCurtis-TRI): Convert these to scalar type T when I know how to
  // transmogrify them. Otherwise, while the engine can't be transmogrified, the
  // results on an <AutoDiffXd> type will still be double.

  // Helper method called by the various ImplementGeometry overrides to
  // facilitate the logistics of creating shapes from specifications. `data`
  // is a unique_ptr of an fcl CollisionObject that should be instantiated
  // with the given shape.
  void TakeShapeOwnership(const std::shared_ptr<fcl::ShapeBased>& shape,
                          void* data) {
    DRAKE_ASSERT(data != nullptr);
    std::unique_ptr<fcl::CollisionObject<double>>& fcl_object_ptr =
        *reinterpret_cast<std::unique_ptr<fcl::CollisionObject<double>>*>(data);
    fcl_object_ptr = make_unique<fcl::CollisionObject<double>>(shape);
  }

  // The BVH of all dynamic geometries; this depends on *all* inputs.
  // TODO(SeanCurtis-TRI): Ultimately, this should probably be a cache entry.
  fcl::DynamicAABBTreeCollisionManager<double> dynamic_tree_;

  // All of the *dynamic* collision elements (spanning all sources). Their
  // GeometryIndex maps to their position in *this* vector.
  // TODO(SeanCurtis-TRI): Cluster the geometries on source such that each
  // source owns a _contiguous_ block of engine indices.
  std::vector<std::unique_ptr<fcl::CollisionObject<double>>> dynamic_objects_;

  // The tree containing all of the anchored geometry.
  fcl::DynamicAABBTreeCollisionManager<double> anchored_tree_;

  // All of the *anchored* collision elements (spanning *all* sources). Their
  // AnchoredGeometryIndex maps to their position in *this* vector.
  std::vector<std::unique_ptr<fcl::CollisionObject<double>>> anchored_objects_;

  // The mechanism for dictating collision filtering.
  CollisionFilterLegacy collision_filter_;

  // The tolerance that determines when the iterative process would terminate.
  // @see ProximityEngine::set_distance_tolerance() for more details.
  double distance_tolerance_{1E-6};
};

template <typename T>
ProximityEngine<T>::ProximityEngine() : impl_(new Impl()) {}

template <typename T>
ProximityEngine<T>::~ProximityEngine() {
  delete impl_;
}

template <typename T>
ProximityEngine<T>::ProximityEngine(ProximityEngine<T>::Impl* impl)
    : impl_(impl) {}

template <typename T>
ProximityEngine<T>::ProximityEngine(const ProximityEngine<T>& other)
    : impl_(new ProximityEngine<T>::Impl(*other.impl_)) {}

template <typename T>
ProximityEngine<T>& ProximityEngine<T>::operator=(
    const ProximityEngine<T>& other) {
  if (this == &other) return *this;
  if (impl_) delete impl_;
  impl_ = new ProximityEngine<T>::Impl(*other.impl_);
  return *this;
}

template <typename T>
ProximityEngine<T>::ProximityEngine(ProximityEngine<T>&& other) noexcept
    : impl_(std::move(other.impl_)) {
  other.impl_ = new ProximityEngine<T>::Impl();
}

template <typename T>
ProximityEngine<T>& ProximityEngine<T>::operator=(
    ProximityEngine<T>&& other) noexcept {
  if (this == &other) return *this;
  if (impl_) delete impl_;
  impl_ = std::move(other.impl_);
  other.impl_ = new ProximityEngine<T>::Impl();
  return *this;
}

template <typename T>
ProximityIndex ProximityEngine<T>::AddDynamicGeometry(
    const Shape& shape, GeometryIndex index) {
  return impl_->AddDynamicGeometry(shape, index);
}

template <typename T>
ProximityIndex ProximityEngine<T>::AddAnchoredGeometry(
    const Shape& shape, const Isometry3<double>& X_WG, GeometryIndex index) {
  return impl_->AddAnchoredGeometry(shape, X_WG, index);
}

template <typename T>
void ProximityEngine<T>::UpdateGeometryIndex(ProximityIndex proximity_index,
                                             bool is_dynamic,
                                             GeometryIndex geometry_index) {
  impl_->UpdateGeometryIndex(proximity_index, is_dynamic, geometry_index);
}

template <typename T>
optional<GeometryIndex> ProximityEngine<T>::RemoveGeometry(
    ProximityIndex index, bool is_dynamic) {
  return impl_->RemoveGeometry(index, is_dynamic);
}

template <typename T>
int ProximityEngine<T>::num_geometries() const {
  return impl_->num_geometries();
}

template <typename T>
int ProximityEngine<T>::num_dynamic() const {
  return impl_->num_dynamic();
}

template <typename T>
int ProximityEngine<T>::num_anchored() const {
  return impl_->num_anchored();
}

template <typename T>
void ProximityEngine<T>::set_distance_tolerance(double tol) {
  impl_->set_distance_tolerance(tol);
}

template <typename T>
double ProximityEngine<T>::distance_tolerance() const {
  return impl_->distance_tolerance();
}

template <typename T>
std::unique_ptr<ProximityEngine<AutoDiffXd>> ProximityEngine<T>::ToAutoDiffXd()
    const {
  return unique_ptr<ProximityEngine<AutoDiffXd>>(
      new ProximityEngine<AutoDiffXd>(impl_->ToAutoDiff().release()));
}

template <typename T>
void ProximityEngine<T>::UpdateWorldPoses(
    const std::vector<Isometry3<T>>& X_WG,
    const std::vector<GeometryIndex>& indices) {
  impl_->UpdateWorldPoses(X_WG, indices);
}

template <typename T>
std::vector<SignedDistancePair<T>>
ProximityEngine<T>::ComputeSignedDistancePairwiseClosestPoints(
    const std::vector<GeometryId>& geometry_map,
    const std::vector<Isometry3<T>>& X_WGs) const {
  return impl_->ComputeSignedDistancePairwiseClosestPoints(geometry_map, X_WGs);
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
ProximityEngine<T>::ComputeSignedDistanceToPoint(
    const Vector3<T>& query, const std::vector<GeometryId>& geometry_map,
    const std::vector<Isometry3<T>>& X_WGs, const double threshold) const {
  return impl_->ComputeSignedDistanceToPoint(query, geometry_map, X_WGs,
                                             threshold);
}

template <typename T>
std::vector<PenetrationAsPointPair<double>>
ProximityEngine<T>::ComputePointPairPenetration(
    const std::vector<GeometryId>& geometry_map) const {
  return impl_->ComputePointPairPenetration(geometry_map);
}

template <typename T>
std::vector<ContactSurface<T>>
ProximityEngine<T>::ComputeContactSurfaces(
    const std::vector<GeometryId>& /* geometry_map */) const {
  throw std::runtime_error(
      "ComputeContactSurfaces() is not implemented yet.");
  // TODO(DamrongGuoy): Compute contact surfaces and remove the above throw.
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsWithin(
    const std::unordered_set<GeometryIndex>& dynamic,
    const std::unordered_set<GeometryIndex>& anchored) {
  impl_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsBetween(
    const std::unordered_set<GeometryIndex>& dynamic1,
    const std::unordered_set<GeometryIndex>& anchored1,
    const std::unordered_set<GeometryIndex>& dynamic2,
    const std::unordered_set<GeometryIndex>& anchored2) {
  impl_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2, anchored2);
}

template <typename T>
bool ProximityEngine<T>::CollisionFiltered(
    GeometryIndex index1, bool is_dynamic_1,
    GeometryIndex index2, bool is_dynamic_2) const {
  return impl_->CollisionFiltered(index1, is_dynamic_1, index2, is_dynamic_2);
}

// Client-attorney interface for GeometryState to manipulate collision filters.

template <typename T>
int ProximityEngine<T>::get_next_clique() {
  return impl_->get_next_clique();
}

template <typename T>
void ProximityEngine<T>::set_clique(GeometryIndex index, int clique) {
  impl_->set_clique(index, clique);
}

// Testing utilities

template <typename T>
bool ProximityEngine<T>::IsDeepCopy(const ProximityEngine<T>& other) const {
  return impl_->IsDeepCopy(*other.impl_);
}

template <typename T>
int ProximityEngine<T>::peek_next_clique() const {
  return impl_->peek_next_clique();
}

template <typename T>
const Isometry3<double>& ProximityEngine<T>::GetX_WG(ProximityIndex index,
                                                     bool is_dynamic) const {
  return impl_->GetX_WG(index, is_dynamic);
}

template <typename T>
GeometryIndex ProximityEngine<T>::GetGeometryIndex(ProximityIndex index,
                                                   bool is_dynamic) const {
  return impl_->GetGeometryIndex(index, is_dynamic);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::ProximityEngine)
