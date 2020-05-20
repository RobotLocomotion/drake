#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <limits>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/collisions_exist_callback.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/distance_to_point_with_gradient.h"
#include "drake/geometry/proximity/distance_to_shape_callback.h"
#include "drake/geometry/proximity/find_collision_candidates_callback.h"
#include "drake/geometry/proximity/hydroelastic_callback.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"
#include "drake/geometry/utilities.h"

static_assert(std::is_same<tinyobj::real_t, double>::value,
              "tinyobjloader must be compiled in double-precision mode");

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using fcl::CollisionObjectd;
using drake::geometry::internal::HydroelasticType;
using math::RigidTransform;
using math::RigidTransformd;
using std::make_shared;
using std::make_unique;
using std::move;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

namespace {

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
    case fcl::GEOM_ELLIPSOID: {
      const auto& ellipsoid = dynamic_cast<const fcl::Ellipsoidd&>(geometry);
      return make_shared<fcl::Ellipsoidd>(ellipsoid.radii);
    }
    case fcl::GEOM_HALFSPACE:
      // All half spaces are defined exactly the same.
      return make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    case fcl::GEOM_BOX: {
      const auto& box = dynamic_cast<const fcl::Boxd&>(geometry);
      return make_shared<fcl::Boxd>(box.side);
    }
    case fcl::GEOM_CAPSULE: {
      const auto& capsule = dynamic_cast<const fcl::Capsuled&>(geometry);
      return make_shared<fcl::Capsuled>(capsule.radius, capsule.lz);
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
unique_ptr<CollisionObjectd> CopyFclObjectOrThrow(
    const CollisionObjectd& object) {
  shared_ptr<fcl::ShapeBased> geometry_copy =
      CopyShapeOrThrow(*object.collisionGeometry());
  auto copy = make_unique<CollisionObjectd>(geometry_copy);
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
    const unordered_map<GeometryId, unique_ptr<CollisionObjectd>>&
        source_objects,
    unordered_map<GeometryId, unique_ptr<CollisionObjectd>>* target_objects,
    std::unordered_map<const CollisionObjectd*, CollisionObjectd*>* copy_map) {
  DRAKE_ASSERT(target_objects->size() == 0);
  for (const auto& source_id_object_pair : source_objects) {
    const GeometryId source_id = source_id_object_pair.first;
    const CollisionObjectd& source_object = *source_id_object_pair.second;
    (*target_objects)[source_id] = CopyFclObjectOrThrow(source_object);
    copy_map->insert({&source_object, (*target_objects)[source_id].get()});
  }
}

// Builds into the target AABB tree manager based on the reference "other"
// manager and the lookup table from other's collision objects to the target's
// collision objects (the map populated by CopyFclObjectsOrThrow()).
void BuildTreeFromReference(
    const fcl::DynamicAABBTreeCollisionManager<double>& other,
    const std::unordered_map<const CollisionObjectd*,
                             CollisionObjectd*>& copy_map,
    fcl::DynamicAABBTreeCollisionManager<double>* target) {
  std::vector<CollisionObjectd*> other_objects;
  other.getObjects(other_objects);
  for (auto* other_object : other_objects) {
    target->registerObject(copy_map.at(other_object));
  }
  target->update();
}

// The data necessary for shape reification.
struct ReifyData {
  unique_ptr<CollisionObjectd> fcl_object;
  const GeometryId id;
  const ProximityProperties& properties;
};

// Small class for identifying mesh geometries. Unlike other kinds of
// geometries, meshes are supported only in ComputeContactSurfaces but
// not other proximity queries.
class MeshIdentifier final : public ShapeReifier {
 public:
  bool is_mesh() const { return is_mesh_; }

  // Implementation of ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Sphere&, void*) final {}
  void ImplementGeometry(const Cylinder&, void*) final {}
  void ImplementGeometry(const Ellipsoid&, void*) final {}
  void ImplementGeometry(const HalfSpace&, void*) final {}
  void ImplementGeometry(const Box&, void*) final {}
  void ImplementGeometry(const Capsule&, void*) final {}
  void ImplementGeometry(const Mesh&, void*) final { is_mesh_ = true; }
  void ImplementGeometry(const Convex&, void*) final {}

 private:
  bool is_mesh_{false};
};

// Helper functions to facilitate exercising FCL's broadphase code. FCL has
// inconsistent usage of `const`. As such, even though the broadphase structures
// do not change during collision and distance queries, they are nevertheless
// declared non-const, requiring Drake to do some const casting in what would
// otherwise be a const context.
template <typename T, typename DataType>
void FclCollide(const fcl::DynamicAABBTreeCollisionManager<double>& tree1,
                const fcl::DynamicAABBTreeCollisionManager<double>& tree2,
                DataType* data, fcl::CollisionCallBack<T> callback) {
  tree1.collide(
      const_cast<fcl::DynamicAABBTreeCollisionManager<T>*>(&tree2), data,
      callback);
}

template <typename T, typename DataType>
void FclDistance(const fcl::DynamicAABBTreeCollisionManager<double>& tree1,
                 const fcl::DynamicAABBTreeCollisionManager<double>& tree2,
                 DataType* data, fcl::DistanceCallBack<T> callback) {
  tree1.distance(
      const_cast<fcl::DynamicAABBTreeCollisionManager<T>*>(&tree2), data,
      callback);
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
    hydroelastic_geometries_ = other.hydroelastic_geometries_;
    dynamic_tree_.clear();
    dynamic_objects_.clear();
    anchored_tree_.clear();
    anchored_objects_.clear();
    dynamic_mesh_tree_.clear();
    dynamic_mesh_objects_.clear();
    anchored_mesh_tree_.clear();
    anchored_mesh_objects_.clear();
    X_MeshBs_.clear();

    // Copy all of the geometry.
    std::unordered_map<const CollisionObjectd*, CollisionObjectd*>
        object_map;
    CopyFclObjectsOrThrow(other.anchored_objects_, &anchored_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(other.dynamic_objects_, &dynamic_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(other.anchored_mesh_objects_, &anchored_mesh_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(other.dynamic_mesh_objects_, &dynamic_mesh_objects_,
                          &object_map);
    X_MeshBs_ = other.X_MeshBs_;

    // Build new AABB trees from the input AABB trees.
    BuildTreeFromReference(other.dynamic_tree_, object_map, &dynamic_tree_);
    BuildTreeFromReference(other.anchored_tree_, object_map, &anchored_tree_);
    BuildTreeFromReference(other.dynamic_mesh_tree_, object_map,
                           &dynamic_mesh_tree_);
    BuildTreeFromReference(other.anchored_mesh_tree_, object_map,
                           &anchored_mesh_tree_);

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
    // Copy all of the geometry.
    std::unordered_map<const CollisionObjectd*, CollisionObjectd*>
        object_map;
    CopyFclObjectsOrThrow(anchored_objects_, &engine->anchored_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(dynamic_objects_, &engine->dynamic_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(anchored_mesh_objects_,
                          &engine->anchored_mesh_objects_, &object_map);
    CopyFclObjectsOrThrow(dynamic_mesh_objects_, &engine->dynamic_mesh_objects_,
                          &object_map);
    engine->X_MeshBs_ = this->X_MeshBs_;

    engine->collision_filter_ = this->collision_filter_;

    // Build new AABB trees from the input AABB trees.
    BuildTreeFromReference(dynamic_tree_, object_map, &engine->dynamic_tree_);
    BuildTreeFromReference(anchored_tree_, object_map, &engine->anchored_tree_);
    BuildTreeFromReference(dynamic_mesh_tree_, object_map,
                           &engine->dynamic_mesh_tree_);
    BuildTreeFromReference(anchored_mesh_tree_, object_map,
                           &engine->anchored_mesh_tree_);

    return engine;
  }

  void AddDynamicGeometry(const Shape& shape, GeometryId id,
                          const ProximityProperties& props) {
    ReifyData data{nullptr, id, props};
    shape.Reify(this, &data);
    EncodedData encoding(id, true /* is dynamic */);
    encoding.write_to(data.fcl_object.get());

    MeshIdentifier mesh_identifier;
    shape.Reify(&mesh_identifier);
    if (!mesh_identifier.is_mesh()) {
      dynamic_tree_.registerObject(data.fcl_object.get());
      dynamic_objects_[id] = std::move(data.fcl_object);
    } else {
      dynamic_mesh_tree_.registerObject(data.fcl_object.get());
      dynamic_mesh_objects_[id] = std::move(data.fcl_object);
    }

    collision_filter_.AddGeometry(encoding.encoding());
  }

  void AddAnchoredGeometry(const Shape& shape, const RigidTransformd& X_WG,
                           GeometryId id, const ProximityProperties& props) {
    ReifyData data{nullptr, id, props};
    shape.Reify(this, &data);
    MeshIdentifier mesh_identifier;
    shape.Reify(&mesh_identifier);

    if (!mesh_identifier.is_mesh()) {
      data.fcl_object->setTransform(X_WG.GetAsIsometry3());
    } else {
      // For a Mesh geometry G, its fcl object is its bounding Box B that has
      // its pose X_GB expressed in G's frame.
      RigidTransformd& X_GB = X_MeshBs_.at(id);
      RigidTransformd X_WB = X_WG * X_GB;
      data.fcl_object->setTransform(X_WB.GetAsIsometry3());
    }
    data.fcl_object->computeAABB();
    EncodedData encoding(id, false /* is dynamic */);
    encoding.write_to(data.fcl_object.get());

    if (!mesh_identifier.is_mesh()) {
      anchored_tree_.registerObject(data.fcl_object.get());
      anchored_tree_.update();
      anchored_objects_[id] = std::move(data.fcl_object);
    } else {
      anchored_mesh_tree_.registerObject(data.fcl_object.get());
      anchored_mesh_tree_.update();
      anchored_mesh_objects_[id] = std::move(data.fcl_object);
    }

    collision_filter_.AddGeometry(encoding.encoding());
  }

  void UpdateRepresentationForNewProperties(
      const InternalGeometry& geometry,
      const ProximityProperties& new_properties) {
    const GeometryId id = geometry.id();
    // Note: Currently, the only aspect of a geometry's representation that can
    // be affected by its proximity properties is its hydroelastic
    // representation.
    if (dynamic_objects_.count(id) == 0 && anchored_objects_.count(id) == 0) {
      throw std::logic_error(
          fmt::format("The proximity engine does not contain a geometry with "
                      "the id {}; its properties cannot be updated",
                      id));
    }

    // TODO(SeanCurtis-TRI): Precondition this with a test -- currently,
    //  I'm mindlessly replacing the old hydroelastic representation with a
    //  new -- even it doesn't actually change. Such an optimization probably
    //  has limited value as this type of operation would really only be done
    //  at initialization.

    // We'll simply mindlessly destroy and recreate the hydroelastic
    // representation.
    hydroelastic_geometries_.RemoveGeometry(id);
    hydroelastic_geometries_.MaybeAddGeometry(geometry.shape(), id,
                                              new_properties);
  }

  void RemoveGeometry(GeometryId id, bool is_dynamic) {
    if (is_dynamic) {
      if (dynamic_objects_.find(id) != dynamic_objects_.end()) {
        RemoveGeometry(id, &dynamic_tree_, &dynamic_objects_);
      } else {
        RemoveGeometry(id, &dynamic_mesh_tree_, &dynamic_mesh_objects_);
      }
    } else {
      if (anchored_objects_.find(id) != anchored_objects_.end()) {
        RemoveGeometry(id, &anchored_tree_, &anchored_objects_);
      } else {
        RemoveGeometry(id, &anchored_mesh_tree_, &anchored_mesh_objects_);
      }
    }
    hydroelastic_geometries_.RemoveGeometry(id);
  }

  int num_geometries() const {
    return static_cast<int>(dynamic_objects_.size() + anchored_objects_.size() +
                            dynamic_mesh_objects_.size() +
                            anchored_mesh_objects_.size());
  }

  int num_dynamic() const {
    return static_cast<int>(dynamic_objects_.size() +
                            dynamic_mesh_objects_.size());
  }

  int num_anchored() const {
    return static_cast<int>(anchored_objects_.size() +
                            anchored_mesh_objects_.size());
  }

  void set_distance_tolerance(double tol) { distance_tolerance_ = tol; }

  double distance_tolerance() const { return distance_tolerance_; }

  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(
      const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs) {
    for (const auto& id_object_pair : dynamic_objects_) {
      const GeometryId id = id_object_pair.first;
      const RigidTransform<T>& X_WG = X_WGs.at(id);
      // The FCL broadphase requires double-valued poses; so we use ADL to
      // efficiently get double-valued poses out of arbitrary T-valued poses.
      dynamic_objects_[id]->setTransform(
          convert_to_double(X_WG).GetAsIsometry3());
      dynamic_objects_[id]->computeAABB();
    }
    dynamic_tree_.update();

    for (const auto& id_object_pair : dynamic_mesh_objects_) {
      const GeometryId id = id_object_pair.first;
      const RigidTransform<T>& X_WG = X_WGs.at(id);
      // For a Mesh G, its fcl object is its bounding Box B that has its pose
      // X_GB expressed in G's frame.
      const RigidTransformd& X_GB = X_MeshBs_.at(id);
      const RigidTransformd X_WB = convert_to_double(X_WG) * X_GB;
      dynamic_mesh_objects_[id]->setTransform(X_WB.GetAsIsometry3());
      dynamic_mesh_objects_[id]->computeAABB();
    }
    dynamic_mesh_tree_.update();
  }

  // Implementation of ShapeReifier interface
  using ShapeReifier::ImplementGeometry;

  // Attempts to process the declared geometry into a hydroelastic
  // representation.
  template <typename Shape>
  void ProcessHydroelastic(const Shape& shape, void* user_data) {
    const ReifyData& data = *static_cast<ReifyData*>(user_data);
    hydroelastic_geometries_.MaybeAddGeometry(shape, data.id, data.properties);
  }

  void ImplementGeometry(const Sphere& sphere, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_sphere = make_shared<fcl::Sphered>(sphere.radius());
    TakeShapeOwnership(fcl_sphere, user_data);
    ProcessHydroelastic(sphere, user_data);
  }

  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_cylinder = make_shared<fcl::Cylinderd>(cylinder.radius(),
                                                    cylinder.length());
    TakeShapeOwnership(fcl_cylinder, user_data);
    ProcessHydroelastic(cylinder, user_data);
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) override {
    static const logging::Warn log_once(
        "Ellipsoid is primarily for ComputeContactSurfaces in hydroelastic "
        "contact model. The accuracy of other collision queries and signed "
        "distance queries are not guaranteed.");
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_ellipsoid = make_shared<fcl::Ellipsoidd>(
        ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
    TakeShapeOwnership(fcl_ellipsoid, user_data);
    ProcessHydroelastic(ellipsoid, user_data);
  }

  void ImplementGeometry(const HalfSpace& half_space,
                         void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_half_space = make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    TakeShapeOwnership(fcl_half_space, user_data);
    ProcessHydroelastic(half_space, user_data);
  }

  void ImplementGeometry(const Box& box, void* user_data) override {
    auto fcl_box = make_shared<fcl::Boxd>(box.size());
    TakeShapeOwnership(fcl_box, user_data);
    ProcessHydroelastic(box, user_data);
  }

  void ImplementGeometry(const Capsule& capsule, void* user_data) override {
    static const logging::Warn log_once(
        "Capsule is currently not supported in hydroelastic contact model. "
        "It is available for collision queries and pairwise signed distance "
        "queries.");
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_capsule =
        make_shared<fcl::Capsuled>(capsule.radius(), capsule.length());
    TakeShapeOwnership(fcl_capsule, user_data);
    ProcessHydroelastic(capsule, user_data);
  }

  // Convert Mesh specification to fcl representation and hydroelastic
  // representation. The fcl representation of the mesh is a box for
  // broadphase culling because meshes are not supported in other proximity
  // queries except ComputeContactSurfaces.
  void ImplementGeometry(const Mesh& mesh, void* user_data) override {
    static const logging::Warn log_once(
        "Mesh is only for ComputeContactSurfaces in hydroelastic contact "
        "model. It is _not_ available in other proximity queries.");
    SurfaceMesh<double> surface =
        ReadObjToSurfaceMesh(mesh.filename(), mesh.scale());
    auto [center, size] = surface.CalcBoundingBox();
    auto fcl_box = make_shared<fcl::Boxd>(size);

    TakeShapeOwnership(fcl_box, user_data);
    // Store the pose X_MB of the bounding box B expressed in mesh's frame M.
    // Since B is axis-aligned, X_MB is simply a translation to B's center.
    RigidTransformd X_MB(center);
    X_MeshBs_[static_cast<ReifyData*>(user_data)->id] = X_MB;
    ProcessHydroelastic(mesh, user_data);
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
    for (int num : mesh.num_face_vertices) {
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

    // Tinyobj doesn't infer the search directory from the directory containing
    // the obj file. We have to provide that directory; of course, this assumes
    // that the material library reference is relative to the obj directory.
    const size_t pos = convex.filename().find_last_of('/');
    const std::string obj_folder = convex.filename().substr(0, pos + 1);
    const char* mtl_basedir = obj_folder.c_str();

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
    ProcessHydroelastic(convex, user_data);

    // TODO(DamrongGuoy): Per f2f with SeanCurtis-TRI, we want ProximityEngine
    // to own vertices and face by a map from filename.  This way we won't have
    // to read the same file again and again when we create multiple Convex
    // objects from the same file.
  }

  std::vector<SignedDistancePair<T>> ComputeSignedDistancePairwiseClosestPoints(
      const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
      const double max_distance) const {
    std::vector<SignedDistancePair<T>> witness_pairs;
    // All these quantities are aliased in the callback data.
    shape_distance::CallbackData<T> data{&collision_filter_, &X_WGs,
                                         max_distance, &witness_pairs};
    data.request.enable_nearest_points = true;
    data.request.enable_signed_distance = true;
    data.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    data.request.distance_tolerance = distance_tolerance_;

    // Perform a query of the dynamic objects against themselves.
    dynamic_tree_.distance(&data, shape_distance::Callback<T>);

    // Perform a query of the dynamic objects against the anchored. We don't do
    // anchored against anchored because those pairs are implicitly filtered.
    FclDistance(dynamic_tree_, anchored_tree_, &data,
                shape_distance::Callback<T>);
    return witness_pairs;
  }

  SignedDistancePair<T> ComputeSignedDistancePairClosestPoints(
      GeometryId id_A, GeometryId id_B,
      const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs) const {
    std::vector<SignedDistancePair<T>> witness_pairs;
    double max_distance = std::numeric_limits<double>::infinity();
    // All these quantities are aliased in the callback data.
    shape_distance::CallbackData<T> data{&collision_filter_, &X_WGs,
                                         max_distance, &witness_pairs};
    data.request.enable_nearest_points = true;
    data.request.enable_signed_distance = true;
    data.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    data.request.distance_tolerance = distance_tolerance_;

    auto find_geometry = [this](GeometryId id) -> CollisionObjectd* {
      auto iter = dynamic_objects_.find(id);
      if (iter == dynamic_objects_.end()) {
        iter = anchored_objects_.find(id);
        if (iter == anchored_objects_.end()) {
          throw std::runtime_error(fmt::format(
              "The geometry given by id {} does not reference a "
              "geometry that can be used in a signed distance query",
              id));
        }
      }
      return const_cast<CollisionObjectd*>(iter->second.get());
    };

    CollisionObjectd* object_A = find_geometry(id_A);
    CollisionObjectd* object_B = find_geometry(id_B);
    shape_distance::Callback<T>(object_A, object_B, &data, max_distance);

    if (witness_pairs.size() == 0) {
      throw std::runtime_error(fmt::format(
          "The geometry pair ({}, {}) does not support a signed distance query",
          id_A, id_B));
    }
    return witness_pairs[0];
  }

  std::vector<SignedDistanceToPoint<T>> ComputeSignedDistanceToPoint(
      const Vector3<T>& p_WQ,
      const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
      const double threshold) const {
    // We create a sphere of zero radius centered at the query point and put
    // it into a CollisionObject.
    auto fcl_sphere = make_shared<fcl::Sphered>(0.0);  // sphere of zero radius
    CollisionObjectd query_point(fcl_sphere);
    // The FCL broadphase requires double-valued poses; so we use ADL to
    // efficiently get double-valued poses out of arbitrary T-valued poses.
    query_point.setTranslation(convert_to_double(p_WQ));
    query_point.computeAABB();

    std::vector<SignedDistanceToPoint<T>> distances;

    point_distance::CallbackData<T> data{
        &query_point, threshold, p_WQ, &X_WGs, &distances};

    // Perform query of point vs dynamic objects.
    dynamic_tree_.distance(&query_point, &data, point_distance::Callback<T>);

    // Perform query of point vs anchored objects.
    anchored_tree_.distance(&query_point, &data, point_distance::Callback<T>);

    return distances;
  }

  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration()
      const {
    std::vector<PenetrationAsPointPair<double>> contacts;
    penetration_as_point_pair::CallbackData data{&collision_filter_, &contacts};

    // Perform a query of the dynamic objects against themselves.
    dynamic_tree_.collide(&data, penetration_as_point_pair::Callback);

    // Perform a query of the dynamic objects against the anchored. We don't do
    // anchored against anchored because those pairs are implicitly filtered.
    FclCollide(dynamic_tree_, anchored_tree_, &data,
               penetration_as_point_pair::Callback);

    return contacts;
  }

  std::vector<SortedPair<GeometryId>> FindCollisionCandidates() const {
    std::vector<SortedPair<GeometryId>> pairs;
    // All these quantities are aliased in the callback data.
    find_collision_candidates::CallbackData data{&collision_filter_, &pairs};

    // Perform a query of the dynamic objects against themselves.
    dynamic_tree_.collide(&data, find_collision_candidates::Callback);

    // Perform a query of the dynamic objects against the anchored. We don't do
    // anchored against anchored because those pairs are implicitly filtered.
    FclCollide(dynamic_tree_, anchored_tree_, &data,
               find_collision_candidates::Callback);
    return pairs;
  }

  bool HasCollisions() const {
    // All these quantities are aliased in the callback data.
    has_collisions::CallbackData data{&collision_filter_};

    // Perform a query of the dynamic objects against themselves.
    dynamic_tree_.collide(&data, has_collisions::Callback);

    // Perform a query of the dynamic objects against the anchored. We don't do
    // anchored against anchored because those pairs are implicitly filtered.
    FclCollide(dynamic_tree_, anchored_tree_, &data, has_collisions::Callback);
    return data.collisions_exist;
  }

  vector<ContactSurface<T>> ComputeContactSurfaces(
      const unordered_map<GeometryId, RigidTransform<T>>& X_WGs) const {
    vector<ContactSurface<T>> surfaces;
    // All these quantities are aliased in the callback data.
    hydroelastic::CallbackData<T> data{&collision_filter_, &X_WGs,
                                       &hydroelastic_geometries_, &surfaces};

    // Perform a query of the dynamic objects against themselves.
    dynamic_tree_.collide(&data, hydroelastic::Callback<T>);

    // Perform a query of the dynamic objects against the anchored. We don't do
    // anchored against anchored because those pairs are implicitly filtered.
    FclCollide(dynamic_tree_, anchored_tree_, &data, hydroelastic::Callback<T>);
    FclCollide(dynamic_tree_, anchored_mesh_tree_, &data,
               hydroelastic::Callback<T>);
    FclCollide(dynamic_tree_, dynamic_mesh_tree_, &data,
               hydroelastic::Callback<T>);

    dynamic_mesh_tree_.collide(&data, hydroelastic::Callback<T>);
    FclCollide(dynamic_mesh_tree_, anchored_tree_, &data,
               hydroelastic::Callback<T>);
    FclCollide(dynamic_mesh_tree_, anchored_mesh_tree_, &data,
               hydroelastic::Callback<T>);

    return surfaces;
  }

  void ComputeContactSurfacesWithFallback(
      const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
      std::vector<ContactSurface<T>>* surfaces,
      std::vector<PenetrationAsPointPair<double>>* point_pairs) const {
    DRAKE_DEMAND(surfaces);
    DRAKE_DEMAND(point_pairs);
    // All these quantities are aliased in the callback data.
    hydroelastic::CallbackWithFallbackData<T> data{
        hydroelastic::CallbackData<T>{&collision_filter_, &X_WGs,
                                      &hydroelastic_geometries_, surfaces},
        point_pairs};

    // Dynamic vs dynamic and dynamic vs anchored represent all the geometries
    // that we can support with the point-pair fallback. Do those first.
    dynamic_tree_.collide(&data, hydroelastic::CallbackWithFallback<T>);

    FclCollide(dynamic_tree_, anchored_tree_, &data,
               hydroelastic::CallbackWithFallback<T>);

    // TODO(SeanCurtis-TRI): There is a special case where the error message is
    //  incomprehensible. If someone _attempts_ to register a soft mesh, the
    //  registration will broadcast a single warning, but no hydroleastic
    //  representation will be created. If that mesh is ever in contact, the
    //  error message will be that a _Box_ is missing a hydroelastic
    //  representation. It really should say *mesh*. Somehow, we have to know
    //  that the box is really the broadphase place-holder of a mesh. Update
    //  proximity_engine_test.cc, ComputeContactSurfaceWithFallback when this
    //  issue is resolved.

    // dynamic_mesh_tree_ and anchored_mesh_tree_ contain meshes that *can't*
    // fall back to point-pair (we don't support meshes in point-pair contact).
    // So, we default to the strict hydroleastic. Each pair generated in the
    // following broadphase calculations *must* include a mesh. If we can't
    // compute a contact surface, we must fail.
    FclCollide(dynamic_tree_, anchored_mesh_tree_, &data.data,
               hydroelastic::Callback<T>);
    FclCollide(dynamic_tree_, dynamic_mesh_tree_, &data.data,
               hydroelastic::Callback<T>);

    dynamic_mesh_tree_.collide(&data, hydroelastic::Callback<T>);
    FclCollide(dynamic_mesh_tree_, anchored_tree_, &data.data,
               hydroelastic::Callback<T>);
    FclCollide(dynamic_mesh_tree_, anchored_mesh_tree_, &data.data,
               hydroelastic::Callback<T>);
  }

  // TODO(SeanCurtis-TRI): Update this with the new collision filter method.
  void ExcludeCollisionsWithin(
      const std::unordered_set<GeometryId>& dynamic,
      const std::unordered_set<GeometryId>& anchored) {
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
      for (auto id : dynamic) {
        EncodedData encoding(id, true /* is dynamic */);
        collision_filter_.AddToCollisionClique(encoding.encoding(), clique);
      }
      for (auto id : anchored) {
        EncodedData encoding(id, false /* is dynamic */);
        collision_filter_.AddToCollisionClique(encoding.encoding(), clique);
      }
    }
  }

  void ExcludeCollisionsBetween(
      const std::unordered_set<GeometryId>& dynamic1,
      const std::unordered_set<GeometryId>& anchored1,
      const std::unordered_set<GeometryId>& dynamic2,
      const std::unordered_set<GeometryId>& anchored2) {
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

  bool CollisionFiltered(GeometryId id1, bool is_dynamic_1,
                         GeometryId id2, bool is_dynamic_2) const {
    // Collisions between anchored geometries are implicitly filtered.
    if (!is_dynamic_1 && !is_dynamic_2) return true;
    EncodedData encoding1(id1, is_dynamic_1);
    EncodedData encoding2(id2, is_dynamic_2);
    return !collision_filter_.CanCollideWith(encoding1.encoding(),
                                             encoding2.encoding());
  }

  int get_next_clique() { return collision_filter_.next_clique_id(); }

  void set_clique(GeometryId id, int clique) {
    EncodedData encoding(id, true /* is dynamic */);
    collision_filter_.AddToCollisionClique(encoding.encoding(), clique);
  }

  // Testing utilities

  bool IsDeepCopy(const Impl& other) const {
    if (this != &other) {
      // TODO(DamrongGuoy): Consider checking other data members such as
      //  [dynamic|anchored]_[mesh]_tree_, hydroelastic_geometries_,
      //  collision_filter_, and X_MeshBs_.
      auto are_maps_deep_copy =
          [](const unordered_map<GeometryId, unique_ptr<CollisionObjectd>>&
                 this_map,
             const unordered_map<GeometryId, unique_ptr<CollisionObjectd>>&
                 other_map) -> bool {
        if (this_map.size() != other_map.size()) {
          return false;
        }
        for (const auto& id_object_pair : this_map) {
          const GeometryId test_id = id_object_pair.first;
          const CollisionObjectd& test = *id_object_pair.second;
          if (other_map.find(test_id) == other_map.end()) {
            return false;
          }
          const CollisionObjectd& ref = *other_map.at(test_id);
          // Validate that two objects are equal. The test isn't exhaustive
          // (for example, the parameters of the particular geometric shape
          // are not compared--instead, we compare the AABBs).
          bool objects_equal =
              test.getUserData() == ref.getUserData() &&
              test.getNodeType() == ref.getNodeType() &&
              test.getObjectType() == ref.getObjectType() &&
              test.getAABB().center() == ref.getAABB().center() &&
              test.getAABB().width() == ref.getAABB().width() &&
              test.getAABB().height() == ref.getAABB().height() &&
              test.getAABB().depth() == ref.getAABB().depth();
          if (!objects_equal) {
            return false;
          }
        }
        return true;
      };  // are_maps_deep_copy

      if (!are_maps_deep_copy(this->dynamic_objects_, other.dynamic_objects_)) {
        return false;
      }
      if (!are_maps_deep_copy(this->anchored_objects_,
                              other.anchored_objects_)) {
        return false;
      }
      if (!are_maps_deep_copy(this->dynamic_mesh_objects_,
                              other.dynamic_mesh_objects_)) {
        return false;
      }
      if (!are_maps_deep_copy(this->anchored_mesh_objects_,
                              other.anchored_mesh_objects_)) {
        return false;
      }
      return true;
    }
    return false;
  }

  int peek_next_clique() const { return collision_filter_.peek_next_clique(); }

  const RigidTransformd GetX_WG(GeometryId id, bool is_dynamic) const {
    const unordered_map<GeometryId, unique_ptr<CollisionObjectd>>& objects =
        is_dynamic ? (dynamic_objects_.find(id) != dynamic_objects_.end())
                         ? dynamic_objects_
                         : dynamic_mesh_objects_
                   : (anchored_objects_.find(id) != anchored_objects_.end())
                         ? anchored_objects_
                         : anchored_mesh_objects_;

    return RigidTransformd(objects.at(id)->getTransform());
  }

  const hydroelastic::Geometries& hydroelastic_geometries() const {
    return hydroelastic_geometries_;
  }

 private:
  // Engine on one scalar can see the members of other engines.
  friend class ProximityEngineTester;
  template <typename>
  friend class ProximityEngine;

  // Removes the geometry with the given id from the given tree.
  void RemoveGeometry(
      GeometryId id, fcl::DynamicAABBTreeCollisionManager<double>* tree,
      unordered_map<GeometryId, unique_ptr<CollisionObjectd>>* geometries) {
    unordered_map<GeometryId, unique_ptr<CollisionObjectd>>& typed_geometries =
        *geometries;
    CollisionObjectd* fcl_object = typed_geometries.at(id).get();
    const size_t old_size = tree->size();
    tree->unregisterObject(fcl_object);
    EncodedData filter_key(*fcl_object);
    collision_filter_.RemoveGeometry(filter_key.encoding());
    typed_geometries.erase(id);
    // NOTE: The FCL API provides no other mechanism for confirming the
    // unregistration was successful.
    DRAKE_DEMAND(old_size == tree->size() + 1);
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
    ReifyData& reify_data = *static_cast<ReifyData*>(data);
    reify_data.fcl_object = make_unique<CollisionObjectd>(shape);
  }

  // The BVH of all dynamic geometries; this depends on *all* inputs.
  // TODO(SeanCurtis-TRI): Ultimately, this should probably be a cache entry.
  fcl::DynamicAABBTreeCollisionManager<double> dynamic_tree_;

  // All of the *dynamic* collision elements (spanning all sources).
  unordered_map<GeometryId, unique_ptr<CollisionObjectd>> dynamic_objects_;

  // The tree containing all of the anchored geometry.
  fcl::DynamicAABBTreeCollisionManager<double> anchored_tree_;

  // All of the *anchored* collision elements (spanning *all* sources).
  unordered_map<GeometryId, unique_ptr<CollisionObjectd>> anchored_objects_;

  // The mechanism for dictating collision filtering.
  CollisionFilterLegacy collision_filter_;

  // The tolerance that determines when the iterative process would terminate.
  // @see ProximityEngine::set_distance_tolerance() for more details.
  double distance_tolerance_{1E-6};

  // All of the hydroelastic representations of supported geometries -- this
  // can get quite large based on mesh resolution.
  hydroelastic::Geometries hydroelastic_geometries_;

  // FCL's mesh representation (fcl::BVHModel) uses a triangle soup without
  // the concept of enclosing volume (there is no inside and outside). We
  // cannot use FCL's mesh representation for general proximity queries but
  // want to allow rigid meshes for hydroelastic contact, therefore:
  //
  // 1. We represent drake::geometry::Mesh M using its bounding box B in FCL as
  //    fcl::Boxd in the AABBTree in FCL, in order to get the advantages of
  //    broadphase culling and to be compatible with the hydroelastic
  //    callback infrastructure.
  // 2. The bounding box B has its pose X_MB expressed in the frame M of the
  //    mesh. This allows the center of the box to be far from the origin of
  //    the mesh's frame.  We keep all X_MB of all bounding boxes of the
  //    meshes in X_MeshBs_ below.
  // 3. Currently Mesh is supported in ComputeContactSurfaces() only, so we
  //    keep their FCL representations in separated AABBTree structures
  //    (dynamic_mesh_tree_, dynamic_mesh_objects_, anchored_mesh_tree_,
  //    anchored_mesh_objects_) and use them only in ComputeContactSurfaces()
  //    but not in other proximity queries.
  // TODO(DamrongGuoy): Merge these mesh-specific data into the main
  //  dynamic_tree_ and anchored_tree when:
  //  1. We have a direct collision-object representation for Mesh in the
  //     broadphase culling, and
  //  2. We have narrowphase support for Mesh in other proximity queries.
  unordered_map<GeometryId, RigidTransformd> X_MeshBs_;
  fcl::DynamicAABBTreeCollisionManager<double> dynamic_mesh_tree_;
  unordered_map<GeometryId, unique_ptr<CollisionObjectd>> dynamic_mesh_objects_;
  fcl::DynamicAABBTreeCollisionManager<double> anchored_mesh_tree_;
  unordered_map<GeometryId, unique_ptr<CollisionObjectd>>
      anchored_mesh_objects_;
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
void ProximityEngine<T>::AddDynamicGeometry(const Shape& shape, GeometryId id,
                                            const ProximityProperties& props) {
  impl_->AddDynamicGeometry(shape, id, props);
}

template <typename T>
void ProximityEngine<T>::AddAnchoredGeometry(
    const Shape& shape, const RigidTransformd& X_WG, GeometryId id,
    const ProximityProperties& props) {
  impl_->AddAnchoredGeometry(shape, X_WG, id, props);
}

template <typename T>
void ProximityEngine<T>::UpdateRepresentationForNewProperties(
    const InternalGeometry& geometry,
    const ProximityProperties& new_properties) {
  impl_->UpdateRepresentationForNewProperties(geometry, new_properties);
}

template <typename T>
void ProximityEngine<T>::RemoveGeometry(GeometryId id, bool is_dynamic) {
  impl_->RemoveGeometry(id, is_dynamic);
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
    const unordered_map<GeometryId, RigidTransform<T>>& X_WGs) {
  impl_->UpdateWorldPoses(X_WGs);
}

template <typename T>
std::vector<SignedDistancePair<T>>
ProximityEngine<T>::ComputeSignedDistancePairwiseClosestPoints(
    const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
    const double max_distance) const {
  return impl_->ComputeSignedDistancePairwiseClosestPoints(X_WGs, max_distance);
}

template <typename T>
SignedDistancePair<T>
ProximityEngine<T>::ComputeSignedDistancePairClosestPoints(
    GeometryId id_A, GeometryId id_B,
    const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs)
    const {
  return impl_->ComputeSignedDistancePairClosestPoints(id_A, id_B, X_WGs);
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
ProximityEngine<T>::ComputeSignedDistanceToPoint(
    const Vector3<T>& query,
    const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
    const double threshold) const {
  return impl_->ComputeSignedDistanceToPoint(query, X_WGs, threshold);
}

template <typename T>
bool ProximityEngine<T>::HasCollisions() const {
  return impl_->HasCollisions();
}

template <typename T>
std::vector<PenetrationAsPointPair<double>>
ProximityEngine<T>::ComputePointPairPenetration() const {
  return impl_->ComputePointPairPenetration();
}

template <typename T>
std::vector<ContactSurface<T>> ProximityEngine<T>::ComputeContactSurfaces(
    const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs) const {
  return impl_->ComputeContactSurfaces(X_WGs);
}

template <typename T>
void ProximityEngine<T>::ComputeContactSurfacesWithFallback(
    const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
    std::vector<ContactSurface<T>>* surfaces,
    std::vector<PenetrationAsPointPair<double>>* point_pairs) const {
  return impl_->ComputeContactSurfacesWithFallback(X_WGs, surfaces,
                                                   point_pairs);
}

template <typename T>
std::vector<SortedPair<GeometryId>>
ProximityEngine<T>::FindCollisionCandidates() const {
  return impl_->FindCollisionCandidates();
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsWithin(
    const std::unordered_set<GeometryId>& dynamic,
    const std::unordered_set<GeometryId>& anchored) {
  impl_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsBetween(
    const std::unordered_set<GeometryId>& dynamic1,
    const std::unordered_set<GeometryId>& anchored1,
    const std::unordered_set<GeometryId>& dynamic2,
    const std::unordered_set<GeometryId>& anchored2) {
  impl_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2, anchored2);
}

template <typename T>
bool ProximityEngine<T>::CollisionFiltered(
    GeometryId id1, bool is_dynamic_1,
    GeometryId id2, bool is_dynamic_2) const {
  return impl_->CollisionFiltered(id1, is_dynamic_1, id2, is_dynamic_2);
}

// Client-attorney interface for GeometryState to manipulate collision filters.

template <typename T>
int ProximityEngine<T>::get_next_clique() {
  return impl_->get_next_clique();
}

template <typename T>
void ProximityEngine<T>::set_clique(GeometryId id, int clique) {
  impl_->set_clique(id, clique);
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
const RigidTransformd ProximityEngine<T>::GetX_WG(GeometryId id,
                                                  bool is_dynamic) const {
  return impl_->GetX_WG(id, is_dynamic);
}

template <typename T>
const hydroelastic::Geometries& ProximityEngine<T>::hydroelastic_geometries()
    const {
  return impl_->hydroelastic_geometries();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::ProximityEngine)
