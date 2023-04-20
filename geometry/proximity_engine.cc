#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <drake_vendor/fcl/fcl.h>
#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collisions_exist_callback.h"
#include "drake/geometry/proximity/deformable_contact_geometries.h"
#include "drake/geometry/proximity/deformable_contact_internal.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/distance_to_shape_callback.h"
#include "drake/geometry/proximity/find_collision_candidates_callback.h"
#include "drake/geometry/proximity/hydroelastic_callback.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity/vtk_to_volume_mesh.h"
#include "drake/geometry/read_obj.h"
#include "drake/geometry/utilities.h"

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
using symbolic::Expression;

namespace {

// Drake compiles FCL using hidden symbol visibility. To avoid visibility
// complaints from the compiler, we need to use hidden subclasses for any
// FCL data types used as member fields of ProximityEngine::Impl. Note
// that FCL Objects on the stack are fine without worrying about hidden;
// it's only Impl member fields that cause trouble.
class FclDynamicAABBTreeCollisionManager
    : public fcl::DynamicAABBTreeCollisionManager<double> {};
class MapGeometryIdToFclCollisionObject
    : public unordered_map<GeometryId, unique_ptr<CollisionObjectd>> {};

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
  const RigidTransformd X_WG;
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

// Compare function to use with ordering PenetrationAsPointPairs.
template <typename T>
bool OrderPointPair(const PenetrationAsPointPair<T>& p1,
                    const PenetrationAsPointPair<T>& p2) {
  if (p1.id_A != p2.id_A) return p1.id_A < p2.id_A;
  return p1.id_B < p2.id_B;
}

// Compare function to use with ordering ContactSurfaces.
template <typename T>
bool OrderContactSurface(const ContactSurface<T>& s1,
                         const ContactSurface<T>& s2) {
  if (s1.id_M() != s2.id_M()) return s1.id_M() < s2.id_M();
  return s1.id_N() < s2.id_N();
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
    geometries_for_deformable_contact_ =
        other.geometries_for_deformable_contact_;
    dynamic_tree_.clear();
    dynamic_objects_.clear();
    anchored_tree_.clear();
    anchored_objects_.clear();

    // Copy all of the geometry.
    std::unordered_map<const CollisionObjectd*, CollisionObjectd*>
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

  template <typename U>
  std::unique_ptr<typename ProximityEngine<U>::Impl> ToScalarType() const {
    auto engine = make_unique<typename ProximityEngine<U>::Impl>();

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

    engine->collision_filter_ = this->collision_filter_;

    // Build new AABB trees from the input AABB trees.
    BuildTreeFromReference(dynamic_tree_, object_map, &engine->dynamic_tree_);
    BuildTreeFromReference(anchored_tree_, object_map, &engine->anchored_tree_);

    engine->hydroelastic_geometries_ = this->hydroelastic_geometries_;
    engine->geometries_for_deformable_contact_ =
        this->geometries_for_deformable_contact_;
    engine->distance_tolerance_ = this->distance_tolerance_;

    return engine;
  }

  CollisionFilter& collision_filter() { return collision_filter_; }

  void AddDynamicGeometry(const Shape& shape, const RigidTransformd& X_WG,
                          GeometryId id, const ProximityProperties& props) {
    AddGeometry(shape, X_WG, id, props, true, &dynamic_tree_,
                &dynamic_objects_);
  }

  void AddAnchoredGeometry(const Shape& shape, const RigidTransformd& X_WG,
                           GeometryId id, const ProximityProperties& props) {
    AddGeometry(shape, X_WG, id, props, false, &anchored_tree_,
                &anchored_objects_);
  }

  void AddDeformableGeometry(const VolumeMesh<double>& mesh_W, GeometryId id) {
    geometries_for_deformable_contact_.AddDeformableGeometry(id, mesh_W);
    // Currently, even though no collision filtering is done for deformable
    // geometries, the collision filter still needs to be aware of the existence
    // of deformable geometries. This is because collision filters implicitly
    // assumes that it is aware of all geometries registered with the proximity
    // engine.
    // Currently, all deformable geometries are registered with the world
    // frame, if the collision filter isn't aware of the geometry, we will
    // trigger an assertion when adding geometries whose collision with
    // world-framed geometries are filtered out.
    collision_filter_.AddGeometry(id);
  }

  void UpdateRepresentationForNewProperties(
      const InternalGeometry& geometry,
      const ProximityProperties& new_properties) {
    const GeometryId id = geometry.id();
    // Note: Currently, the only aspects of a geometry's representation that can
    // be affected by its proximity properties are its hydroelastic
    // representation and rigid (non-deformable) representation for deformable
    // contact.
    if (!IsRegisteredAsDeformable(id) && !IsRegisteredAsRigid(id)) {
      throw std::logic_error(
          fmt::format("The proximity engine does not contain a geometry with "
                      "the id {}; its properties cannot be updated",
                      id));
    }
    if (IsRegisteredAsDeformable(id)) {
      // Since deformable geometries currently don't depend on proximity
      // properties for anything we simply return.
      return;
    }
    // TODO(SeanCurtis-TRI): Precondition this with a test -- currently,
    //  I'm mindlessly replacing the old representation (for hydroelastic and
    //  for rigid geometries participating in deformable contact) with a new
    //  one -- even it doesn't actually change. Such an optimization probably
    //  has limited value as this type of operation would really only be done
    //  at initialization.

    // We'll simply mindlessly destroy and recreate the hydroelastic and
    // deformable contact representations of rigid (non-deformable)
    // geometries.
    hydroelastic_geometries_.RemoveGeometry(id);
    hydroelastic_geometries_.MaybeAddGeometry(geometry.shape(), id,
                                              new_properties);
    const RigidTransformd X_WG = GetX_WG(id, geometry.is_dynamic());
    geometries_for_deformable_contact_.RemoveGeometry(id);
    geometries_for_deformable_contact_.MaybeAddRigidGeometry(
        geometry.shape(), id, new_properties, X_WG);
  }

  // Returns true if the geometry with the given Id has been registered in
  // `this` ProximityEngine as a deformable geometry (via
  // "AddDeformableGeometry()") and has not been since removed (via
  // "RemoveDeformableGeometry()").
  bool IsRegisteredAsDeformable(GeometryId id) {
    return geometries_for_deformable_contact_.is_deformable(id);
  }

  // Returns true if the geometry with the given Id has been registered in
  // `this` ProximityEngine as a rigid (non-deformable) geometry (via
  // "AddDynamicGeometry() or AddAnchoredGeometry()") and has not been since
  // removed (via "RemoveGeometry()").
  bool IsRegisteredAsRigid(GeometryId id) {
    return dynamic_objects_.count(id) > 0 || anchored_objects_.count(id) > 0;
  }

  // Removes a non-deformable geometry from this engine.
  void RemoveGeometry(GeometryId id, bool is_dynamic) {
    if (is_dynamic) {
      RemoveGeometry(id, &dynamic_tree_, &dynamic_objects_);
    } else {
      RemoveGeometry(id, &anchored_tree_, &anchored_objects_);
    }
    hydroelastic_geometries_.RemoveGeometry(id);
    geometries_for_deformable_contact_.RemoveGeometry(id);
  }

  void RemoveDeformableGeometry(GeometryId id) {
    if (!geometries_for_deformable_contact_.is_deformable(id)) {
      throw std::logic_error(fmt::format(
          "The proximity engine does not contain a deformable geometry with "
          "the id {}; it cannot be removed.",
          id));
    }
    geometries_for_deformable_contact_.RemoveGeometry(id);
  }

  // Returns the total number of **rigid** geometries in this engine.
  int num_geometries() const {
    return num_dynamic() + num_anchored();
  }

  int num_dynamic() const {
    return static_cast<int>(dynamic_objects_.size());
  }

  int num_anchored() const {
    return static_cast<int>(anchored_objects_.size());
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
      const RigidTransform<double>& X_WG_d = convert_to_double(X_WG);
      dynamic_objects_[id]->setTransform(X_WG_d.GetAsIsometry3());
      dynamic_objects_[id]->computeAABB();
      geometries_for_deformable_contact_.UpdateRigidWorldPose(id, X_WG_d);
    }
    dynamic_tree_.update();
  }

  void UpdateDeformableVertexPositions(
      const std::unordered_map<GeometryId, VectorX<T>>& q_WGs) {
    for (const auto& [id, q_WG] : q_WGs) {
      geometries_for_deformable_contact_.UpdateDeformableVertexPositions(
          id, ExtractDoubleOrThrow(q_WG));
    }
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

  // Attempts to process the declared geometry into a rigid representation for
  // deformable contact.
  template <typename Shape>
  void ProcessGeometriesForDeformableContact(const Shape& shape,
                                             void* user_data) {
    const ReifyData& data = *static_cast<ReifyData*>(user_data);

    geometries_for_deformable_contact_.MaybeAddRigidGeometry(
        shape, data.id, data.properties, data.X_WG);
  }

  void ImplementGeometry(const Box& box, void* user_data) override {
    auto fcl_box = make_shared<fcl::Boxd>(box.size());
    TakeShapeOwnership(fcl_box, user_data);
    ProcessHydroelastic(box, user_data);
    ProcessGeometriesForDeformableContact(box, user_data);
  }

  void ImplementGeometry(const Capsule& capsule, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_capsule =
        make_shared<fcl::Capsuled>(capsule.radius(), capsule.length());
    TakeShapeOwnership(fcl_capsule, user_data);
    ProcessHydroelastic(capsule, user_data);
    ProcessGeometriesForDeformableContact(capsule, user_data);
  }

  void ImplementGeometry(const Convex& convex, void* user_data) override {
    // Don't bother triangulating; Convex supports polygons.
    const auto [vertices, faces, num_faces] =
        ReadObjFile(convex.filename(), convex.scale(), false /* triangulate */);

    // Create fcl::Convex.
    auto fcl_convex = make_shared<fcl::Convexd>(vertices, num_faces, faces);

    TakeShapeOwnership(fcl_convex, user_data);
    ProcessHydroelastic(convex, user_data);
    ProcessGeometriesForDeformableContact(convex, user_data);

    // TODO(DamrongGuoy): Per f2f with SeanCurtis-TRI, we want ProximityEngine
    // to own vertices and face by a map from filename.  This way we won't have
    // to read the same file again and again when we create multiple Convex
    // objects from the same file.
  }

  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_cylinder = make_shared<fcl::Cylinderd>(cylinder.radius(),
                                                    cylinder.length());
    TakeShapeOwnership(fcl_cylinder, user_data);
    ProcessHydroelastic(cylinder, user_data);
    ProcessGeometriesForDeformableContact(cylinder, user_data);
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_ellipsoid = make_shared<fcl::Ellipsoidd>(
        ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
    TakeShapeOwnership(fcl_ellipsoid, user_data);
    ProcessHydroelastic(ellipsoid, user_data);
    ProcessGeometriesForDeformableContact(ellipsoid, user_data);
  }

  void ImplementGeometry(const HalfSpace& half_space,
                         void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_half_space = make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    TakeShapeOwnership(fcl_half_space, user_data);
    ProcessHydroelastic(half_space, user_data);
    ProcessGeometriesForDeformableContact(half_space, user_data);
  }

  void ImplementGeometry(const Mesh& mesh, void* user_data) override {
    const ReifyData& data = *static_cast<ReifyData*>(user_data);
    const HydroelasticType type = data.properties.GetPropertyOrDefault(
        kHydroGroup, kComplianceType, HydroelasticType::kUndefined);

    // We process hydroelastic geometry first, so we have access to mesh
    // vertices that we will pass to FCL later without reading the mesh
    // file again.
    ProcessHydroelastic(mesh, user_data);
    shared_ptr<const std::vector<Vector3d>> shared_verts;
    if (type == HydroelasticType::kSoft) {
      shared_verts = make_shared<const std::vector<Vector3d>>(
          ConvertVolumeToSurfaceMesh(
              hydroelastic_geometries_.soft_geometry(data.id).mesh())
              .vertices());
    } else if (type == HydroelasticType::kRigid) {
      shared_verts = make_shared<const std::vector<Vector3d>>(
          hydroelastic_geometries_.rigid_geometry(data.id).mesh().vertices());
    } else {
      std::string extension =
          std::filesystem::path(mesh.filename()).extension();
      std::transform(extension.begin(), extension.end(), extension.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      if (extension != ".obj") {
        throw std::runtime_error(
            fmt::format("ProximityEngine: expect an Obj file for "
                        "non-hydroelastics but get {} file ({}) instead.",
                        extension, mesh.filename()));
      }
      // TODO(SeanCurtis-TRI) Add a troubleshooting entry to give more helpful
      //  advice.

      // Don't bother triangulating; we're ignoring the faces.
      std::tie(shared_verts, std::ignore, std::ignore) =
          ReadObjFile(mesh.filename(), mesh.scale(), false /* triangulate */);
    }

    // Note: the strategy here is to use an *invalid* fcl::Convex shape for the
    // mesh. A minimum condition for "invalid" is that the convex specification
    // contains vertices that are not referenced by a face. Passing zero faces
    // will accomplish that. GJK asks Convex for a "supporting vertex" in a
    // particular direction. "Invalid" Convex instances find that vertex by
    // doing a linear search through all vertices. "Valid" looking instances
    // walk around an explicitly defined convex hull. We can't easily create a
    // valid convex hull, so we create an obviously invalid one to force FCL to
    // search all vertices as a guarantee for correctness.
    auto fcl_convex = make_shared<fcl::Convexd>(
        shared_verts, 0, make_shared<std::vector<int>>());
    TakeShapeOwnership(fcl_convex, user_data);

    // TODO(DamrongGuoy):  Right now ProcessGeometriesForDeformableContact()
    //  will call deformable::Geometries::MaybeAddRigidGeometry(), which will
    //  add the geometry only when its proximity property has
    //  (kHydroGroup, kRezHint). We should make exception for Mesh since it
    //  doesn't need resolution hint.
    ProcessGeometriesForDeformableContact(mesh, user_data);
  }

  void ImplementGeometry(const Sphere& sphere, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_sphere = make_shared<fcl::Sphered>(sphere.radius());
    TakeShapeOwnership(fcl_sphere, user_data);
    ProcessHydroelastic(sphere, user_data);
    ProcessGeometriesForDeformableContact(sphere, user_data);
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
    shape_distance::CallbackData<T> data{nullptr, &X_WGs, max_distance,
                                         &witness_pairs};
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

    // If the callback didn't throw, it returned an actual value.
    DRAKE_DEMAND(witness_pairs.size() > 0);

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

  std::vector<PenetrationAsPointPair<T>> ComputePointPairPenetration(
      const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs) const {
    std::vector<PenetrationAsPointPair<T>> contacts;
    penetration_as_point_pair::CallbackData data{&collision_filter_, &X_WGs,
                                                 &contacts};

    // Perform a query of the dynamic objects against themselves.
    dynamic_tree_.collide(&data, penetration_as_point_pair::Callback<T>);

    // Perform a query of the dynamic objects against the anchored. We don't do
    // anchored against anchored because those pairs are implicitly filtered.
    FclCollide(dynamic_tree_, anchored_tree_, &data,
               penetration_as_point_pair::Callback<T>);

    std::sort(contacts.begin(), contacts.end(), OrderPointPair<T>);

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

    std::sort(
        pairs.begin(), pairs.end(),
        [](const SortedPair<GeometryId>& p1, const SortedPair<GeometryId>& p2) {
          if (p1.first() != p2.first()) return p1.first() < p2.first();
          return p1.second() < p2.second();
        });

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

  template <typename T1 = T>
  typename std::enable_if_t<scalar_predicate<T1>::is_bool,
                            std::vector<ContactSurface<T>>>
  ComputeContactSurfaces(
      HydroelasticContactRepresentation representation,
      const unordered_map<GeometryId, RigidTransform<T>>& X_WGs) const {
    vector<ContactSurface<T>> surfaces;
    // All these quantities are aliased in the callback data.
    hydroelastic::CallbackData<T> data{&collision_filter_, &X_WGs,
                                       &hydroelastic_geometries_,
                                       representation, &surfaces};

    // Perform a query of the dynamic objects against themselves.
    dynamic_tree_.collide(&data, hydroelastic::Callback<T>);

    // Perform a query of the dynamic objects against the anchored. We don't do
    // anchored against anchored because those pairs are implicitly filtered.
    FclCollide(dynamic_tree_, anchored_tree_, &data, hydroelastic::Callback<T>);

    std::sort(surfaces.begin(), surfaces.end(), OrderContactSurface<T>);

    return surfaces;
  }

  template <typename T1 = T>
  typename std::enable_if_t<scalar_predicate<T1>::is_bool, void>
  ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation representation,
      const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
      std::vector<ContactSurface<T>>* surfaces,
      std::vector<PenetrationAsPointPair<T>>* point_pairs) const {
    DRAKE_DEMAND(surfaces != nullptr);
    DRAKE_DEMAND(point_pairs != nullptr);

    // All these quantities are aliased in the callback data.
    hydroelastic::CallbackWithFallbackData<T> data{
        hydroelastic::CallbackData<T>{&collision_filter_, &X_WGs,
                                      &hydroelastic_geometries_, representation,
                                      surfaces},
        point_pairs};

    // Dynamic vs dynamic and dynamic vs anchored represent all the geometries
    // that we can support with the point-pair fallback. Do those first.
    dynamic_tree_.collide(&data, hydroelastic::CallbackWithFallback<T>);

    FclCollide(dynamic_tree_, anchored_tree_, &data,
               hydroelastic::CallbackWithFallback<T>);

    std::sort(surfaces->begin(), surfaces->end(), OrderContactSurface<T>);

    std::sort(point_pairs->begin(), point_pairs->end(), OrderPointPair<T>);
  }

  void ComputeDeformableContact(
      DeformableContact<double>* deformable_contact) const {
    *deformable_contact =
        geometries_for_deformable_contact_.ComputeDeformableContact();
  }

  // Testing utilities

  bool IsDeepCopy(const Impl& other) const {
    if (this != &other) {
      // TODO(DamrongGuoy): Consider checking other data members such as
      //  hydroelastic_geometries_.
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
      if (this->collision_filter_ != other.collision_filter_) return false;
      return true;
    }
    return false;
  }

  const RigidTransformd GetX_WG(GeometryId id, bool is_dynamic) const {
    const unordered_map<GeometryId, unique_ptr<CollisionObjectd>>& objects =
        is_dynamic ? dynamic_objects_ : anchored_objects_;

    return RigidTransformd(objects.at(id)->getTransform());
  }

  const hydroelastic::Geometries& hydroelastic_geometries() const {
    return hydroelastic_geometries_;
  }

  const deformable::Geometries& deformable_contact_geometries() const {
    return geometries_for_deformable_contact_;
  }

  bool IsFclConvexType(GeometryId id) const {
    auto iter = dynamic_objects_.find(id);
    if (iter == dynamic_objects_.end()) {
      iter = anchored_objects_.find(id);
      if (iter == anchored_objects_.end()) {
        throw std::logic_error(
            fmt::format("ProximityEngine::IsFclConvexType() cannot be "
                        "called for invalid geometry id {}.",
                        id));
      }
    }
    return iter->second->getNodeType() == fcl::GEOM_CONVEX;
  }

 private:
  // Engine on one scalar can see the members of other engines.
  friend class ProximityEngineTester;
  template <typename>
  friend class ProximityEngine;

  void AddGeometry(
      const Shape& shape, const RigidTransformd& X_WG, GeometryId id,
      const ProximityProperties& props, bool is_dynamic,
      fcl::DynamicAABBTreeCollisionManager<double>* tree,
      unordered_map<GeometryId, unique_ptr<CollisionObjectd>>* objects) {
    ReifyData data{nullptr, id, props, X_WG};
    shape.Reify(this, &data);

    data.fcl_object->setTransform(X_WG.GetAsIsometry3());
    data.fcl_object->computeAABB();
    EncodedData encoding(id, is_dynamic);
    encoding.write_to(data.fcl_object.get());

    tree->registerObject(data.fcl_object.get());
    tree->update();
    (*objects)[id] = std::move(data.fcl_object);

    collision_filter_.AddGeometry(id);
  }

  // Removes the geometry with the given id from the given tree.
  void RemoveGeometry(
      GeometryId id, fcl::DynamicAABBTreeCollisionManager<double>* tree,
      unordered_map<GeometryId, unique_ptr<CollisionObjectd>>* geometries) {
    unordered_map<GeometryId, unique_ptr<CollisionObjectd>>& typed_geometries =
        *geometries;
    CollisionObjectd* fcl_object = typed_geometries.at(id).get();
    const size_t old_size = tree->size();
    tree->unregisterObject(fcl_object);
    collision_filter_.RemoveGeometry(id);
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
  FclDynamicAABBTreeCollisionManager dynamic_tree_;

  // All of the *dynamic* collision elements (spanning all sources).
  MapGeometryIdToFclCollisionObject dynamic_objects_;

  // The tree containing all of the anchored geometry.
  FclDynamicAABBTreeCollisionManager anchored_tree_;

  // All of the *anchored* collision elements (spanning *all* sources).
  MapGeometryIdToFclCollisionObject anchored_objects_;

  // The mechanism for dictating collision filtering.
  CollisionFilter collision_filter_;

  // The tolerance that determines when the iterative process would terminate.
  // @see ProximityEngine::set_distance_tolerance() for more details.
  double distance_tolerance_{1E-6};

  // All of the hydroelastic representations of supported geometries -- this
  // can get quite large based on mesh resolution.
  hydroelastic::Geometries hydroelastic_geometries_;

  // All of the geometries that produce contacts that involve deformable
  // geometries. This includes deformable geometries as well as rigid geometry
  // representations that participate in contacts with deformable geometries.
  // The deformable geometries registered here are not included in
  // `dynamic_objects_` and `dynamic_tree_`.
  deformable::Geometries geometries_for_deformable_contact_;
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
void ProximityEngine<T>::AddDynamicGeometry(const Shape& shape,
                                            const RigidTransformd& X_WG,
                                            GeometryId id,
                                            const ProximityProperties& props) {
  impl_->AddDynamicGeometry(shape, X_WG, id, props);
}

template <typename T>
void ProximityEngine<T>::AddAnchoredGeometry(
    const Shape& shape, const RigidTransformd& X_WG, GeometryId id,
    const ProximityProperties& props) {
  impl_->AddAnchoredGeometry(shape, X_WG, id, props);
}

template <typename T>
void ProximityEngine<T>::AddDeformableGeometry(const VolumeMesh<double>& mesh,
                                               GeometryId id) {
  impl_->AddDeformableGeometry(mesh, id);
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
void ProximityEngine<T>::RemoveDeformableGeometry(GeometryId id) {
  impl_->RemoveDeformableGeometry(id);
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
template <typename U>
std::unique_ptr<ProximityEngine<U>> ProximityEngine<T>::ToScalarType() const {
  return std::unique_ptr<ProximityEngine<U>>(
      new ProximityEngine<U>(impl_->template ToScalarType<U>().release()));
}

template <typename T>
CollisionFilter& ProximityEngine<T>::collision_filter() {
  return impl_->collision_filter();
}

template <typename T>
void ProximityEngine<T>::UpdateWorldPoses(
    const unordered_map<GeometryId, RigidTransform<T>>& X_WGs) {
  impl_->UpdateWorldPoses(X_WGs);
}

template <typename T>
void ProximityEngine<T>::UpdateDeformableVertexPositions(
    const std::unordered_map<GeometryId, VectorX<T>>& q_WGs) {
  impl_->UpdateDeformableVertexPositions(q_WGs);
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
std::vector<PenetrationAsPointPair<T>>
ProximityEngine<T>::ComputePointPairPenetration(
    const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs)
    const {
  return impl_->ComputePointPairPenetration(X_WGs);
}

template <typename T>
template <typename T1>
typename std::enable_if_t<scalar_predicate<T1>::is_bool,
                          std::vector<ContactSurface<T>>>
ProximityEngine<T>::ComputeContactSurfaces(
    HydroelasticContactRepresentation representation,
    const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs) const {
  return impl_->ComputeContactSurfaces(representation, X_WGs);
}

template <typename T>
template <typename T1>
typename std::enable_if_t<scalar_predicate<T1>::is_bool, void>
ProximityEngine<T>::ComputeContactSurfacesWithFallback(
    HydroelasticContactRepresentation representation,
    const std::unordered_map<GeometryId, RigidTransform<T>>& X_WGs,
    std::vector<ContactSurface<T>>* surfaces,
    std::vector<PenetrationAsPointPair<T>>* point_pairs) const {
  return impl_->ComputeContactSurfacesWithFallback(representation, X_WGs,
                                                   surfaces, point_pairs);
}

template <typename T>
template <typename T1>
typename std::enable_if_t<std::is_same_v<T1, double>, void>
ProximityEngine<T>::ComputeDeformableContact(
    DeformableContact<T>* deformable_contact) const {
  impl_->ComputeDeformableContact(deformable_contact);
}

template <typename T>
std::vector<SortedPair<GeometryId>>
ProximityEngine<T>::FindCollisionCandidates() const {
  return impl_->FindCollisionCandidates();
}

// Testing utilities

template <typename T>
bool ProximityEngine<T>::IsDeepCopy(const ProximityEngine<T>& other) const {
  return impl_->IsDeepCopy(*other.impl_);
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

template <typename T>
const deformable::Geometries&
ProximityEngine<T>::deformable_contact_geometries() const {
  return impl_->deformable_contact_geometries();
}

template <typename T>
bool ProximityEngine<T>::IsFclConvexType(GeometryId id) const {
  return impl_->IsFclConvexType(id);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&ProximityEngine<T>::template ToScalarType<U>))

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&ProximityEngine<T>::template ComputeContactSurfaces<T>,
     &ProximityEngine<T>::template ComputeContactSurfacesWithFallback<T>))

template void ProximityEngine<double>::ComputeDeformableContact<double>(
    DeformableContact<double>*) const;

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::geometry::internal::ProximityEngine)
