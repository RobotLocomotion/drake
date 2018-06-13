#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <unordered_map>
#include <utility>

#include <fcl/fcl.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/collision_request.h>

#include "drake/common/default_scalars.h"
#include "drake/common/sorted_vectors_have_intersection.h"

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

// TODO(SeanCurtis-TRI): Swap all Isometry3 for Transforms.

// ADL-reliant helper functions for converting Isometry<T> to Isometry<double>.
const Isometry3<double>& convert(const Isometry3<double>& transform) {
  return transform;
}

template <class VectorType>
Isometry3<double> convert(
    const Isometry3<Eigen::AutoDiffScalar<VectorType>>& transform) {
  Isometry3<double> result;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      result.matrix()(r, c) = ExtractDoubleOrThrow(transform.matrix()(r, c));
    }
  }
  return result;
}

// Utilities/functions for working with the encoding of collision object index
// and mobility type in the fcl::CollisionObject user data field. The encoded
// data produces a value that is guaranteed to be unique across all geometries.
// The guarantee arises from the fact that it's an engine index combined with
// a bit reporting anchored vs dynamic geometry; every geometry is uniquely
// identified by its engine index and its dynamic/anchored state.
//
// The encoding packs a geometry index value with a bit indicating if the index
// refers to dynamic or anchored geometry (they are stored in separate vectors
// which means indices can be repeated). The highest-order bit indicates
// dynamic (1) or anchored (0). The remaining lower bits store the index. The
// data is stored in a pointer-sized integer. This integer is, in turn, stored
// directly into fcl::CollisionObject's void* user data member.
class EncodedData {
 public:
  // Constructor which defines its encoded data by combining a geometry's index
  // and its dynamic/anchored characterization.
  EncodedData(int index, bool is_dynamic)
      : data_(static_cast<uintptr_t>(index)) {
    if (is_dynamic) set_dynamic();
    // NOTE: data is encoded as anchored by default. So, an action only needs to
    // be taken in the dynamic case.
  }

  // Constructor which defines its encoded data value by extracting it from the
  // given Fcl collision object.
  explicit EncodedData(const fcl::CollisionObject<double>& fcl_object)
      : data_(reinterpret_cast<uintptr_t>(fcl_object.getUserData())) {}

  // Factory method for creating EncodedData from the given `index`. By
  // definition, the GeometryIndex only applies to dynamic geometry.
  static EncodedData encode_dynamic(GeometryIndex index) {
    return EncodedData(index, true);
  }

  // Factory method for creating EncodedData from the given `index`. By
  // definition, the AchoredGeometryIndex only applies to anchored geometry.
  static EncodedData encode_anchored(AnchoredGeometryIndex index) {
    return EncodedData(index, false);
  }

  // Sets the encoded data to be dynamic.
  void set_dynamic() { data_ |= kIsDynamicMask; }

  // Sets the encoded data to be anchored.
  void set_anchored() { data_ &= ~kIsDynamicMask; }

  // Stores the encoded data in the collision object's user data.
  void store_in(fcl::CollisionObject<double>* object) {
    object->setUserData(reinterpret_cast<void*>(data_));
  }

  // Reports true if the given fcl_object's user data is encoded as dynamic.
  // False if anchored.
  bool is_dynamic() const { return (data_ & kIsDynamicMask) != 0; }

  // Reports the stored index.
  int index() const { return static_cast<int>(data_ & ~kIsDynamicMask); }

  // Given an fcl object and maps from index to id of both dynamic and anchored
  // geometry, returns the geometry id for the given fcl object.
  GeometryId id(const std::vector<GeometryId>& dynamic_map,
                const std::vector<GeometryId>& anchored_map) const {
    const uintptr_t i = index();
    return is_dynamic() ? dynamic_map[i] : anchored_map[i];
  }

  // Reports the encoded data.
  uintptr_t encoded_data() const { return data_; }

  // Fcl Collision objects allow for user data. We're storing *two* pieces of
  // information: the engine index of the corresponding geometry and the
  // _mobility_ type (i.e., dynamic vs anchored). We do this by mangling bits.
  // Because user data is a void*, we can read it like a uintptr_t. The highest
  // bit will represent dynamic (1) or anchored (0).The remaining lower order
  // bits store the index.

  // Note: This sets a mask to be 1000...0 based on the size of a pointer.
  // C++ guarantees that uintptr_t can hold a pointer and cast to a pointer,
  // but it doesn't guarantee it's the *same size* as a pointer. So, we set
  // the mask value based on pointer size.
  static const uintptr_t kIsDynamicMask = uintptr_t{1}
                                          << (sizeof(void*) * 8 - 1);

 private:
  // The encoded data - index and mobility type.
  // We're using an unsigned value here because:
  //   - Bitmasking games are typically more intuitive with unsigned values
  //     (think of bit shifting as an example here).
  //   - This unsigned integer doesn't bleed out into the API at all.
  uintptr_t data_{};
};

// A simple class for providing collision filtering functionality similar to
// that found in RigidBodyTree but made compatible with fcl. The majority of
// this code is lifted verbatim from drake/multibody/collision/element.{h, cc}.
//
// Note: I'm using uintptr_t instead of EncodedData directly to avoid having
// to hash EncodedData.
// TODO(SeanCurtis-TRI): Replace this "legacy" mechanism with the
// new-and-improved alternative when it is ready.
class CollisionFilterLegacy {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterLegacy)

  CollisionFilterLegacy() = default;

  void AddGeometry(uintptr_t id) {
    collision_cliques_.insert({id, std::vector<int>()});
  }

  // NOTE: This assumes that `id_A` and `id_B` will *never* both be anchored
  // geometries. The structure of the collision query logic precludes that
  // possibility; dynamic is collided against dynamic and dynamic is collided
  // against anchored, but anchored is never collided against anchored.
  bool CanCollideWith(uintptr_t id_A, uintptr_t id_B) const {
    // These ids should all be registered with the filter machinery.
    DRAKE_ASSERT(collision_cliques_.count(id_A) == 1);
    DRAKE_ASSERT(collision_cliques_.count(id_B) == 1);

    bool excluded = id_A == id_B ||
                    SortedVectorsHaveIntersection(collision_cliques_.at(id_A),
                                                  collision_cliques_.at(id_B));

    return !excluded;
  }

  void AddToCollisionClique(uintptr_t geometry_id, int clique_id) {
    DRAKE_ASSERT(collision_cliques_.count(geometry_id) == 1);

    std::vector<int>& cliques = collision_cliques_[geometry_id];
    // Order(N) insertion.
    // `cliques` is a sorted vector so that checking if two collision elements
    // belong to a common group can be performed efficiently in order N.
    // See Element::CanCollideWith() and Element::collision_cliques_ for
    // explanation.
    auto it = std::lower_bound(cliques.begin(), cliques.end(), clique_id);

    // This test prevents duplicate clique ids from being added.
    if (it == cliques.end() || clique_id < *it)
      cliques.insert(it, clique_id);
  }

  int num_cliques(uintptr_t geometry_id) const {
    DRAKE_ASSERT(collision_cliques_.count(geometry_id) == 1);
    return static_cast<int>(collision_cliques_.at(geometry_id).size());
  }

  // This method is not thread safe.
  int next_clique_id() { return next_available_clique_++; }

  // Test support; to detect when cliques are generated.
  int peek_next_clique() const { return next_available_clique_; }

 private:
  // A map between the EncodedData::encoded_data() value for a geometry and
  // its set of cliques.
  std::unordered_map<uintptr_t, std::vector<int>> collision_cliques_;
  int next_available_clique_{0};
};

// Struct for use in SingleCollisionCallback(). Contains the collision request
// and accumulates results in a drake::multibody::collision::PointPair vector.
struct CollisionData {
  CollisionData(const std::vector<GeometryId>* dynamic_map_in,
                const std::vector<GeometryId>* anchored_map_in,
                const CollisionFilterLegacy* collision_filter_in)
      : dynamic_map(*dynamic_map_in),
        anchored_map(*anchored_map_in),
        collision_filter(*collision_filter_in) {}
  // Maps so the penetration call back can map from engine index to geometry id.
  const std::vector<GeometryId>& dynamic_map;
  const std::vector<GeometryId>& anchored_map;
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
      encoding_A.encoded_data(), encoding_B.encoded_data());

  if (can_collide) {
    // Unpack the callback data
    const fcl::CollisionRequestd& request = collision_data.request;

    // This callback only works for a single contact, this confirms a request
    // hasn't been made for more contacts.
    DRAKE_ASSERT(request.num_max_contacts == 1);
    fcl::CollisionResultd result;

    // Perform nearphase collision detection
    fcl::collide(&fcl_object_A, &fcl_object_B, request, result);

    // Process the contact points
    if (result.isCollision()) {
      // NOTE: This assumes that the request is configured to use a single
      // contact.
      const fcl::Contactd& contact = result.getContact(0);
      //  By convention, Drake requires the contact normal to point out of B and
      //  into A. FCL uses the opposite convention.
      Vector3d drake_normal = -contact.normal;

      // Signed distance is negative when penetration depth is positive.
      double depth = contact.penetration_depth;

      // FCL returns a single contact point centered between the two penetrating
      // surfaces. PenetrationAsPointPair expects
      // two, one on the surface of body A (Ac) and one on the surface of body B
      // (Bc). Choose points along the line defined by the contact point and
      // normal, equidistant to the contact point. Recall that signed_distance
      // is strictly non-positive, so signed_distance * drake_normal points out
      // of A and into B.
      Vector3d p_WAc = contact.pos - 0.5 * depth * drake_normal;
      Vector3d p_WBc = contact.pos + 0.5 * depth * drake_normal;

      PenetrationAsPointPair<double> penetration;
      penetration.depth = depth;
      // The engine doesn't know geometry ids; it returns engine indices. The
      // caller must map engine indices to geometry ids.
      const std::vector<GeometryId>& dynamic_map = collision_data.dynamic_map;
      const std::vector<GeometryId>& anchored_map = collision_data.anchored_map;
      penetration.id_A = encoding_A.id(dynamic_map, anchored_map);
      penetration.id_B = encoding_B.id(dynamic_map, anchored_map);
      penetration.p_WCa = p_WAc;
      penetration.p_WCb = p_WBc;
      penetration.nhat_BA_W = drake_normal;
      collision_data.contacts->emplace_back(std::move(penetration));
    }
  }

  // Returning true would tell the broadphase manager to terminate early. Since
  // we want to find all the collisions present in the model's current
  // configuration, we return false.
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
    case fcl::GEOM_ELLIPSOID:
    case fcl::GEOM_CAPSULE:
    case fcl::GEOM_CONE:
    case fcl::GEOM_CONVEX:
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
}

}  // namespace

// The implementation class for the fcl engine. Each of these functions
// mirrors a method on the ProximityEngine (unless otherwise indicated.
// See ProximityEngine for documentation.
template <typename T>
class ProximityEngine<T>::Impl : public ShapeReifier {
 public:
  Impl() = default;

  Impl(const Impl& other) {
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

  GeometryIndex AddDynamicGeometry(const Shape& shape) {
    // The collision object gets instantiated in the reification process and
    // placed in this unique pointer.
    std::unique_ptr<fcl::CollisionObject<double>> fcl_object;
    shape.Reify(this, &fcl_object);
    dynamic_tree_.registerObject(fcl_object.get());
    GeometryIndex index(static_cast<int>(dynamic_objects_.size()));
    EncodedData encoding(index, true /* is dynamic */);
    encoding.store_in(fcl_object.get());
    dynamic_objects_.emplace_back(std::move(fcl_object));
    collision_filter_.AddGeometry(encoding.encoded_data());
    return index;
  }

  AnchoredGeometryIndex AddAnchoredGeometry(const Shape& shape,
                                            const Isometry3<double>& X_WG) {
    // The collision object gets instantiated in the reification process and
    // placed in this unique pointer.
    std::unique_ptr<fcl::CollisionObject<double>> fcl_object;
    shape.Reify(this, &fcl_object);
    fcl_object->setTransform(X_WG);
    anchored_tree_.registerObject(fcl_object.get());
    AnchoredGeometryIndex index(static_cast<int>(anchored_objects_.size()));
    EncodedData encoding(index, false /* is dynamic */);
    encoding.store_in(fcl_object.get());
    anchored_objects_.emplace_back(std::move(fcl_object));
    collision_filter_.AddGeometry(encoding.encoded_data());

    return index;
  }

  int num_geometries() const {
    return static_cast<int>(dynamic_objects_.size() + anchored_objects_.size());
  }

  int num_dynamic() const { return static_cast<int>(dynamic_objects_.size()); }

  int num_anchored() const {
    return static_cast<int>(anchored_objects_.size());
  }

  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG) {
    DRAKE_DEMAND(X_WG.size() == dynamic_objects_.size());
    for (size_t i = 0; i < X_WG.size(); ++i) {
      dynamic_objects_[i]->setTransform(convert(X_WG[i]));
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

  void ImplementGeometry(const HalfSpace&,
                         void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_half_space = make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    TakeShapeOwnership(fcl_half_space, user_data);
  }

  void ImplementGeometry(const Box& box, void* user_data) override {
    auto fcl_box = make_shared<fcl::Boxd>(box.size());
    TakeShapeOwnership(fcl_box, user_data);
  }

  void ImplementGeometry(const Mesh&, void* user_data) override {
    // TODO(SeanCurtis-TRI): Replace this with a legitimate fcl mesh. This
    // assumes that a zero-radius sphere has no interesting interactions with
    // other meshes. However, it *does* increase the collision space. :(
    auto fcl_sphere = make_shared<fcl::Sphered>(0.0);
    TakeShapeOwnership(fcl_sphere, user_data);
  }

  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration(
      const std::vector<GeometryId>& dynamic_map,
      const std::vector<GeometryId>& anchored_map) const {
    std::vector<PenetrationAsPointPair<double>> contacts;
    // CollisionData stores references to the provided data structures.
    CollisionData collision_data{&dynamic_map, &anchored_map,
                                 &collision_filter_};
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
      const std::unordered_set<AnchoredGeometryIndex>& anchored) {
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
        collision_filter_.AddToCollisionClique(encoding.encoded_data(), clique);
      }
      for (auto index : anchored) {
        EncodedData encoding(index, false /* is dynamic */);
        collision_filter_.AddToCollisionClique(encoding.encoded_data(), clique);
      }
    }
  }

  void ExcludeCollisionsBetween(
      const std::unordered_set<GeometryIndex>& dynamic1,
      const std::unordered_set<AnchoredGeometryIndex>& anchored1,
      const std::unordered_set<GeometryIndex>& dynamic2,
      const std::unordered_set<AnchoredGeometryIndex>& anchored2) {
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
              collision_filter_.CanCollideWith(encoding1.encoded_data(),
                                               encoding2.encoded_data())) {
            int clique = collision_filter_.next_clique_id();
            collision_filter_.AddToCollisionClique(encoding2.encoded_data(),
                                                   clique);
            collision_filter_.AddToCollisionClique(encoding1.encoded_data(),
                                                   clique);
          }
        }
      }
    }
  }

  int get_next_clique() {
    return collision_filter_.next_clique_id();
  }

  void set_clique(GeometryIndex index, int clique) {
    EncodedData encoding(index, true /* is dynamic */);
    collision_filter_.AddToCollisionClique(encoding.encoded_data(), clique);
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
        return test.getUserData() == ref.getUserData() &&
            &test != &ref &&
            test.getNodeType() == ref.getNodeType() &&
            test.getObjectType() == ref.getObjectType() &&
            test.getAABB().center() == ref.getAABB().center() &&
            test.getAABB().width(), ref.getAABB().width() &&
            test.getAABB().height(), ref.getAABB().height() &&
            test.getAABB().depth(), ref.getAABB().depth();
      };
      bool is_copy = true;
      is_copy = is_copy &&
          this->dynamic_objects_.size() == other.dynamic_objects_.size();
      is_copy = is_copy &&
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

  int peek_next_clique() const {
    return collision_filter_.peek_next_clique();
  }

 private:
  // Engine on one scalar can see the members of other engines.
  friend class ProximityEngineTester;
  template <typename> friend class ProximityEngine;

  // TODO(SeanCurtis-TRI): Convert these to scalar type T when I know how to
  // transmogrify them. Otherwise, while the engine can be transmogrified, the
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
};

template <typename T>
ProximityEngine<T>::ProximityEngine() : impl_(new Impl()) {}

template <typename T>
ProximityEngine<T>::~ProximityEngine() { delete impl_; }

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
GeometryIndex ProximityEngine<T>::AddDynamicGeometry(const Shape& shape) {
  return impl_->AddDynamicGeometry(shape);
}

template <typename T>
AnchoredGeometryIndex ProximityEngine<T>::AddAnchoredGeometry(
    const Shape& shape, const Isometry3<double>& X_WG) {
  return impl_->AddAnchoredGeometry(shape, X_WG);
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
std::unique_ptr<ProximityEngine<AutoDiffXd>> ProximityEngine<T>::ToAutoDiffXd()
    const {
  return unique_ptr<ProximityEngine<AutoDiffXd>>(
      new ProximityEngine<AutoDiffXd>(impl_->ToAutoDiff().release()));
}

template <typename T>
void ProximityEngine<T>::UpdateWorldPoses(
    const std::vector<Isometry3<T>>& X_WG) {
  impl_->UpdateWorldPoses(X_WG);
}

template <typename T>
std::vector<PenetrationAsPointPair<double>>
ProximityEngine<T>::ComputePointPairPenetration(
    const std::vector<GeometryId>& dynamic_map,
    const std::vector<GeometryId>& anchored_map) const {
  return impl_->ComputePointPairPenetration(dynamic_map, anchored_map);
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsWithin(
    const std::unordered_set<GeometryIndex>& dynamic,
    const std::unordered_set<AnchoredGeometryIndex>& anchored) {
  impl_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsBetween(
    const std::unordered_set<GeometryIndex>& dynamic1,
    const std::unordered_set<AnchoredGeometryIndex>& anchored1,
    const std::unordered_set<GeometryIndex>& dynamic2,
    const std::unordered_set<AnchoredGeometryIndex>& anchored2) {
  impl_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2, anchored2);
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::ProximityEngine)
