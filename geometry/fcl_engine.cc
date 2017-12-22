#include "drake/geometry/fcl_engine.h"

#include <unordered_map>
#include <utility>

#include <fcl/fcl.h>

#include "drake/common/default_scalars.h"

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

// ADL-reliant helper functions for converting Isometry<T> to Isometry<double>.
const Isometry3<double>& convert(const Isometry3<double>& transform) {
  return transform;
}

Isometry3<double> convert(const Isometry3<AutoDiffXd>& transform) {
  Isometry3<double> result;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      result.matrix()(r, c) = ExtractDoubleOrThrow(transform.matrix()(r, c));
    }
  }
  return result;
}

// Utilities/functions for working with the encoding of collision object index
// and mobility type in the fcl::CollisionObject user data field.
class EncodedData {
 public:
  EncodedData(int index, bool is_dynamic)
      : data_(static_cast<size_t>(index)) {
    if (is_dynamic) set_dynamic();
    // NOTE: data is encoded as anchored by default. So, an action only needs to
    // be taken in the dynamic case.
  }

  explicit EncodedData(const fcl::CollisionObject<double>& fcl_object) :
    data_(reinterpret_cast<size_t>(fcl_object.getUserData())) {}

  // Sets the encoded data to be dynamic.
  void set_dynamic() { data_ |= kIsDynamicMask; }

  // Sets the encoded data to be anchored.
  void set_anchored() { data_ &= ~kIsDynamicMask; }

  // Stores the encoded data in the collision object's user data.
  void store(fcl::CollisionObject<double>* object) {
    object->setUserData(reinterpret_cast<void*>(data_));
  }

  // Reports true if the given fcl_object's user data is encoded as dynamic.
  // False if anchored.
  bool is_dynamic() const {
    return (data_ & kIsDynamicMask) > 0;
  }

  // Reports the stored index.
  size_t index() const { return data_ & ~kIsDynamicMask; }

  // Given an fcl object and maps from index to id of both dynamic and anchored
  // geometry, returns the geometry id for the given fcl object.
  GeometryId id(const std::vector<GeometryId>& dynamic_map,
                const std::vector<GeometryId>& anchored_map) const {
    const size_t i = index();
    return is_dynamic() ? dynamic_map[i] : anchored_map[i];
  }

 private:
  // Fcl Collision objects allow for user data. We're storing *two* pieces of
  // information: the engine index of the corresponding geometry and the
  // _mobility_ type (i.e., dynamic vs anchored). We do this my mangling bits.
  // Because userdata is a void*, we can read it like a size_t. The highest
  // bit will represent dynamic (1) or anchored (0). The remaining lower order
  // bits store the index. These values and functions facilitate manipulation of
  // this encoded collision data.
  static_assert(sizeof(size_t) == 8,
                "Wrong size for size_t, the dynamic bit mask will fail");
  static const size_t kIsDynamicMask = size_t{1} << 63;

  // The encoded data - index and mobility type.
  size_t data_{};
};

// Returns a copy of the given fcl collision geometry; throws an exception for
// unsupported collision geometry types. This supplements the *missing* cloning
// functionality in FCL. Issue has been submitted to FCL:
// https://github.com/flexible-collision-library/fcl/issues/246
shared_ptr<fcl::ShapeBased> CopyShapeOrThrow(
    const fcl::CollisionGeometryd& geometry) {
  // NOTE: Returns a shared pointer because of the FCL API in assigning
  // collision geometry to collision objects.
  switch (geometry.getNodeType()) {
    case fcl::GEOM_SPHERE:
    {
      const fcl::Sphered& sphere = dynamic_cast<const fcl::Sphered&>(geometry);
      return make_shared<fcl::Sphered>(sphere.radius);
    }
    case fcl::GEOM_CYLINDER:
    {
      const fcl::Cylinderd& cylinder =
          dynamic_cast<const fcl::Cylinderd&>(geometry);
      return make_shared<fcl::Cylinderd>(cylinder.radius, cylinder.lz);
    }
    case fcl::GEOM_HALFSPACE:
      // All half spaces are defined exactly the same.
      return make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    case fcl::GEOM_BOX:
    case fcl::GEOM_ELLIPSOID:
    case fcl::GEOM_CAPSULE:
    case fcl::GEOM_CONE:
    case fcl::GEOM_CONVEX:
    case fcl::GEOM_PLANE:
    case fcl::GEOM_TRIANGLE:
      throw std::runtime_error(
          "Trying to copy fcl::CollisionGeometry of unsupported GEOM_* type");
    default:
      throw std::runtime_error(
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
// FclEngine::operator=()).
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

template <typename T>
FclEngine<T>::FclEngine(const FclEngine<T>& other) {
  *this = other;
}

template <typename T>
FclEngine<T>& FclEngine<T>::operator=(const FclEngine<T>& other) {
  last_implemented_.reset();
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
  CopyFclObjectsOrThrow(other.dynamic_objects_, &dynamic_objects_, &object_map);

  // Build new AABB trees from the input AABB trees.
  BuildTreeFromReference(other.dynamic_tree_, object_map, &dynamic_tree_);
  BuildTreeFromReference(other.anchored_tree_, object_map, &anchored_tree_);

  return *this;
}

template <typename T>
FclEngine<T>::FclEngine(FclEngine<T>&& other) noexcept {
  *this = other;
}

template <typename T>
FclEngine<T>& FclEngine<T>::operator=(FclEngine<T>&& other) noexcept {
  // TODO(SeanCurtis-TRI): Provide this *proper* move semantics rather than
  // simply copying the other. This is a short-term contrivance to allow the
  // code to move forward.
  return *this = other;
}

template<typename T>
GeometryIndex FclEngine<T>::AddDynamicGeometry(const Shape& shape) {
  shape.Reify(this);
  fcl::CollisionObjectd* fcl_object = last_implemented_.get();
  dynamic_tree_.registerObject(fcl_object);
  GeometryIndex index(static_cast<int>(dynamic_objects_.size()));
  EncodedData(index, true /* is dynamic */).store(fcl_object);
  dynamic_objects_.push_back(std::move(last_implemented_));

  return index;
}

template <typename T>
AnchoredGeometryIndex FclEngine<T>::AddAnchoredGeometry(
    const Shape& shape, const Isometry3<double>& X_WG) {
  shape.Reify(this);
  fcl::CollisionObjectd* fcl_object = last_implemented_.get();
  fcl_object->setTransform(X_WG);
  anchored_tree_.registerObject(fcl_object);
  AnchoredGeometryIndex index(static_cast<int>(anchored_objects_.size()));
  EncodedData(index, false /* is dynamic */).store(fcl_object);
  anchored_objects_.push_back(std::move(last_implemented_));

  return index;
}

template <typename T>
void FclEngine<T>::ImplementGeometry(const Sphere& sphere) {
  // Note: Using `shared_ptr` because of FCL API requirements.
  auto fcl_sphere = make_shared<fcl::Sphered>(sphere.get_radius());
  TakeShapeOwnership(fcl_sphere);
}

template <typename T>
void FclEngine<T>::ImplementGeometry(const Cylinder& cylinder) {
  // Note: Using `shared_ptr` because of FCL API requirements.
  auto fcl_cylinder =
      make_shared<fcl::Cylinderd>(cylinder.get_radius(), cylinder.get_length());
  TakeShapeOwnership(fcl_cylinder);
}

template <typename T>
void FclEngine<T>::ImplementGeometry(const HalfSpace&) {
  // Note: Using `shared_ptr` because of FCL API requirements.
  auto fcl_half_space = make_shared<fcl::Halfspaced>(0, 0, 1, 0);
  TakeShapeOwnership(fcl_half_space);
}

template <typename T>
void FclEngine<T>::TakeShapeOwnership(
    const shared_ptr<fcl::ShapeBased>& shape) {
  DRAKE_DEMAND(last_implemented_ == nullptr);
  last_implemented_ = make_unique<fcl::CollisionObjectd>(shape);
}

template <typename T>
std::unique_ptr<FclEngine<AutoDiffXd>> FclEngine<T>::ToAutoDiff() const {
  auto engine = make_unique<FclEngine<AutoDiffXd>>();

  // Copy all of the geometry to ensure that the engine index values stay
  // aligned.
  std::unordered_map<const fcl::CollisionObjectd*, fcl::CollisionObjectd*>
      object_map;
  CopyFclObjectsOrThrow(anchored_objects_, &engine->anchored_objects_,
                        &object_map);
  CopyFclObjectsOrThrow(dynamic_objects_, &engine->dynamic_objects_,
                        &object_map);

  // Build new AABB trees from the input AABB trees.
  BuildTreeFromReference(dynamic_tree_, object_map, &engine->dynamic_tree_);
  BuildTreeFromReference(anchored_tree_, object_map, &engine->anchored_tree_);

  return engine;
}

template <typename T>
void FclEngine<T>::UpdateWorldPoses(
    const std::vector<Isometry3<T>>& X_WG) {
  DRAKE_DEMAND(X_WG.size() == dynamic_objects_.size());
  for (size_t i = 0; i < X_WG.size(); ++i) {
    dynamic_objects_[i]->setTransform(convert(X_WG[i]));
  }
  dynamic_tree_.update();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::FclEngine)
