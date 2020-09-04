#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/color_palette.h"

namespace drake {
namespace geometry {
namespace internal {

/* A dummy implementation of the RenderEngine to facilitate testing. Its
 purpose is to facilitate testing at the RenderEngine interface and _not_ the
 details of any particular implementation. To that end, it has the following
 features:

 1. It makes the RenderLabel-Color conversion methods of the RenderEngine
    public to this renderer (so test infrastructure can invoke the conversion).
 2. It conditionally registers geometry based on an "accepting" set of
    properties and remembers the ids of geometries successfully registered.
 3. By remembering successful registration, it can test geometry removal (in
    that this implementation will only report removal of a geometry id that it
    knows to be registered).
 4. Records which poses have been updated via UpdatePoses() to validate which
    ids values are updated and which aren't (and with what pose).
 5. Records the camera pose provided to UpdateViewpoint() and report it with
    last_updated_X_WC().  */
class DummyRenderEngine : public render::RenderEngine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyRenderEngine);

  /* Constructor to exercise the default constructor of RenderEngine.  */
  DummyRenderEngine()
      : color_props_{-1, -1, -1.0, ""},
        depth_props_{-1, -1, -1.0, "", -1, -1},
        label_props_{-1, -1, -1.0, ""} {}

  /* Constructor for configuring the underlying RenderEngine.  */
  explicit DummyRenderEngine(const render::RenderLabel& label)
      : render::RenderEngine(label),
        color_props_{-1, -1, -1.0, ""},
        depth_props_{-1, -1, -1.0, "", -1, -1},
        label_props_{-1, -1, -1.0, ""} {}

  /* @group No-op implementation of RenderEngine interface.  */
  //@{
  void UpdateViewpoint(const math::RigidTransformd& X_WC) override {
    X_WC_ = X_WC;
  }
  using render::RenderEngine::RenderColorImage;
  using render::RenderEngine::RenderDepthImage;
  using render::RenderEngine::RenderLabelImage;

  void RenderColorImage(const render::CameraProperties& camera, bool,
                        systems::sensors::ImageRgba8U*) const override {
    ++simple_color_count_;
    color_props_ = camera;
  }
  void RenderDepthImage(const render::DepthCameraProperties& camera,
                        systems::sensors::ImageDepth32F*) const override {
    ++simple_depth_count_;
    depth_props_ = camera;
  }
  void RenderLabelImage(const render::CameraProperties& camera, bool,
                        systems::sensors::ImageLabel16I*) const override {
    ++simple_label_count_;
    label_props_ = camera;
  }

  using RenderEngine::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* user_data) override {}
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override {}
  void ImplementGeometry(const HalfSpace& half_space,
                         void* user_data) override {}
  void ImplementGeometry(const Box& box, void* user_data) override {}
  void ImplementGeometry(const Capsule& capsule, void* user_data) override {}
  void ImplementGeometry(const Mesh& mesh, void* user_data) override {}
  void ImplementGeometry(const Convex& convex, void* user_data) override {}
  //@}

  /* @name  Functions for supporting tests.  */
  //@{

  /* Creates a set of perception properties that will cause this render engine
   to _accept_ geometry during registration.  */
  PerceptionProperties accepting_properties() const {
    PerceptionProperties properties;
    properties.AddProperty(include_group_name_, "ignored", 0);
    return properties;
  }

  /* Creates a set of perception properties that will cause this render engine
   to _reject_ geometry during registration.  */
  PerceptionProperties rejecting_properties() const {
    return PerceptionProperties();
  }

  /* Initializes the set data to the freshly-constructed values. This
   leaves the registered data intact.  */
  void init_test_data() {
    updated_ids_.clear();
  }

  /* If true, this render engine will accept all registered geometry.  */
  void set_force_accept(bool force_accept) {
    force_accept_ = force_accept;
  }

  /* Reports the number of geometries that have been _accepted_ in
   registration.  */
  int num_registered() const {
    return static_cast<int>(registered_geometries_.size());
  }

  /* Reports `true` if the given id is registered with `this` engine.  */
  bool is_registered(GeometryId id) const {
    return registered_geometries_.count(id) > 0;
  }

  // These six functions (and supporting members) facilitate testing while there
  // are two APIs for specifying the camera for rendering images. They should
  // go away when the legacy "simple" intrinsics are removed.

  // Reports the number of times the Render[]Image API was called using the
  // simple camera specification.
  int num_simple_color_renders() const { return simple_color_count_;  }
  int num_simple_depth_renders() const { return simple_depth_count_;  }
  int num_simple_label_renders() const { return simple_label_count_;  }

  const render::CameraProperties& last_color_camera_properties() const {
    return color_props_;
  }
  const render::DepthCameraProperties& last_depth_camera_properties() const {
    return depth_props_;
  }
  const render::CameraProperties& last_label_camera_properties() const {
    return label_props_;
  }

  /* Returns the ids that have been updated via a call to UpdatePoses() and
   the poses that were set.  */
  const std::map<GeometryId, math::RigidTransformd>& updated_ids() const {
    return updated_ids_;
  }

  /* Returns the most recent pose of the geometry with the given `id` in the
   world frame.  */
  const math::RigidTransformd& world_pose(GeometryId id) const {
    return X_WGs_.at(id);
  }

  const math::RigidTransformd& last_updated_X_WC() const { return X_WC_; }

  // Promote these to be public to facilitate testing.
  using RenderEngine::LabelFromColor;
  using RenderEngine::GetColorDFromLabel;
  using RenderEngine::GetColorIFromLabel;

 protected:
  /* Dummy implementation that registers the given `shape` if the `properties`
   contains the "in_test" group or the render engine has been forced to accept
   all geometries (via set_force_accept()). (Also counts the number of
   successfully registered shape over the lifespan of `this` instance.)  */
  bool DoRegisterVisual(GeometryId id, const Shape&,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) override {
    X_WGs_[id] = X_WG;
    GetRenderLabelOrThrow(properties);
    if (force_accept_ || properties.HasGroup(include_group_name_)) {
      registered_geometries_.insert(id);
      return true;
    }
    return false;
  }

  /* Updates the pose X_WG for the geometry with the given `id`. Also tracks
   which ids have been updated and the poses set (over the _lifespan_ of
   `this` instance). This can be reset with a call to init_test_data().  */
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) override {
    updated_ids_[id] = X_WG;
    X_WGs_[id] = X_WG;
  }

  /* Removes the given geometry id (if it is registered).  */
  bool DoRemoveGeometry(GeometryId id) override {
    return registered_geometries_.erase(id) > 0;
  }

  /* Implementation of RenderEngine::DoClone().  */
  std::unique_ptr<render::RenderEngine> DoClone() const override {
    return std::make_unique<DummyRenderEngine>(*this);
  }

 private:
  // If true, the engine will accept all geometries.
  bool force_accept_{};

  std::unordered_set<GeometryId> registered_geometries_;

  // The current poses of the geometries in the world frame.
  std::map<GeometryId, math::RigidTransformd> X_WGs_;
  // TODO(SeanCurtis-TRI) Shuffle this around so that the updated ids no longer
  //  redundantly stores the updated poses; those should be accessible via
  //  X_WGs_.
  // Track each id that gets updated and what it has been updated to.
  std::map<GeometryId, math::RigidTransformd> updated_ids_;

  // The group name whose presence will lead to a shape being added to the
  // engine.
  std::string include_group_name_{"in_test"};

  // The last updated camera pose (defaults to identity).
  math::RigidTransformd X_WC_;

  // The counts of the times that the various render APIs were called.
  // This should *only* apply to the render API with *simple* camera intrinsics.
  // When that API is deprecated, this can be removed.
  mutable int simple_color_count_{};
  mutable int simple_depth_count_{};
  mutable int simple_label_count_{};

  mutable render::CameraProperties color_props_;
  mutable render::DepthCameraProperties depth_props_;
  mutable render::CameraProperties label_props_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
