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
  DummyRenderEngine() : DummyRenderEngine(render::RenderLabel::kUnspecified) {}

  /* Constructor for configuring the underlying RenderEngine.  */
  explicit DummyRenderEngine(const render::RenderLabel& label)
      : render::RenderEngine(label),
        color_camera_{{"", {2, 2, 1.0}, {0.01, 1}, {}}, false},
        depth_camera_{color_camera_.core(), {0.01, 0.011}},
        label_camera_{color_camera_} {}

  /* @group No-op implementation of RenderEngine interface.  */
  //@{
  void UpdateViewpoint(const math::RigidTransformd& X_WC) override {
    X_WC_ = X_WC;
  }
  using render::RenderEngine::RenderColorImage;
  using render::RenderEngine::RenderDepthImage;
  using render::RenderEngine::RenderLabelImage;

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
  int num_color_renders() const { return color_count_;  }
  int num_depth_renders() const { return depth_count_;  }
  int num_label_renders() const { return label_count_;  }

  const render::ColorRenderCamera& last_color_camera() const {
    return color_camera_;
  }
  const render::DepthRenderCamera& last_depth_camera() const {
    return depth_camera_;
  }
  const render::ColorRenderCamera& last_label_camera() const {
    return label_camera_;
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

  /* Implementation of RenderEngine::DoRenderColorImage().  */
  void DoRenderColorImage(const render::ColorRenderCamera& camera,
                          systems::sensors::ImageRgba8U*) const override {
    ++color_count_;
    color_camera_ = camera;
  }

  /* Implementation of RenderEngine::DoRenderDepthImage().  */
  void DoRenderDepthImage(const render::DepthRenderCamera& camera,
                          systems::sensors::ImageDepth32F*) const override {
    ++depth_count_;
    depth_camera_ = camera;
  }

  /* Implementation of RenderEngine::DoRenderLabelImage().  */
  void DoRenderLabelImage(const render::ColorRenderCamera& camera,
                          systems::sensors::ImageLabel16I*) const override {
    ++label_count_;
    label_camera_ = camera;
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
  mutable int color_count_{};
  mutable int depth_count_{};
  mutable int label_count_{};

  mutable render::ColorRenderCamera color_camera_;
  mutable render::DepthRenderCamera depth_camera_;
  mutable render::ColorRenderCamera label_camera_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
