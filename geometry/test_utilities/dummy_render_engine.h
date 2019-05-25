#pragma once

#include <map>
#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/color_palette.h"

namespace drake {
namespace geometry {
namespace internal {

/** A dummy implementation of the RenderEngine to facilitate testing. Its
 purpose is to facilitate testing at the RenderEngine interface and _not_ the
 details of any particular implementation. To that end, it has the following
 features:

 1. It makes the RenderLabel-Color conversion methods of the RenderEngine
    public to this renderer (so test infrastructure can invoke the conversion).
 2. It facilitates testing RemoveGeometry() logic by allowing the test to
    explicitly set what RenderIndex this dummy engine returns in
    DoRemoveGeometry().
 3. It facilitates testing the RegisterGeometry() method by making the
    registration of a geometry dependent on particular contents in the
    PerceptionProperties (see accepting_properties() and
    rejecting_properties()).
 4. Records which poses have been updated via UpdatePoses() to validate which
    RenderIndex values are updated and which aren't (and with what pose).  */
class DummyRenderEngine final : public render::RenderEngine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyRenderEngine);
  /** Constructor to exercise the default constructor of RenderEngine.  */
  DummyRenderEngine() = default;

  /** Constructor for configuring the underlying RenderEngine.  */
  explicit DummyRenderEngine(const render::RenderLabel& label)
      : render::RenderEngine(label) {}

  /** @group No-op implementation of RenderEngine interface.  */
  //@{
  void UpdateViewpoint(const math::RigidTransformd&) const final {}
  void RenderColorImage(const render::CameraProperties&, bool,
                        systems::sensors::ImageRgba8U*) const final {}
  void RenderDepthImage(const render::DepthCameraProperties&,
                        systems::sensors::ImageDepth32F*) const final {}
  void RenderLabelImage(const render::CameraProperties&, bool,
                        systems::sensors::ImageLabel16I*) const final {}
  void ImplementGeometry(const Sphere& sphere, void* user_data) final {}
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final {}
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final {}
  void ImplementGeometry(const Box& box, void* user_data) final {}
  void ImplementGeometry(const Mesh& mesh, void* user_data) final {}
  void ImplementGeometry(const Convex& convex, void* user_data) final {}
  //@}

  /** @name  Functions for supporting tests.  */
  //@{

  /** Creates a set of perception properties that will cause this render engine
   to _accept_ geometry during registration.  */
  PerceptionProperties accepting_properties() const {
    PerceptionProperties properties;
    properties.AddProperty(include_group_name_, "ignored", 0);
    return properties;
  }

  /** Creates a set of perception properties that will cause this render engine
   to _reject_ geometry during registration.  */
  PerceptionProperties rejecting_properties() const {
    return PerceptionProperties();
  }

  /** Reports the number of geometries that have been _accepted_ in
   registration.  */
  int num_registered() const { return register_count_; }

  /** Returns the indices that have been updated via a call to UpdatePoses() and
   the poses that were set.  */
  const std::map<RenderIndex, math::RigidTransformd>& updated_indices() const {
    return updated_indices_;
  }

  /** Sets the RenderIndex that DoRemoveGeometry() returns. Set to `nullopt`
   to have RemoveGeometry() do no additional work. Otherwise, set it to some
   valid index to cause index re-ordering.  */
  void set_moved_index(optional<RenderIndex> index) {
    moved_index_ = index;
  }

  // Promote these to be public to facilitate testing.
  using RenderEngine::LabelFromColor;
  using RenderEngine::GetColorDFromLabel;
  using RenderEngine::GetColorIFromLabel;

 protected:
  /** Dummy implementation that registers the given `shape` iff the `properties`
   contains the "in_test" group. (Also counts the number of successfully
   registered shape over the lifespan of `this` instance.)  */
  optional<RenderIndex> DoRegisterVisual(const Shape&,
                                         const PerceptionProperties& properties,
                                         const math::RigidTransformd&) final {
    GetRenderLabelOrThrow(properties);
    if (properties.HasGroup(include_group_name_)) {
      return RenderIndex(register_count_++);
    }
    return nullopt;
  }

  /** Updates the pose X_WG for the geometry with the given `index`. Also tracks
   which indices have been updated and the poses set (over the _lifespan_ of
   `this` instance).  */
  void DoUpdateVisualPose(RenderIndex index,
                          const math::RigidTransformd& X_WG) final {
    updated_indices_[index] = X_WG;
  }

  /** Simulates removing a geometry and returns the index defined by
   set_moved_index().  */
  optional<RenderIndex> DoRemoveGeometry(RenderIndex index) final {
    return moved_index_;
  }

  /** Implementation of RenderEngine::DoClone().  */
  std::unique_ptr<render::RenderEngine> DoClone() const final {
    return std::make_unique<DummyRenderEngine>(*this);
  }

 private:
  // Number of registered geometries.
  int register_count_{};

  // Track each index and what it has been updated to (allows us to confirm
  // RenderIndex and GeometryIndex association.
  std::map<RenderIndex, math::RigidTransformd> updated_indices_;

  // The group name whose presence will lead to a shape being added to the
  // engine.
  std::string include_group_name_{"in_test"};

  // The RenderIndex value to return on invocation of DoRemoveGeometry().
  optional<RenderIndex> moved_index_{};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
