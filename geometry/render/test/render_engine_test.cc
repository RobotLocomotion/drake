#include "drake/geometry/render/render_engine.h"

#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

// Dummy implementation of the RenderEngine interface to facilitate testing
// the specific RenderEngine functionality.
class DummyRenderEngine final : public RenderEngine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyRenderEngine);
  DummyRenderEngine() = default;
  void AddFlatTerrain() final {}
  void UpdateViewpoint(const Eigen::Isometry3d&) const final {}
  void RenderColorImage(const render::CameraProperties&,
                        systems::sensors::ImageRgba8U*, bool) const final {}
  void RenderDepthImage(const render::DepthCameraProperties&,
                        systems::sensors::ImageDepth32F*) const final {}
  void RenderLabelImage(const render::CameraProperties&,
                        systems::sensors::ImageLabel16I*, bool) const final {}
  void ImplementGeometry(const Sphere& sphere, void* user_data) final {}
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final {}
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final {}
  void ImplementGeometry(const Box& box, void* user_data) final {}
  void ImplementGeometry(const Mesh& mesh, void* user_data) final {}
  void ImplementGeometry(const Convex& convex, void* user_data) final {}

  // Test harness.

  // Provide access to the group name that implies full registration.
  const std::string& include_group_name() const { return include_group_name_; }

  // Return the indices that have been updated via a call to UpdatePoses().
  const std::map<RenderIndex, Eigen::Isometry3d>& updated_indices() const {
    return updated_indices_;
  }

 protected:
  // Conditionally register the visual based on the properties having an
  // "in_test" group.
  optional<RenderIndex> DoRegisterVisual(const Shape&,
                                         const PerceptionProperties& properties,
                                         const Isometry3<double>&) final {
    if (properties.HasGroup(include_group_name_)) {
      return RenderIndex(++register_count_);
    }
    return nullopt;
  }

  // Track all of the indices provided an update pose for.
  void DoUpdateVisualPose(const Eigen::Isometry3d& X_WG,
                          RenderIndex index) final {
    updated_indices_[index] = X_WG;
  }

  std::unique_ptr<render::RenderEngine> DoClone() const final {
    return std::make_unique<DummyRenderEngine>(*this);
  }

 private:
  int register_count_{};
  // Track each index and what it has been updated to (allows us to confirm
  // RenderIndex and GeometryIndex association.
  std::map<RenderIndex, Eigen::Isometry3d> updated_indices_;
  // The group name whose presence will lead to a shape being added to the
  // engine.
  std::string include_group_name_{"in_test"};
};

// Tests the RenderEngine-specific functionality for managing registration of
// geometry and its corresponding update behavior. The former should configure
// each geometry correctly on whether it gets updated or not, and the latter
// will confirm that the right geometries get updated.
GTEST_TEST(RenderEngine, RegistrationAndUpdate) {
  DummyRenderEngine engine;

  // Configure parameters for registering visuals.
  PerceptionProperties skip_properties;
  PerceptionProperties add_properties;
  add_properties.AddProperty(engine.include_group_name(), "ignored", 0);
  Sphere sphere(1.0);
  Isometry3<double> X_WG = Isometry3<double>::Identity();
  // A collection of poses to provide to calls to UpdatePoses(). Configured
  // to all identity transforms because the values generally don't matter. In
  // the single case where it does matter, a value is explicitly set (see
  // below).
  std::vector<Isometry3<double>> X_WG_all{3, X_WG};

  // These test cases are accumulative; re-ordering them will require
  // refactoring.

  // Case: the shape is configured to be ignored by the render engine. Returns
  // nullopt (and other arguments do not matter).
  optional<RenderIndex> optional_index = engine.RegisterVisual(
      GeometryIndex(0), sphere, skip_properties, X_WG, false);
  EXPECT_FALSE(optional_index);
  optional_index = engine.RegisterVisual(GeometryIndex(0), sphere,
                                         skip_properties, X_WG, true);
  EXPECT_FALSE(optional_index);
  // Confirm nothing is updated - because nothing is registered.
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_indices().size(), 0);

  // Case: the shape is configured for registration, but does *not* require
  // updating. We get a valid render index, but it is _not_ included in
  // UpdatePoses().
  optional_index = engine.RegisterVisual(GeometryIndex(1), sphere,
                                         add_properties, X_WG, false);
  EXPECT_TRUE(optional_index);
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_indices().size(), 0);

  // Case: the shape is configured for registration *and* requires updating. We
  // get a valid render index and it _is_ included in UpdatePoses().
  GeometryIndex update_index{2};
  // Configure the pose for index 2 to *not* be the identity so we can confirm
  // that the registered GeometryIndex is properly associated with the resulting
  // RenderIndex.
  Vector3<double> p_WG(1, 2, 3);
  X_WG_all[update_index].translation() << p_WG;
  optional_index =
      engine.RegisterVisual(update_index, sphere, add_properties, X_WG, true);
  EXPECT_TRUE(optional_index);
  engine.UpdatePoses(X_WG_all);
  EXPECT_EQ(engine.updated_indices().size(), 1);
  ASSERT_EQ(engine.updated_indices().count(*optional_index), 1);
  EXPECT_TRUE(CompareMatrices(
      engine.updated_indices().at(*optional_index).translation(), p_WG));
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
