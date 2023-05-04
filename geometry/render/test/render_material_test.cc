#include "drake/geometry/render/render_material.h"

#include <filesystem>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

namespace fs = std::filesystem;

/* Confirms the expected diagnostic warnings in the presence of material
 properties. */
class MaybeWarnForRedundantMaterialTest
    : public test::DiagnosticPolicyTestBase {};

/* Confirm the presence of a diffuse color dispatches a warning. */
TEST_F(MaybeWarnForRedundantMaterialTest, WarningsDispatchedDiffuseColor) {
  PerceptionProperties props;
  props.AddProperty("phong", "diffuse", Rgba(0.1, 0.2, 0.3, 0.4));
  MaybeWarnForRedundantMaterial(props, "dummy_file_name", diagnostic_policy_);
  EXPECT_THAT(TakeWarning(), testing::ContainsRegex(
                                 "has its own materials.*'phong', 'diffuse'"));
}

/* Confirm the presence of a diffuse map dispatches a warning. */
TEST_F(MaybeWarnForRedundantMaterialTest, WarningsDispatchedDiffuseMap) {
  PerceptionProperties props;
  props.AddProperty("phong", "diffuse_map", "no_such.png");
  MaybeWarnForRedundantMaterial(props, "dummy_file_name", diagnostic_policy_);
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex(
                  ".*has its own materials.*'phong', 'diffuse_map'.*"));
}

/* Tests the DefineMaterial() function (with the potential to dispatch warnings
 to a diagnostic policy). */
class DefineMaterialTest : public test::DiagnosticPolicyTestBase {
 protected:
  Rgba default_diffuse() const { return Rgba(0.25, 0.5, 0.75, 0.5); }
  PerceptionProperties props_;
};

/* When the properties provide no material properties, the default color is
 used. */
TEST_F(DefineMaterialTest, DefaultFallback) {
  const RenderMaterial mat =
      DefineMaterial(props_, default_diffuse(), diagnostic_policy_);

  EXPECT_TRUE(mat.diffuse_map.empty());
  EXPECT_EQ(mat.diffuse, default_diffuse());
}

/* When only the (phong, diffuse) is defined, it is used. */
TEST_F(DefineMaterialTest, PhongDiffuseOnly) {
  const Rgba diffuse(0.75, 0.75, 0.25, 0.25);
  ASSERT_NE(diffuse, default_diffuse());
  props_.AddProperty("phong", "diffuse", diffuse);

  const RenderMaterial mat =
      DefineMaterial(props_, default_diffuse(), diagnostic_policy_);

  EXPECT_TRUE(mat.diffuse_map.empty());
  EXPECT_EQ(mat.diffuse, diffuse);
}

/* When only the (phong, diffuse_map) is defined, it is used with a white
 diffuse color. */
TEST_F(DefineMaterialTest, PhongDiffuseMapOnly) {
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props_.AddProperty("phong", "diffuse_map", tex_name);

  const RenderMaterial mat =
      DefineMaterial(props_, default_diffuse(), diagnostic_policy_);

  EXPECT_EQ(mat.diffuse_map, tex_name);
  EXPECT_EQ(mat.diffuse, Rgba(1, 1, 1));
}

/* When diffuse and diffuse_map are defined, both get used exactly. */
TEST_F(DefineMaterialTest, PhongDiffuseAll) {
  const Rgba diffuse(0.75, 0.75, 0.25, 0.25);
  ASSERT_NE(diffuse, default_diffuse());
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props_.AddProperty("phong", "diffuse_map", tex_name);
  props_.AddProperty("phong", "diffuse", diffuse);

  const RenderMaterial mat =
      DefineMaterial(props_, default_diffuse(), diagnostic_policy_);

  EXPECT_EQ(mat.diffuse_map, tex_name);
  EXPECT_EQ(mat.diffuse, diffuse);
}

/* When (phong, diffuse_map) references a "bad" image, the image is omitted and
 a warning is dispatched. The resulting material is simply white. */
TEST_F(DefineMaterialTest, DiffuseMapError) {
  props_.AddProperty("phong", "diffuse_map", "not_an_image.png");

  const RenderMaterial mat =
      DefineMaterial(props_, default_diffuse(), diagnostic_policy_);

  EXPECT_TRUE(mat.diffuse_map.empty());
  EXPECT_EQ(mat.diffuse, Rgba(1, 1, 1));
  EXPECT_THAT(
      TakeWarning(),
      testing::MatchesRegex(".*referenced a map that could not be found.*"));
}

/* Tests the MakeMeshFallbackMaterial() function. This function should only
 be called if the mesh has no intrinsic material. Its unique duties are:

   - Determine if a material is defined *at all* in the geometry properties and
     delegate to DefineMaterial() if so.
   - Otherwise, look for foo.png for foo.obj and apply it if it exists.
   - Otherwise simply apply a default-colored material.

 The tests below assume:
  1. That the invocation of DefineMaterial() is correct, and
  2. DefineMaterial() has been sufficiently tested.

 Note: the only diagnostic policy messages that get sent are attributable to
 DefineMaterial(), so they are not directly accounted for in this test. */
class MakeMeshFallbackMaterialTest : public test::DiagnosticPolicyTestBase {
 protected:
  static Rgba default_diffuse() { return Rgba(0.125, 0.25, 0.375, 0.5); }
};

/* No material defined in the properties and no foo.png --> default-colored
 material. This doesn't test the case where foo.png *does* exist, but isn't
 available. We're not testing the "unaccessible foo.png" case. Not worth it
 in light of its imminent death.  */
TEST_F(MakeMeshFallbackMaterialTest, DefaultDiffuseMaterial) {
  PerceptionProperties props;

  const RenderMaterial mat = MakeMeshFallbackMaterial(
      props, "no_png_for_this.obj", default_diffuse(), diagnostic_policy_);

  EXPECT_TRUE(mat.diffuse_map.empty());
  EXPECT_EQ(mat.diffuse, default_diffuse());
}

/* No material defined in the properties, but foo.png exists and is available.*/
TEST_F(MakeMeshFallbackMaterialTest, ValidFooPngMaterial) {
  PerceptionProperties props;
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");
  const fs::path tex_path(tex_name);
  // N.B. The obj doesn't actually have to exist for this test to work.
  fs::path obj_path = tex_path.parent_path() / "box.obj";

  const RenderMaterial mat = MakeMeshFallbackMaterial(
      props, obj_path.string(), default_diffuse(), diagnostic_policy_);

  EXPECT_EQ(mat.diffuse_map, tex_name);
  EXPECT_EQ(mat.diffuse, Rgba(1, 1, 1));
}

/* The presence of any material property should create a material.
 MakeMeshFallbackMaterial() defers to DefineMaterial() in the presence of
 material properties. For diffuse color, it passes a white default. We just need
 evidence that suggests it gets called as expected.

 We test properties independently and combined to make sure the parameters
 are all passed to DefineMaterial(). We don't explicitly test for degenerate
 cases (inaccessible texture), because DefineMaterial() handles that and has
 been tested above.

 As we extend the set of material properties Drake knows about, we should add
 an independent test for each one, and each should likewise be added into the
 PropertiesHaveEverything test. */
TEST_F(MakeMeshFallbackMaterialTest, PropertiesHaveDiffuseColor) {
  PerceptionProperties props;
  props.AddProperty("phong", "diffuse", Rgba(0.25, 0.5, 0.75, 0.5));
  const RenderMaterial mat = MakeMeshFallbackMaterial(
      props, "doesn't_matter.obj", default_diffuse(), diagnostic_policy_);

  EXPECT_TRUE(mat.diffuse_map.empty());
  EXPECT_EQ(mat.diffuse, props.GetProperty<Rgba>("phong", "diffuse"));
}

TEST_F(MakeMeshFallbackMaterialTest, PropertiesHaveDiffuseMap) {
  PerceptionProperties props;
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props.AddProperty("phong", "diffuse_map", tex_name);
  const RenderMaterial mat = MakeMeshFallbackMaterial(
      props, "doesn't_matter.obj", default_diffuse(), diagnostic_policy_);

  EXPECT_EQ(mat.diffuse_map, tex_name);
  EXPECT_EQ(mat.diffuse, Rgba(1, 1, 1));
}

TEST_F(MakeMeshFallbackMaterialTest, PropertiesHaveEverything) {
  PerceptionProperties props;
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props.AddProperty("phong", "diffuse_map", tex_name);
  props.AddProperty("phong", "diffuse", Rgba(0.25, 0.5, 0.75, 0.5));
  const RenderMaterial mat = MakeMeshFallbackMaterial(
      props, "doesn't_matter.obj", default_diffuse(), diagnostic_policy_);

  EXPECT_EQ(mat.diffuse_map, tex_name);
  EXPECT_EQ(mat.diffuse, props.GetProperty<Rgba>("phong", "diffuse"));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
