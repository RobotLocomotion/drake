#include "drake/geometry/render/render_material.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

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

// Support the document about what happens if you assign a string.
GTEST_TEST(TextureSourceTest, StringAssignent) {
  TextureSource source;

  auto is_path = [](const TextureSource& s) {
    return std::get_if<fs::path>(&s) != nullptr;
  };

  auto is_key = [](const TextureSource& s) {
    return std::get_if<TextureKey>(&s) != nullptr;
  };

  source = "cstr";
  EXPECT_TRUE(is_path(source));
  source = TextureKey{"cstr"};
  EXPECT_TRUE(is_key(source));

  source = std::string("str");
  EXPECT_TRUE(is_path(source));
  source = TextureKey{std::string("str")};
  EXPECT_TRUE(is_key(source));

  // Note: there is *implicit* conversion from string view to fs::path but no
  // such conversion to std::string.
  source = std::string_view("str");
  EXPECT_TRUE(is_path(source));
}

GTEST_TEST(TextureSourceTest, Empty) {
  EXPECT_TRUE(IsEmpty(TextureSource()));
  EXPECT_TRUE(IsEmpty(TextureSource(fs::path(""))));
  EXPECT_TRUE(IsEmpty(TextureSource("")));
  EXPECT_TRUE(IsEmpty(TextureSource(std::string())));
  EXPECT_TRUE(IsEmpty(TextureSource(std::string_view())));
  EXPECT_TRUE(IsEmpty(TextureSource(MemoryFile("", ".ext", "hint"))));

  TextureSource source;
  EXPECT_TRUE(IsEmpty(source));
  source = fs::path("");
  EXPECT_TRUE(IsEmpty(source));
  source = TextureKey{""};
  EXPECT_TRUE(IsEmpty(source));
  source = std::string();
  EXPECT_TRUE(IsEmpty(source));
  source = std::string_view();
  EXPECT_TRUE(IsEmpty(source));
  source = MemoryFile("", ".ext", "hint");
  EXPECT_TRUE(IsEmpty(source));
}

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
  int WarningCount() const { return ssize(warning_records_); }
  Rgba default_diffuse() const { return Rgba(0.25, 0.5, 0.75, 0.5); }
  PerceptionProperties props_;
};

/* When the properties provide no material properties, the default color is
 used. */
TEST_F(DefineMaterialTest, DefaultFallback) {
  const RenderMaterial mat =
      DefineMaterial(props_, default_diffuse(), diagnostic_policy_);

  EXPECT_TRUE(IsEmpty(mat.diffuse_map));
  EXPECT_EQ(mat.diffuse, default_diffuse());
}

/* When only the (phong, diffuse) is defined, it is used. */
TEST_F(DefineMaterialTest, PhongDiffuseOnly) {
  const Rgba diffuse(0.75, 0.75, 0.25, 0.25);
  ASSERT_NE(diffuse, default_diffuse());
  props_.AddProperty("phong", "diffuse", diffuse);

  const RenderMaterial mat =
      DefineMaterial(props_, default_diffuse(), diagnostic_policy_);

  EXPECT_TRUE(IsEmpty(mat.diffuse_map));
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

  ASSERT_TRUE(std::holds_alternative<fs::path>(mat.diffuse_map));
  EXPECT_EQ(std::get<fs::path>(mat.diffuse_map), tex_name);
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

  ASSERT_TRUE(std::holds_alternative<fs::path>(mat.diffuse_map));
  EXPECT_EQ(std::get<fs::path>(mat.diffuse_map), tex_name);
  EXPECT_EQ(mat.diffuse, diffuse);
}

/* When (phong, diffuse_map) references a "bad" image, the image is omitted and
 a warning is dispatched. The resulting material is simply white. */
TEST_F(DefineMaterialTest, DiffuseMapError) {
  props_.AddProperty("phong", "diffuse_map", "not_an_image.png");

  /* Note: we also indicate a non-full set of UVs to show that we don't complain
   about the UVs if the image itself isn't accessible. */
  const RenderMaterial mat = DefineMaterial(props_, default_diffuse(),
                                            diagnostic_policy_, UvState::kNone);

  EXPECT_TRUE(IsEmpty(mat.diffuse_map));
  EXPECT_EQ(mat.diffuse, Rgba(1, 1, 1));
  EXPECT_THAT(
      TakeWarning(),
      testing::MatchesRegex(".*referenced a map that could not be found.*"));
  /* No further warnings. */
  EXPECT_EQ(WarningCount(), 0);
}

/* When (phong, diffuse_map) references an image, but the uv state isn't "full",
 the image is omitted and a warning is dispatched. The resulting material is
 simply white. */
TEST_F(DefineMaterialTest, DiffuseMapUvCoverageError) {
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props_.AddProperty("phong", "diffuse_map", tex_name);

  /* No Uvs assigned. */
  {
    const RenderMaterial mat = DefineMaterial(
        props_, default_diffuse(), diagnostic_policy_, UvState::kNone);

    EXPECT_TRUE(IsEmpty(mat.diffuse_map));
    EXPECT_EQ(mat.diffuse, Rgba(1, 1, 1));
    EXPECT_THAT(TakeWarning(),
                testing::MatchesRegex(
                    ".*referenced a map, .* doesn't define any texture.*"));
  }

  /* Partial UVs assigned. */
  {
    const RenderMaterial mat = DefineMaterial(
        props_, default_diffuse(), diagnostic_policy_, UvState::kPartial);

    EXPECT_TRUE(IsEmpty(mat.diffuse_map));
    EXPECT_EQ(mat.diffuse, Rgba(1, 1, 1));
    EXPECT_THAT(TakeWarning(),
                testing::MatchesRegex(".*referenced a map, .* doesn't define a "
                                      "complete set of texture.*"));
  }
}

/* Tests the MaybeMakeMeshFallbackMaterial() function. This function should only
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
class MaybeMakeMeshFallbackMaterialTest
    : public test::DiagnosticPolicyTestBase {
 protected:
  static Rgba default_diffuse() { return Rgba(0.125, 0.25, 0.375, 0.5); }
};

/* No material defined in the properties and no foo.png --> default-colored
 material. This doesn't test the case where foo.png *does* exist, but isn't
 available. We're not testing the "unaccessible foo.png" case. Not worth it
 in light of its imminent death.  */
TEST_F(MaybeMakeMeshFallbackMaterialTest, DefaultDiffuseMaterial) {
  PerceptionProperties props;

  const std::optional<RenderMaterial> mat = MaybeMakeMeshFallbackMaterial(
      props, "no_png_for_this.obj", default_diffuse(), diagnostic_policy_,
      UvState::kFull);
  ASSERT_TRUE(mat.has_value());
  EXPECT_TRUE(IsEmpty(mat->diffuse_map));
  EXPECT_EQ(mat->diffuse, default_diffuse());
}

/* No material defined in the properties and no foo.png --> default-colored
 material. The default diffuse color is not provided either. No
 material-generating condition is met and std::nullopt is expected.  */
TEST_F(MaybeMakeMeshFallbackMaterialTest, NoMaterial) {
  PerceptionProperties props;

  const std::optional<RenderMaterial> mat =
      MaybeMakeMeshFallbackMaterial(props, "no_png_for_this.obj", std::nullopt,
                                    diagnostic_policy_, UvState::kFull);
  EXPECT_FALSE(mat.has_value());
}

/* No material defined in the properties, but foo.png exists and is available.
 Also covers the special case where the mesh path is empty -- it is treated as
 if there is no compatible foo.png and it falls through to apply the default
 diffuse. */
TEST_F(MaybeMakeMeshFallbackMaterialTest, ValidFooPngMaterial) {
  PerceptionProperties props;
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");
  const fs::path tex_path(tex_name);
  /* N.B. The obj doesn't actually have to exist for this test to work. */
  fs::path obj_path = tex_path.parent_path() / "box.obj";

  struct TestCase {
    UvState uv_state;
    std::string expected_texture;
    std::string error;
    fs::path path;
    Rgba rgba = Rgba(1, 1, 1);  // Default rgb for auto-loaded texture.
    std::string description;
  };

  const std::vector<TestCase> cases{
      {.uv_state = UvState::kFull,
       .expected_texture = tex_name,
       .path = obj_path,
       .description = "Full UVs"},
      {.uv_state = UvState::kPartial,
       .error = "a complete set of",
       .path = obj_path,
       .description = "Partial UVs"},
      {.uv_state = UvState::kNone,
       .error = "any",
       .path = obj_path,
       .description = "No UVs"},
      {.uv_state = UvState::kFull,
       .path = fs::path(),
       .rgba = default_diffuse(),
       .description = "Empty path produces default diffuse"}};
  for (const TestCase& test_case : cases) {
    SCOPED_TRACE(test_case.description);

    const std::optional<RenderMaterial> mat =
        MaybeMakeMeshFallbackMaterial(props, test_case.path, default_diffuse(),
                                      diagnostic_policy_, test_case.uv_state);
    ASSERT_TRUE(mat.has_value());
    ASSERT_EQ(IsEmpty(mat->diffuse_map), test_case.expected_texture.empty());
    if (!test_case.expected_texture.empty()) {
      EXPECT_EQ(std::get<fs::path>(mat->diffuse_map),
                test_case.expected_texture);
    }
    EXPECT_EQ(mat->diffuse, test_case.rgba);
    if (!test_case.error.empty()) {
      EXPECT_THAT(
          TakeWarning(),
          testing::MatchesRegex(fmt::format(
              ".*png file of the same name .* doesn't define {} texture.*",
              test_case.error)));
    }
  }
}

/* The presence of any material property should create a material.
 MaybeMakeMeshFallbackMaterial() defers to DefineMaterial() in the presence of
 material properties. For diffuse color, it passes a white default. We just need
 evidence that suggests it gets called as expected.

 We test properties independently and combined to make sure the parameters
 are all passed to DefineMaterial(). We don't explicitly test for degenerate
 cases (inaccessible texture), because DefineMaterial() handles that and has
 been tested above.

 As we extend the set of material properties Drake knows about, we should add
 an independent test for each one, and each should likewise be added into the
 PropertiesHaveEverything test. */
TEST_F(MaybeMakeMeshFallbackMaterialTest, PropertiesHaveDiffuseColor) {
  PerceptionProperties props;
  props.AddProperty("phong", "diffuse", Rgba(0.25, 0.5, 0.75, 0.5));
  const std::optional<RenderMaterial> mat = MaybeMakeMeshFallbackMaterial(
      props, "doesn't_matter.obj", default_diffuse(), diagnostic_policy_,
      UvState::kFull);

  ASSERT_TRUE(mat.has_value());
  EXPECT_TRUE(IsEmpty(mat->diffuse_map));
  EXPECT_EQ(mat->diffuse, props.GetProperty<Rgba>("phong", "diffuse"));
}

TEST_F(MaybeMakeMeshFallbackMaterialTest, PropertiesHaveDiffuseMap) {
  PerceptionProperties props;
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props.AddProperty("phong", "diffuse_map", tex_name);
  const std::optional<RenderMaterial> mat = MaybeMakeMeshFallbackMaterial(
      props, "doesn't_matter.obj", default_diffuse(), diagnostic_policy_,
      UvState::kFull);

  ASSERT_TRUE(mat.has_value());
  ASSERT_TRUE(std::holds_alternative<fs::path>(mat->diffuse_map));
  EXPECT_EQ(std::get<fs::path>(mat->diffuse_map), tex_name);
  EXPECT_EQ(mat->diffuse, Rgba(1, 1, 1));
}

TEST_F(MaybeMakeMeshFallbackMaterialTest, PropertiesHaveEverything) {
  PerceptionProperties props;
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props.AddProperty("phong", "diffuse_map", tex_name);
  props.AddProperty("phong", "diffuse", Rgba(0.25, 0.5, 0.75, 0.5));
  const std::optional<RenderMaterial> mat = MaybeMakeMeshFallbackMaterial(
      props, "doesn't_matter.obj", default_diffuse(), diagnostic_policy_,
      UvState::kFull);

  ASSERT_TRUE(mat.has_value());
  ASSERT_TRUE(std::holds_alternative<fs::path>(mat->diffuse_map));
  EXPECT_EQ(std::get<fs::path>(mat->diffuse_map), tex_name);
  EXPECT_EQ(mat->diffuse, props.GetProperty<Rgba>("phong", "diffuse"));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
