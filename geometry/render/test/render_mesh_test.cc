#include "drake/geometry/render/render_mesh.h"

#include <filesystem>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/test/diagnostic_policy_test_base.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

namespace fs = std::filesystem;

/* The tests for this class are partitioned into several categories defined by
 prefix:

   - Geometry: evaluation of the geometry produced.
   - WithMaterial: various valid spellings that should resolve into a single
     mesh with a valid material specificaiton.
   - NoMaterial: the reported RenderMesh has no material. In some cases, a
     warning may be dispatched.
   - Error: These test the cases in which we *throw* -- there is no recovery.
 */
class RenderMeshTest : public multibody::test::DiagnosticPolicyTestBase {
 protected:
  static fs::path WriteFile(std::string_view contents,
                            std::string_view file_name,
                            const fs::path& dir = kTempDir) {
    std::cerr << contents << "\n";
    const fs::path file_path = dir / file_name;
    std::ofstream out(file_path);
    DRAKE_DEMAND(out.is_open());
    out << contents;
    std::cerr << file_path << "\n";
    return file_path;
  }

  static void SetUpTestSuite() {
    // Make sure we have a valid texture with the mtl file that names it.
    fs::path image_source_path(
        FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png"));
    fs::path image_target_path = kTempDir / "diag_gradient.png";
    fs::copy(image_source_path, image_target_path);

    WriteFile(R"""(
    newmtl test_material
    Ka 0.1 0.1 0.1
    Kd 1.0 1.0 1.0
    Ks 1.0 1.0 1.0
    d 1
    illum 2
    Ns 0
  )""",
              basic_mtl);

    WriteFile(R"""(
    newmtl test_material_1
    Kd 1.0 1.0 1.0

    newmtl test_material_2
    Kd 1.0 0.0 0.0
  )""",
              multi_mtl);

    WriteFile(R"""(
    newmtl test_material
    Ka 0.1 0.1 0.1
    Kd 1.0 1.0 1.0
    Ks 1.0 1.0 1.0
    d 1
    illum 2
    Ns 0
    map_Kd diag_gradient.png
  )""",
              texture_mtl);
  }

  static constexpr char basic_mtl[] = "basic.mtl";
  static constexpr char multi_mtl[] = "multi.mtl";
  static constexpr char texture_mtl[] = "texture.mtl";
  static const fs::path kTempDir;
};

const fs::path RenderMeshTest::kTempDir = temp_directory();

/* A vague smoke test of the geometry buffer data. Failure in this part of the
 algorithm will be immediately obvious in any visualization. So, this doesn't
 attempt to exhaustively confirm correctness. Just provide a couple of
 indicators as an early CI warning against regression. */
TEST_F(RenderMeshTest, GeometryData) {
  /* Normals and UVs get copied for each vertex */
  {
    const fs::path obj_path = WriteFile(R"""(
    v 0 0 0
    v 0 1 0
    v 0 0 1
    vn 0 1 0
    vt 0 1
    f 1/1/1 2/1/1 3/1/1
  )""",
                                        "geo_simple.obj");
    const RenderMesh data = LoadMeshFromObj(obj_path, diagnostic_policy_);
    EXPECT_EQ(data.positions.rows(), 3);
    EXPECT_EQ(data.normals.rows(), 3);
    EXPECT_EQ(data.uvs.rows(), 3);
    EXPECT_EQ(data.indices.rows(), 1);
    EXPECT_TRUE(data.has_tex_coord);
  }

  /* No UVs creates a bunch of zero uv values. */
  {
    const fs::path obj_path = WriteFile(R"""(
    v 0 0 0
    v 0 1 0
    v 0 0 1
    vn 0 1 0
    f 1//1 2//1 3//1
  )""",
                                        "geo_simple_no_uv.obj");
    const RenderMesh data = LoadMeshFromObj(obj_path, diagnostic_policy_);
    EXPECT_EQ(data.positions.rows(), 3);
    EXPECT_EQ(data.normals.rows(), 3);
    EXPECT_EQ(data.uvs.rows(), 3);
    EXPECT_TRUE(CompareMatrices(data.uvs, Eigen::Matrix<float, 3, 2>::Zero()));
    EXPECT_EQ(data.indices.rows(), 1);
    EXPECT_FALSE(data.has_tex_coord);
  }
}

/* Confirm that when a material is actually created, that the values get stored
 as expected. This is the only time we'll peek under the hood at the material
 values. */
TEST_F(RenderMeshTest, WithMaterialValueTest) {
  WriteFile(R"""(
    newmtl ad_hoc_mat
    Kd 0.25 0.5 0.75
    d 0.125
    map_Kd diag_gradient.png
  )""",
            "ad_hoc.mtl");
  const fs::path obj_path = WriteFile(R"""(
    mtllib ad_hoc.mtl
    v 0 0 0
    v 1 1 1
    v 2 2 2
    vn 0 1 0
    vt 0 1
    usemtl ad_hoc_mat
    f 1/1/1 2/1/1 3/1/1)""",
                                      "ad_hoc.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(0.25, 0.5, 0.75, 0.125));
  EXPECT_EQ(mesh.material->diffuse_map,
            (kTempDir / "diag_gradient.png").string());
}

/* An .mtl file with multiple material definitions is not a problem as long as
 only one is used. */
TEST_F(RenderMeshTest, WithMaterialExtraMaterialsDefined) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material_1
        f 1//1 2//1 3//1
        f 1//1 2//1 3//1)""",
                                                  multi_mtl),
                                      "use_only_one_material.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_TRUE(mesh.material.has_value());
}

/* We're not sensitive to how many times `usemtl` is invoked, only that, at the
 end of the day, a single material is applied to all faces. */
TEST_F(RenderMeshTest, WithMaterialRedundantUsemtl) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material
        f 1//1 2//1 3//1
        usemtl test_material
        f 1//1 2//1 3//1)""",
                                                  basic_mtl),
                                      "redundant_usemtl.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_TRUE(mesh.material.has_value());
}

/* The image doesn't have to be collocated, just available. */
TEST_F(RenderMeshTest, WithMaterialDislocatedTexture) {
  const fs::path obj_dir = kTempDir / "dis_texture";
  fs::create_directory(obj_dir);
  const fs::path obj_path = WriteFile(R"""(
    mtllib my.mtl
    v 0 0 0
    v 1 1 1
    v 2 2 2
    vn 0 1 0
    vt 0 1
    usemtl test_material
    f 1/1/1 2/1/1 3/1/1)""",
                                      "my.obj", obj_dir);
  WriteFile(R"""(
    newmtl test_material
    Kd 1.0 1.0 1.0
    d 1
    map_Kd ../diag_gradient.png)""",
            "my.mtl", obj_dir);
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse_map,
            (kTempDir / "diag_gradient.png").string());
}

/* If the obj specifies no mtl, there will be no material and no warning. */
TEST_F(RenderMeshTest, NoMaterialNoMtllib) {
  const fs::path obj_path = WriteFile(R"""(
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1)""",
                                      "no_mtllib.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
}

/* If the obj doesn't actually *use* any materials, there will be no material
 and no warning. */
TEST_F(RenderMeshTest, NoMaterialNoUsemtl) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1)""",
                                                  basic_mtl),
                                      "no_usemtl.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
}

/* If the applied material has a texture but the mesh doesn't have valid uvs,
 there will be a warning and no material */
TEST_F(RenderMeshTest, NoMaterialTextureWithoutUvs) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material
        f 1//1 2//1 3//1)""",
                                                  texture_mtl),
                                      "no_uvs_with_texture.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex(".*requested a diffuse texture.*doesn't "
                                    "define texture coordinates.*"));
}

/* If we can't find the specified mtl file, there will be warning and no
 material. */
TEST_F(RenderMeshTest, NoMaterialMissingMtl) {
  const fs::path obj_path = WriteFile(R"""(
        mtllib not_really_a.mtl
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material
        f 1//1 2//1 3//1)""",
                                      "non_existent_mtl.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("Material file [ not_really_a.mtl ] not found"));
}

/* If the obj references an invalid material name, there will be a warning an
 no material. */
TEST_F(RenderMeshTest, NoMaterialBadMaterialName) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl bad_material
        f 1//1 2//1 3//1)""",
                                                  texture_mtl),
                                      "bad_material_name.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  EXPECT_THAT(TakeWarning(),
              testing::HasSubstr("material [ 'bad_material' ] not found"));
}

/* If the mtl file references a texture that can't be found, there will be a
 warning and no material. */
TEST_F(RenderMeshTest, NoMaterialUnavailableTexture) {
  WriteFile("newmtl test_material\nmap_Kd cant_be_found.png\n",
            "missing_texture.mtl");
  const fs::path obj_path = WriteFile(R"""(
        mtllib missing_texture.mtl
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material
        f 1/1/1 2/1/1 3/1/1)""",
                                      "missing_texture.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("requested an unavailable diffuse texture image"));
}

/* If the obj applies too many materials, there will be a warning and no
 material. */
TEST_F(RenderMeshTest, NoMaterialTooManyUsemtl) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material_1
        f 1//1 2//1 3//1        
        usemtl test_material_2
        f 1//1 2//1 3//1)""",
                                                  multi_mtl),
                                      "too_many_usemtl.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("use a single material across the whole mesh"));
}

/* If the obj applies a material to *some* of the faces, there will be a warning
 and no material. */
TEST_F(RenderMeshTest, NoMaterialDefaultedFaces) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1        
        usemtl test_material_2
        f 1//1 2//1 3//1)""",
                                                  multi_mtl),
                                      "default_material_faces.obj");
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("use a single material across the whole mesh"));
}

/* If the obj references the mtl file with an absolute path, there will be a
 warning and no material (due to tiny obj logic). */
TEST_F(RenderMeshTest, NoMaterialObjMtlDislocationAbsolute) {
  // Note: This is an absolute path that we imbed in the obj. This makes tinyobj
  // angry.
  const fs::path mtl_path = kTempDir / texture_mtl;
  // This will be a *different* temp directory than for the test suite.
  const fs::path obj_dir(temp_directory());
  const fs::path obj_path = obj_dir / "mtl_elsewhere.obj";
  {
    std::ofstream out(obj_path);
    DRAKE_DEMAND(out.is_open());
    out << fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material
        f 1/1/1 2/1/1 3/1/1)""",
                       mtl_path.string());
  }
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  // The tinyobj warning that the material library can't be found.
  EXPECT_THAT(TakeWarning(), testing::HasSubstr("not found in a path"));
}

/* This test merely exposes a known flaw in the code. Images are defined
 relative to the mtl file, but we don't know the mtl file that got read. So,
 we *assume* mtl and obj are in the same place. If not, it can erroneously lead
 to not being able to located used textures. The victory condition is that when
 we fix this flaw, this test fails and we reverse the semantics: a valid
 material is included in the resulting mesh data. */
TEST_F(RenderMeshTest, NoMaterialObjMtlDislocationRelative) {
  const fs::path obj_dir = kTempDir / "relative_obj";
  fs::create_directory(obj_dir);
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib ../{}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material
        f 1/1/1 2/1/1 3/1/1)""",
                                                  texture_mtl),
                                      "mtl_elsewhere.obj", obj_dir);
  const RenderMesh mesh = LoadMeshFromObj(obj_path, diagnostic_policy_);
  EXPECT_FALSE(mesh.material.has_value());
  // Because of our limited access to parsing information, this will look like
  // the image is unavailable.
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("requested an unavailable diffuse texture image"));
}

/* Generally, the error case names are self documenting. */

TEST_F(RenderMeshTest, ErrorNonExistentFile) {
  // The *surest* way to guarantee that tiny obj reports an invalid parse is to
  // give it a non-existent file to parse. There are other circumstances, we
  // won't worry about them. This is sufficient proof to show that failure to
  // parse is handled.
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadMeshFromObj("__garbage_doesn't_exist__.obj", diagnostic_policy_),
      "Failed parsing the obj file[^]*");
}

TEST_F(RenderMeshTest, ErrorNoFaces) {
  // N.B. tinyobj doesn't validate the OBJ spec. It may read an entire non-OBJ
  // file and not complain (it only cries if it sees something it considers is
  // an OBJ artifact that is malformed). So, a garbage file will likewise get
  // the same "no faces" error.
  const fs::path obj_path1 = WriteFile("v 0 0 0\nv 1 1 1\n", "no_faces.obj");
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadMeshFromObj(obj_path1.string(), diagnostic_policy_),
      "OBJ has no faces.*");

  const fs::path obj_path2 = WriteFile("Generic\nContent\n", "not_an.obj");
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadMeshFromObj(obj_path2.string(), diagnostic_policy_),
      "OBJ has no faces.*");
}

TEST_F(RenderMeshTest, ErrorNoNormals) {
  const fs::path obj_path =
      WriteFile("v 0 0 0\nv 1 1 1\nv 2 2 2\n f 1 2 3\n", "no_normals.obj");
  DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(obj_path, diagnostic_policy_),
                              "OBJ has no normals.*");
}

TEST_F(RenderMeshTest, ErrorFacesMissingNormals) {
  const fs::path obj_path = WriteFile(R"""(
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1
        f 1 2 3)""",
                                      "faces_missing_normals.obj");
  DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(obj_path, diagnostic_policy_),
                              "Not all faces reference normals.*");
}

TEST_F(RenderMeshTest, ErrorFacesMissingUvs) {
  const fs::path obj_path = WriteFile(R"""(
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 0
        f 1/1/1 2/1/1 3/1/1
        f 1//1 2//1 3//1)""",
                                      "faces_missing_uvs.obj");
  DRAKE_EXPECT_THROWS_MESSAGE(LoadMeshFromObj(obj_path, diagnostic_policy_),
                              "Not all faces reference texture coord.*");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
