#include "drake/geometry/render/render_mesh.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;

namespace fs = std::filesystem;

/* The tests for the function LoadRenderMeshesFromObj are partitioned into two
 broad categories indicated by prefix:

   - Geometry: evaluation of the geometry produced.
     - The tests confirm that the data gets processed as expected, including
       expected error conditions.
   - Material: evaluation of the material generation logic.
     - LoadRenderMeshesFromObj is responsible for defining a material from a
       material library where possible and, failing that, creating a fallback
       material using the RenderMaterial functions.
     - There are many ways in which the library has problems, but we still get
       a library-based material. These tests are prefixed as MaterialLibrary.
     - Where the material library can't be used, we resort to the material
       fallback logic. These are prefixed as MaterialFallback. */
class LoadRenderMeshFromObjTest : public test::DiagnosticPolicyTestBase {
 protected:
  static fs::path WriteFile(std::string_view contents,
                            std::string_view file_name,
                            const fs::path& dir = temp_dir()) {
    const fs::path file_path = dir / file_name;
    std::ofstream out(file_path);
    DRAKE_DEMAND(out.is_open());
    out << contents;
    return file_path;
  }

  static void SetUpTestSuite() {
    // Make sure we have a valid texture in the same directory as the mtl file
    // that names it.
    fs::path image_source_path(
        FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png"));
    fs::path image_target_path = temp_dir() / "diag_gradient.png";
    fs::copy(image_source_path, image_target_path);

    WriteFile(R"""(
    newmtl test_material
    Ka 0.1 0.1 0.1
    Kd 0.5 1.0 1.0
    Ks 1.0 1.0 1.0
    d 1
    illum 2
    Ns 0
  )""",
              basic_mtl);

    WriteFile(R"""(
    newmtl test_material_1
    Kd 1.0 0.0 1.0

    newmtl test_material_2
    Kd 1.0 0.0 0.0
  )""",
              multi_mtl);

    WriteFile(R"""(
    newmtl test_material
    Ka 0.1 0.1 0.1
    Kd 1.0 1.0 0.0
    Ks 1.0 1.0 1.0
    d 1
    illum 2
    Ns 0
    map_Kd diag_gradient.png
  )""",
              texture_mtl);

    WriteFile(R"""(
    newmtl test_material
    map_Kd diag_gradient.png
  )""",
              texture_only_mtl);
  }

  static fs::path temp_dir() {
    static const never_destroyed<fs::path> temp_path(temp_directory());
    return temp_path.access();
  }

  /* Specifies no material properties at all. */
  static PerceptionProperties empty_props() { return {}; }

  static constexpr char basic_mtl[] = "basic.mtl";
  static constexpr char multi_mtl[] = "multi.mtl";
  static constexpr char texture_mtl[] = "texture.mtl";
  static constexpr char texture_only_mtl[] = "texture_only.mtl";
};

static_assert(
    std::is_trivially_destructible_v<Rgba>,
    "If this fails, kDefaultDiffuse needs to become a never-destroyed static.");
const Rgba kDefaultDiffuse(0.75, 0.5, 0.25, 0.125);

TEST_F(LoadRenderMeshFromObjTest, GeometryErrorModes) {
  int error = -1;
  {
    // Case: file can't be parsed. The simplest way to achieve a invalid parse
    // is simply for the file to not exist. Any other conditions that tinyobj
    // considers a parse failure will get treated the same.
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshesFromObj(fs::path("__garbage_doesn't_exist__.obj"),
                                empty_props(), kDefaultDiffuse,
                                diagnostic_policy_),
        "Failed parsing obj data[^]*");
  }
  {
    // Case: Vertices only reports no faces found.
    const fs::path obj_path =
        WriteFile("v 1 2 3", fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshesFromObj(obj_path, empty_props(), kDefaultDiffuse,
                                diagnostic_policy_),
        "The OBJ data appears to have no faces.*");
  }
  {
    // Case: Not an obj in any way reports as no faces.
    const fs::path obj_path =
        WriteFile("Not an obj\njust some\nmeaningles text.\n",
                  fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshesFromObj(obj_path, empty_props(), kDefaultDiffuse,
                                diagnostic_policy_),
        "The OBJ data appears to have no faces.* might not be an OBJ file.+");
  }
  {
    // Case: The obj has no normals.
    const fs::path obj_path =
        WriteFile("v 1 2 3\nf 1 1 1\n", fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshesFromObj(obj_path, empty_props(), kDefaultDiffuse,
                                diagnostic_policy_),
        "OBJ has no normals.+");
  }
  {
    // Case: Not all faces reference normals.
    const fs::path obj_path = WriteFile(R"""(
v 1 2 3
vn 0 0 1
vt 0 1
f 1 1 1
f 1//1 1//1 1//1
)""",
                                        fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshesFromObj(obj_path, empty_props(), kDefaultDiffuse,
                                diagnostic_policy_),
        "Not all faces reference normals.+");
  }
}

/* Confirms that non-triangular faces get triangulated. This also confirms that
 duplication occurs due to associations of vertex positions with multiple
 normals or texture coordinates. */
TEST_F(LoadRenderMeshFromObjTest, GeometryTriangulatePolygons) {
  /*
             o 4
            ╱  ╲            A five-sided polygon and a four-sided polygon
           ╱    ╲           should be triangulated into three and two
          ╱      ╲          triangles respectively. But with the same
         ╱        ╲         vertices.
        ╱    f0    ╲
     5 o            o 3     The upper and lower polygons have *different*
        ╲          ╱        normals. So, vertices 1 & 2 will be copied.
         ╲        ╱         All vertices have the same texture coordinate, so
       1  o──────o 2        its value does not trigger duplication.
          │  f1  │
          o──────o
          6      7
  */
  constexpr char positions[] = R"""(
  v -1 -1 0
  v 1 -1 0
  v 2 1 0
  v 0 2 0
  v -2 1 0
  v -1 -2 0
  v 1 -2 0)""";

  const Vector3d unit3_z = Vector3d::UnitZ();
  const Vector3d unit3_y = Vector3d::UnitY();
  const Vector2d zero2 = Vector2d::Zero();
  const Vector2d ones2(1, 1);

  // Specification of test case
  struct TriangluateTestParmas {
    std::string name;
    std::string obj_spec;
    int position_count;
    // The expected normal and uv for the vertices in face 0 (f0).
    Vector3d n0;
    Vector2d uv0;
    // The expected normal and uv for the vertices in face 1 (f1).
    Vector3d n1;
    Vector2d uv1;
  };

  vector<TriangluateTestParmas> test_params{
      // Every vertex is associated with a single normal and texture coordinate.
      {"unique_vertices", R"""(
      vn 0 0 1
      vt 0 0
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/1/1 7/1/1 2/1/1 1/1/1)""",
       7, unit3_z, zero2, unit3_z, zero2},
      // Upper and lower faces have different normals; vertices 1 & 2 will be
      // duplicated.
      {"vertices_with_multiple_normals", R"""(
      vn 0 0 1
      vn 0 1 0
      vt 0 0
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/1/2 7/1/2 2/1/2 1/1/2)""",
       9, unit3_z, zero2, unit3_y, zero2},
      // Upper and lower faces have different uvs; vertices 1 & 2 will be
      // duplicated.
      {"vertices_with_multiple_uvs", R"""(
      vn 0 0 1
      vt 0 0
      vt 1 1
      f 1/1/1 2/1/1 3/1/1 4/1/1 5/1/1
      f 6/2/1 7/2/1 2/2/1 1/2/1)""",
       9, unit3_z, zero2, unit3_z, ones2},
  };

  for (const auto& params : test_params) {
    SCOPED_TRACE(params.name);
    const fs::path obj_path = WriteFile(
        fmt::format("{}{}", positions, params.obj_spec), params.name + ".obj");
    const RenderMesh mesh_data = LoadRenderMeshesFromObj(
        obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_)[0];
    EXPECT_EQ(mesh_data.positions.rows(), params.position_count);
    EXPECT_EQ(mesh_data.normals.rows(), params.position_count);
    EXPECT_EQ(mesh_data.uvs.rows(), params.position_count);
    // The two faces will always be triangulated into five triangles.
    EXPECT_EQ(mesh_data.indices.rows(), 5);

    // The first three triangles come from f0, and all have the expected normal
    // and texture coordinate.
    for (int t = 0; t < 3; ++t) {
      for (int i = 0; i < 3; ++i) {
        const int index = mesh_data.indices(t, i);
        EXPECT_TRUE(CompareMatrices(mesh_data.normals.row(index),
                                    params.n0.transpose()));
        EXPECT_TRUE(
            CompareMatrices(mesh_data.uvs.row(index), params.uv0.transpose()));
      }
    }
    // The last two triangles come from f1 and all have the expected normal and
    // texture coordinate.
    for (int t = 3; t < 5; ++t) {
      for (int i = 0; i < 3; ++i) {
        const int index = mesh_data.indices(t, i);
        EXPECT_TRUE(CompareMatrices(mesh_data.normals.row(index),
                                    params.n1.transpose()));
        EXPECT_TRUE(
            CompareMatrices(mesh_data.uvs.row(index), params.uv1.transpose()));
      }
    }
  }
}

/* Geometry already triangulated gets preserved. This doesn't do variations on
 whether vertex positions need copying due to associations with multiple
 normals/uvs. It relies on `TriangulatePolygons` to handle that. */
TEST_F(LoadRenderMeshFromObjTest, GeometryPreserveTriangulation) {
  /*
             o 4
            ╱  ╲
           ╱    ╲
          ╱      ╲
         ╱        ╲
        ╱          ╲       Note: Ascii art makes drawing the following edges
     5 o────────────o 3    nearly impossible.
        ╲          ╱
         ╲        ╱        Connect 2 to 5 to form two triangles.
       1  o──────o 2
          │      │         Connect 1 to 7 to form two triangles.
          o──────o
          6      7
  */
  const fs::path obj_path = WriteFile(R"""(
  v -1 -1 0
  v 1 -1 0
  v 2 1 0
  v 0 2 0
  v -2 1 0
  v -1 -2 0
  v 1 -2 0
  vn 0 0 1
  vt 0 0
  f 1/1/1 2/1/1 5/1/1
  f 2/1/1 3/1/1 5/1/1
  f 3/1/1 4/1/1 5/1/1
  f 1/1/1 6/1/1 7/1/1
  f 1/1/1 7/1/1 2/1/1
  )""",
                                      "preserve_triangulation.obj");

  const RenderMesh mesh_data = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_)[0];
  EXPECT_EQ(mesh_data.positions.rows(), 7);
  EXPECT_EQ(mesh_data.normals.rows(), 7);
  EXPECT_EQ(mesh_data.uvs.rows(), 7);
  EXPECT_EQ(mesh_data.indices.rows(), 5);
}

/* The RenderMesh produces *new* vertices from what was in the OBJ based on what
 vertex gets referenced by which faces. A vertex that doesn't get referenced
 gets omitted. */
TEST_F(LoadRenderMeshFromObjTest, GeometryRemoveUnreferencedVertices) {
  /*

        4 o───────o 3
          │     ╱ │
          │   ╱   │   o 5
          │ ╱     │
    1, 6  o───────o 2

  */
  const fs::path obj_path = WriteFile(R"""(
  v -1 -1 0  # First four corners form a box around the origin.
  v 1 -1 0
  v 1 1 0
  v -1 1 0
  v 3 0 0    # Unreferenced vertex to the right of the box (omitted).
  v -1 -1 0  # Duplicate of vertex 1 but propagated through.
  vn 0 0 1
  vt 0 0
  f 1/1/1 2/1/1 3/1/1
  f 6/1/1 3/1/1 4/1/1
  )""",
                                      "unreferenced_vertices.obj");
  const RenderMesh mesh_data = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_)[0];
  EXPECT_EQ(mesh_data.positions.rows(), 5);
  EXPECT_EQ(mesh_data.normals.rows(), 5);
  EXPECT_EQ(mesh_data.uvs.rows(), 5);
  EXPECT_EQ(mesh_data.indices.rows(), 2);
}

/* The OBJ has a single intrinsic material which only defines diffuse color.
 Only a single mesh is created and its material is the defined color. */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryColorOnly) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material
        f 1//1 2//1 3//1)""",
                                                  basic_mtl),
                                      "intrinsic_color_mat.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(0.5, 1, 1));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
}

/* The OBJ has a single intrinsic material which only defines diffuse texture.
 The texture is in the same directory as the .mtl file. Only a single mesh is
 created and its material has a white diffuse color and the named texture. */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryTextureOnly) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material
        f 1/1/1 2/1/1 3/1/1)""",
                                                  texture_only_mtl),
                                      "intrinsic_texture_mat.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 1));
  ASSERT_TRUE(std::holds_alternative<fs::path>(mesh.material->diffuse_map));
  EXPECT_THAT(std::get<fs::path>(mesh.material->diffuse_map).string(),
              testing::EndsWith("diag_gradient.png"));
}

/* The OBJ has a single intrinsic material which defines color and texture. Only
 a single mesh is created and its material includes both color and texture. */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryColorAndTexture) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material
        f 1/1/1 2/1/1 3/1/1)""",
                                                  texture_mtl),
                                      "intrinsic_full_diffuse_mat.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 0));
  ASSERT_TRUE(std::holds_alternative<fs::path>(mesh.material->diffuse_map));
  EXPECT_THAT(std::get<fs::path>(mesh.material->diffuse_map).string(),
              testing::EndsWith("diag_gradient.png"));
}

/* The OBJ has a single intrinsic material which defines a diffuse texture.
 The texture is in a *different* directory from the mtl file. But the mtl must
 be in the same directory as the obj. (See notes in implementation.)  Only a
 single mesh is created and its material has a *white* diffuse color and the
 named texture. */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryDislocatedTexture) {
  const fs::path obj_dir = temp_dir() / "dis_texture";
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
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 1));
  ASSERT_TRUE(std::holds_alternative<fs::path>(mesh.material->diffuse_map));
  EXPECT_EQ(std::get<fs::path>(mesh.material->diffuse_map),
            (temp_dir() / "diag_gradient.png"));
}

/* The OBJ has multiple intrinsic materials *defined* in the .mtl file. But only
 one is applied. That counts as a single intrinsic material. */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryMultipleDefinedIntrinsic) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material_1
        f 1/1/1 2/1/1 3/1/1)""",
                                                  multi_mtl),
                                      "use_only_one_material.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 0, 1));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
}

/* The OBJ has a single intrinsic material which defines a bad texture. The
 resulting material has *only* the color value. */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryColorAndBadTexture) {
  WriteFile("newmtl test_material\nKd 1 0.5 1\nmap_Kd cant_be_found.png\n",
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
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 0.5, 1));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("requested an unavailable diffuse texture image"));
}

/* The OBJ has a single intrinsic material with a texture but no uvs. The
 resulting material has *only* the color value (and a warning is dispatched). */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryTextureButNoUvs) {
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
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex(".*requested a diffuse texture.*doesn't "
                                    "define any texture coordinates.*"));
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 0));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
}

/* The OBJ has a single intrinsic material with a texture but no uvs. The
 resulting material has *only* the color value (and a warning is dispatched). */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryTextureButPartialUvs) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material
        f 1/1/1 2/1/1 3//1)""",  // Note: final vertex has no uv index.
                                                  texture_mtl),
                                      "partial_uvs_with_texture.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  EXPECT_THAT(
      TakeWarning(),
      testing::MatchesRegex(".*requested a diffuse texture.*doesn't define a "
                            "complete set of texture coordinates.*"));
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 0));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
}

/* This test merely exposes a known flaw in the code. Images are defined
 relative to the mtl file, but we don't know the mtl file that got read. So,
 we *assume* mtl and obj are in the same place. If not, it can erroneously lead
 to not being able to locate used textures. The victory condition is that when
 we fix this flaw, this test fails and we reverse the semantics: a valid
 material is included in the resulting mesh data. */
TEST_F(LoadRenderMeshFromObjTest, MaterialLibraryObjMtlDislocation) {
  const fs::path obj_dir = temp_dir() / "relative_obj";
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
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 0));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  // Because of our limited access to parsing information, this will look like
  // the image is unavailable.
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("requested an unavailable diffuse texture image"));
}

/* The OBJ has multiple intrinsic materials *applied* to the mesh. It leads to
 multiple RenderMesh instances with the various materials. We'll confirm the
 partitioning of the geometry and the resulting materials. This test provides
 token coverage of the partitioning -- it only considers the number of triangles
 in each partition -- real bugs would be obvious in the rendered results. */
TEST_F(LoadRenderMeshFromObjTest, MultipleValidIntrinsicMaterials) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material_1
        f 1//1 2//1 3//1
        usemtl test_material_2
        f 2//1 3//1 1//1
        f 3//1 1//1 2//1)""",  // Distinguish meshes by triangle count.
                                                  multi_mtl),
                                      "too_many_usemtl.obj");
  vector<RenderMesh> meshes = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(meshes.size(), 2);
  for (const auto& mesh : meshes) {
    ASSERT_TRUE(mesh.material.has_value());
    if (mesh.indices.rows() == 1) {
      // test_material_1 applied to a single triangle.
      EXPECT_EQ(mesh.material->diffuse, Rgba(1, 0, 1));
      EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
    } else if (mesh.indices.rows() == 2) {
      // test_material_2 applied to two triangles.
      EXPECT_EQ(mesh.material->diffuse, Rgba(1, 0, 0));
      EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
    } else {
      DRAKE_UNREACHABLE();
    }
  }
}

/* If the obj references the mtl file with an absolute path, there will be a
 warning and default material (due to tiny obj logic). */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackObjMtlDislocationAbsolute) {
  // Note: This is an absolute path that we imbed in the obj. This makes tinyobj
  // angry.
  const fs::path mtl_path = temp_dir() / texture_mtl;
  // This will be a *different* temp directory than for the test suite.
  const fs::path obj_dir(temp_directory());
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        usemtl test_material
        f 1/1/1 2/1/1 3/1/1)""",
                                                  mtl_path.string()),
                                      "mtl_elsewhere.obj", obj_dir);
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  // The tinyobj warning that the material library can't be found.
  EXPECT_THAT(TakeWarning(), testing::HasSubstr("not found in a path"));
}

/* The obj explicitly applies a material to *some* of the faces. This is
 implicitly two materials (the default material being applied to the otherwise
 unspecified faces). Two meshes are created. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackDefaultedFaces) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1
        usemtl test_material
        f 1//1 2//1 3//1
        f 1//1 2//1 3//1)""",  // Distinguish meshes by triangle count.
                                                  basic_mtl),
                                      "default_material_faces.obj");
  vector<RenderMesh> meshes = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(meshes.size(), 2);
  for (const auto& mesh : meshes) {
    ASSERT_TRUE(mesh.material.has_value());
    if (mesh.indices.rows() == 1) {
      // default material applied to a single triangle.
      EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
      EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
    } else if (mesh.indices.rows() == 2) {
      // test_material applied to two triangles.
      EXPECT_EQ(mesh.material->diffuse, Rgba(0.5, 1, 1));
      EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
    } else {
      DRAKE_UNREACHABLE();
    }
  }
}

/* If the obj references an invalid material name, there will be a warning and
 a fallback material. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackBadMaterialName) {
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
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  EXPECT_THAT(TakeWarning(),
              testing::HasSubstr("material [ 'bad_material' ] not found"));
}

/* If we can't find the specified mtl file, there will be warning and fallback
 material. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackMissingMtl) {
  const fs::path obj_path = WriteFile(R"""(
        mtllib not_really_a.mtl
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material
        f 1//1 2//1 3//1)""",
                                      "non_existent_mtl.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("Material file [ not_really_a.mtl ] not found"));
}

/* The OBJ has no materials applied. The material should be the fallback
 material. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackNoAppliedMaterial) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1)""",
                                                  basic_mtl),
                                      "no_usemtl.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
}

/* The OBJ references no material library. The material should be the fallback
 material. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackNoMaterialLibrary) {
  const fs::path obj_path = WriteFile(R"""(
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1)""",
                                      "no_mtllib.obj");
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
}

/* The OBJ references no material library. The material should be the fallback
 material. If a diffuse color is specified through the properties, then it is
 used to make a material. However, with no property specified *and* no default
 diffuse value, the parsed RenderMesh doesn't have any material. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackWithNoDefaultDiffuse) {
  const fs::path obj_path = WriteFile(R"""(
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1)""",
                                      "no_mtllib.obj");
  Rgba diffuse(0.1, 0.2, 0.3, 0.4);
  PerceptionProperties props;
  props.AddProperty("phong", "diffuse", diffuse);
  vector<RenderMesh> result_with_material = LoadRenderMeshesFromObj(
      obj_path, props, std::nullopt, diagnostic_policy_);
  ASSERT_EQ(result_with_material.size(), 1);
  EXPECT_TRUE(result_with_material[0].material.has_value());

  vector<RenderMesh> result_without_material = LoadRenderMeshesFromObj(
      obj_path, empty_props(), std::nullopt, diagnostic_policy_);
  ASSERT_EQ(result_without_material.size(), 1);
  EXPECT_FALSE(result_without_material[0].material.has_value());
}

/* If the mesh has a material library and material properties are *also*
 defined, the material library should win but we *should* have warnings on
 the material properties. There are multiple cases in which it would throw
 (see tests in render_material.cc), we just need a simple indication that it
 is being exercised. A single warning will provide sufficient evidence. */
TEST_F(LoadRenderMeshFromObjTest, RedundantMaterialWarnings) {
  // We just need an obj with *any* intrinsic material; we'll do color only.
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        usemtl test_material
        f 1//1 2//1 3//1)""",
                                                  basic_mtl),
                                      "intrinsic_vs_props.obj");

  PerceptionProperties props;
  props.AddProperty("phong", "diffuse", Rgba(0.1, 0.2, 0.3, 0.4));
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, props, kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  // We still get the intrinsic material.
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(0.5, 1, 1));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  EXPECT_THAT(
      TakeWarning(),
      testing::MatchesRegex(".*has its own materials.*'phong', 'diffuse'.*"));
}

/* We need evidence that the uv state is passed to the fallback material. So,
 we'll create an obj with partial UVs and a valid ("phong", "diffuse_map"). We
 should get the RenderMaterial warning. */
TEST_F(LoadRenderMeshFromObjTest, UvStatePassedToFallback) {
  // We just need an obj with *any* intrinsic material; we'll do color only.
  const fs::path obj_path = WriteFile(R"""(
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        vt 0 1
        f 1/1/1 2/1/1 3//1)""",
                                      "partial_uvs_fallback_texture.obj");

  PerceptionProperties props;
  const std::string tex_name =
      FindResourceOrThrow("drake/geometry/render/test/diag_gradient.png");
  props.AddProperty("phong", "diffuse_map", tex_name);
  vector<RenderMesh> result = LoadRenderMeshesFromObj(
      obj_path, props, kDefaultDiffuse, diagnostic_policy_);
  ASSERT_EQ(result.size(), 1);
  const RenderMesh& mesh = result[0];
  // Inability to apply a valid texture leaves the material flat white.
  ASSERT_TRUE(mesh.material.has_value());
  EXPECT_EQ(mesh.material->diffuse, Rgba(1, 1, 1));
  EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex(
                  ".*'diffuse_map'.* doesn't define a complete set of.*"));
}

/* Tests if the `from_mesh_file` flag is correctly propagated. */
TEST_F(LoadRenderMeshFromObjTest, PropagateFromMeshFileFlag) {
  for (const bool from_mesh_file : {false, true}) {
    // N.B. box_no_mtl.obj doesn't exist in the mesh_source tree and is
    // generated from box.obj by stripping out material data by the build
    // system.
    const fs::path filename =
        from_mesh_file
            ? FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj")
            : FindResourceOrThrow(
                  "drake/geometry/render/test/meshes/box_no_mtl.obj");

    const RenderMesh mesh_data = LoadRenderMeshesFromObj(
        filename, empty_props(), kDefaultDiffuse, diagnostic_policy_)[0];
    ASSERT_TRUE(mesh_data.material.has_value());
    EXPECT_EQ(from_mesh_file, mesh_data.material->from_mesh_file);
  }
}

/* Tests parsing of in-memory obj. This isn't exhaustive. This largely tests the
 use of supporting files. What happens when a supporting file is referenced but
 is or isn't present in the set of supporting files. The rest of the parsing
 is assumed to be identical to the on-disk parsing. */
TEST_F(LoadRenderMeshFromObjTest, InMemoryMesh) {
  const PerceptionProperties props;

  /* Obj references no mtl file. */
  {
    const MeshSource mesh_source(InMemoryMesh{MemoryFile(
        R"""(v 0 0 0
             v 1 0 0
             v 0 1 0
             vn 0 0 1
             f 1//1 2//1 3//1
          )""",
        ".obj", "obj1")});
    const RenderMesh mesh = LoadRenderMeshesFromObj(
        mesh_source, props, kDefaultDiffuse, diagnostic_policy_)[0];
    EXPECT_EQ(mesh.positions.rows(), 3);
    EXPECT_EQ(mesh.normals.rows(), 3);
    EXPECT_EQ(mesh.uvs.rows(), 3);
    EXPECT_EQ(mesh.indices.rows(), 1);
    ASSERT_TRUE(mesh.material.has_value());
    EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
    EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  }

  /* Obj references available mtl, no textures. */
  {
    const MeshSource mesh_source(
        InMemoryMesh{MemoryFile(
                         R"""(mtllib mem.mtl
                              v 0 0 0
                              v 1 0 0
                              v 0 1 0
                              vn 0 0 1
                              usemtl test
                              f 1//1 2//1 3//1
                            )""",
                         ".obj", "obj2"),
                     {{"mem.mtl", MemoryFile(
                                      R"""(newmtl test
                                           Kd 1 0 0)""",
                                      ".mtl", "mem.mtl")}}});
    const RenderMesh mesh = LoadRenderMeshesFromObj(
        mesh_source, props, kDefaultDiffuse, diagnostic_policy_)[0];
    EXPECT_EQ(mesh.positions.rows(), 3);
    EXPECT_EQ(mesh.normals.rows(), 3);
    EXPECT_EQ(mesh.uvs.rows(), 3);
    EXPECT_EQ(mesh.indices.rows(), 1);
    ASSERT_TRUE(mesh.material.has_value());
    EXPECT_EQ(mesh.material->diffuse, Rgba(1, 0, 0));
    EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
  }

  /* Obj references available in-memory mtl, with available in-memory image. */
  {
    const MeshSource mesh_source(
        InMemoryMesh{MemoryFile(
                         R"""(mtllib mem.mtl
                              v 0 0 0
                              v 1 0 0
                              v 0 1 0
                              vn 0 0 1
                              usemtl test
                              f 1//1 2//1 3//1
                            )""",
                         ".obj", "obj3"),
                     {{"mem.mtl", MemoryFile(
                                      R"""(newmtl test
                                           map_Kd fake.png)""",
                                      ".mtl", "mem.mtl")},
                      {"fake.png", MemoryFile("abc", ".png", "png")}}});
    const RenderMesh mesh = LoadRenderMeshesFromObj(
        mesh_source, props, kDefaultDiffuse, diagnostic_policy_)[0];
    EXPECT_EQ(mesh.positions.rows(), 3);
    EXPECT_EQ(mesh.normals.rows(), 3);
    EXPECT_EQ(mesh.uvs.rows(), 3);
    EXPECT_EQ(mesh.indices.rows(), 1);
    ASSERT_TRUE(mesh.material.has_value());
    EXPECT_TRUE(std::holds_alternative<MemoryFile>(mesh.material->diffuse_map));
  }

  /* Obj references available on-disk mtl, with available in-memory image. */
  {
    /* We reference an existing .mtl file and use the material name and texture
     names found in that file (box.obj.mtl). */
    const MeshSource mesh_source(InMemoryMesh{
        MemoryFile(
            R"""(mtllib mem.mtl
                 v 0 0 0
                 v 1 0 0
                 v 0 1 0
                 vn 0 0 1
                 usemtl material_0
                 f 1//1 2//1 3//1
               )""",
            ".obj", "obj3"),
        {{"mem.mtl", fs::path(FindResourceOrThrow(
                         "drake/geometry/render/test/meshes/box.obj.mtl"))},
         {"box.png", MemoryFile("abc", ".png", "png")}}});
    const RenderMesh mesh = LoadRenderMeshesFromObj(
        mesh_source, props, kDefaultDiffuse, diagnostic_policy_)[0];
    EXPECT_EQ(mesh.positions.rows(), 3);
    EXPECT_EQ(mesh.normals.rows(), 3);
    EXPECT_EQ(mesh.uvs.rows(), 3);
    EXPECT_EQ(mesh.indices.rows(), 1);
    ASSERT_TRUE(mesh.material.has_value());
    EXPECT_TRUE(std::holds_alternative<MemoryFile>(mesh.material->diffuse_map));
  }

  /* Obj references unavailable mtl. */
  {
    ASSERT_EQ(this->NumWarnings(), 0);
    ASSERT_EQ(this->NumErrors(), 0);
    const MeshSource mesh_source(InMemoryMesh{MemoryFile(
        R"""(mtllib missing.mtl
             v 0 0 0
             v 1 0 0
             v 0 1 0
             vn 0 0 1
             f 1//1 2//1 3//1
           )""",
        ".obj", "obj4")});
    const RenderMesh mesh = LoadRenderMeshesFromObj(
        mesh_source, props, kDefaultDiffuse, diagnostic_policy_)[0];
    EXPECT_EQ(mesh.positions.rows(), 3);
    EXPECT_EQ(mesh.normals.rows(), 3);
    EXPECT_EQ(mesh.uvs.rows(), 3);
    EXPECT_EQ(mesh.indices.rows(), 1);
    ASSERT_TRUE(mesh.material.has_value());
    EXPECT_EQ(mesh.material->diffuse, kDefaultDiffuse);
    EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
    ASSERT_EQ(this->NumWarnings(), 2);  // One from Drake, one from tinyobj.
    EXPECT_THAT(this->TakeWarning(),
                testing::HasSubstr("not in its supporting files"));
    EXPECT_THAT(this->TakeWarning(),
                testing::HasSubstr("Failed to load material file"));
  }

  /* Obj references unavailable texture image. */
  {
    ASSERT_EQ(this->NumWarnings(), 0);
    ASSERT_EQ(this->NumErrors(), 0);
    const MeshSource mesh_source(
        InMemoryMesh{MemoryFile(
                         R"""(mtllib mem.mtl
                              v 0 0 0
                              v 1 0 0
                              v 0 1 0
                              vn 0 0 1
                              usemtl test
                              f 1//1 2//1 3//1
                            )""",
                         ".obj", "obj5"),
                     {{"mem.mtl", MemoryFile(R"""(newmtl test
                                                  map_Kd fake.png)""",
                                             ".mtl", "mem.mtl")}}});
    const RenderMesh mesh = LoadRenderMeshesFromObj(
        mesh_source, props, kDefaultDiffuse, diagnostic_policy_)[0];
    EXPECT_EQ(mesh.positions.rows(), 3);
    EXPECT_EQ(mesh.normals.rows(), 3);
    EXPECT_EQ(mesh.uvs.rows(), 3);
    EXPECT_EQ(mesh.indices.rows(), 1);
    ASSERT_TRUE(mesh.material.has_value());
    EXPECT_TRUE(IsEmpty(mesh.material->diffuse_map));
    ASSERT_EQ(this->NumWarnings(), 1);
    EXPECT_THAT(this->TakeWarning(),
                testing::HasSubstr("image will be omitted"));
  }
}

GTEST_TEST(MakeRenderMeshFromTriangleSurfaceMeshTest, SingleTriangle) {
  std::vector<Vector3d> vertices;
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  std::vector<SurfaceTriangle> triangles;
  triangles.emplace_back(0, 1, 2);
  const TriangleSurfaceMesh tri_mesh(std::move(triangles), std::move(vertices));
  const PerceptionProperties empty_props;
  RenderMesh render_mesh =
      MakeRenderMeshFromTriangleSurfaceMesh(tri_mesh, empty_props);

  // Check geometry.
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> expected_positions(
      3, 3);
  // clang-format off
  expected_positions << 0.0, 0.0, 0.0,
                        1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0;
  // clang-format on
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> expected_normals(3,
                                                                             3);
  // All vertex normals inherit face normal of the only face.
  // clang-format off
  expected_normals << 0.0, 0.0, 1.0,
                      0.0, 0.0, 1.0,
                      0.0, 0.0, 1.0;
  // clang-format on
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> expected_uvs(3, 2);
  // All uvs are set to (0,0).
  // clang-format off
  expected_uvs << 0.0, 0.0,
                  0.0, 0.0,
                  0.0, 0.0;
  // clang-format on
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 3, Eigen::RowMajor>
      expected_indices(1, 3);
  expected_indices << 0, 1, 2;
  EXPECT_EQ(render_mesh.positions, expected_positions);
  EXPECT_EQ(render_mesh.normals, expected_normals);
  EXPECT_EQ(render_mesh.uvs, expected_uvs);
  EXPECT_EQ(render_mesh.indices, expected_indices);

  EXPECT_EQ(render_mesh.uv_state, UvState::kNone);

  // Check material
  EXPECT_FALSE(render_mesh.material.has_value());
}

// Tests that the vertex normals are indeed computed using area weighted average
// of face normals.
GTEST_TEST(MakeRenderMeshFromTriangleSurfaceMeshTest, AreaWeightedNormals) {
  /* Make a triangle surface mesh that's the surface of a single tet.
                  +Fz   -Fx
                   |   /
                   v3 /
                   | /
                   |/
   -Fy-----------v0+------v2---+ Fy
                  /| Fo
                 / |
               v1  |
               /   |
             +Fx   |
                  -Fz
  */
  std::vector<Vector3d> vertices;
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  vertices.emplace_back(0, 0, 1);
  std::vector<SurfaceTriangle> triangles;
  triangles.emplace_back(0, 2, 1);  // Area 0.5, normal (0, 0, -1)
  triangles.emplace_back(0, 1, 3);  // Area 0.5, normal (0, -1, 0)
  triangles.emplace_back(0, 3, 2);  // Area 0.5, normal (-1, 0, 0)
  triangles.emplace_back(1, 2,
                         3);  // Area sqrt(3)/2, normal (1, 1, 1)/sqrt(3)
  const TriangleSurfaceMesh tri_mesh(std::move(triangles), std::move(vertices));
  const PerceptionProperties empty_props;
  const RenderMesh render_mesh =
      MakeRenderMeshFromTriangleSurfaceMesh(tri_mesh, empty_props);

  // Compare the computed normals with the pen-and-paper result. Note that this
  // tetrahedron has been selected so that the weighted normal for v0 points in
  // the (-1, -1, -1) direction, and v2, v3, and v4 have weighted normals that
  // point in the x, y, and z directions, respectively.
  EXPECT_TRUE(CompareMatrices(render_mesh.normals.row(0).transpose(),
                              Vector3d(-1.0, -1.0, -1.0).normalized()));
  EXPECT_TRUE(CompareMatrices(render_mesh.normals.row(1).transpose(),
                              Vector3d::UnitX()));
  EXPECT_TRUE(CompareMatrices(render_mesh.normals.row(2).transpose(),
                              Vector3d::UnitY()));
  EXPECT_TRUE(CompareMatrices(render_mesh.normals.row(3).transpose(),
                              Vector3d::UnitZ()));
}

GTEST_TEST(MakeRenderMeshFromTriangleSurfaceMeshTest, RoundTrip) {
  const fs::path filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");
  PerceptionProperties empty_props;
  const RenderMesh render_mesh =
      LoadRenderMeshesFromObj(filename, empty_props, kDefaultDiffuse)[0];
  const TriangleSurfaceMesh tri_mesh = MakeTriangleSurfaceMesh(render_mesh);
  const RenderMesh roundtrip_render_mesh =
      MakeRenderMeshFromTriangleSurfaceMesh(tri_mesh, empty_props);
  EXPECT_EQ(render_mesh.positions, roundtrip_render_mesh.positions);
  EXPECT_EQ(render_mesh.normals, roundtrip_render_mesh.normals);
  EXPECT_EQ(render_mesh.indices, roundtrip_render_mesh.indices);
  // uv, uv_state, and material are not preserved in the roundtrip.

  const TriangleSurfaceMesh roundtrip_tri_mesh =
      MakeTriangleSurfaceMesh(roundtrip_render_mesh);
  EXPECT_TRUE(roundtrip_tri_mesh.Equal(tri_mesh));
}

// Exploiting the fact that MakeFacetedRenderMeshFromTriangleSurfaceMesh()
// simply invokes MakeRenderMeshFromTriangleSurfaceMesh(), we just need to
// confirm that the result is as faceted as we expect.
GTEST_TEST(MakeFacetedRenderMeshFromTriangleSurfaceMeshTest, Simple) {
  /* Make a mesh with two triangles with a shared edge. The first face will
   have a normal in the -Fx direction, the second in the -Fy direction.
                +Fz   -Fx
                  |   /
                  v3 /
                  | /
                  |/
   -Fy-----------v0+------v2---+ Fy
                 /| Fo
                / |
              v1  |
              /   |
            +Fx   |
                -Fz                  */
  std::vector<Vector3d> vertices;
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  vertices.emplace_back(0, 0, 1);
  std::vector<SurfaceTriangle> triangles;
  triangles.emplace_back(0, 2, 3);  // normal (-1, 0, 0)
  triangles.emplace_back(0, 1, 3);  // normal (0, -1, 0)
  const TriangleSurfaceMesh tri_mesh(std::move(triangles), std::move(vertices));

  const RenderMesh render_mesh = MakeFacetedRenderMeshFromTriangleSurfaceMesh(
      tri_mesh, PerceptionProperties());

  ASSERT_EQ(render_mesh.indices.rows(), 2);
  // Three unique vertices per triangle.
  ASSERT_EQ(render_mesh.positions.rows(), 2 * 3);

  // Each triple of vertices constitute a single triangle. The normal implied
  // by the vertices should be shared for all vertices.
  for (int ti = 0; ti < render_mesh.indices.rows(); ++ti) {
    const auto tri = render_mesh.indices.row(ti);
    EXPECT_EQ(tri[0], ti * 3);
    EXPECT_EQ(tri[1], tri[0] + 1);
    EXPECT_EQ(tri[2], tri[0] + 2);

    const auto p_MA = render_mesh.positions.row(tri[0]);
    const auto p_MB = render_mesh.positions.row(tri[1]);
    const auto p_MC = render_mesh.positions.row(tri[2]);
    const Vector3d n_M = (p_MB - p_MA).cross(p_MC - p_MA).normalized();
    EXPECT_TRUE(
        CompareMatrices(Vector3d(render_mesh.normals.row(tri[0])), n_M));
    EXPECT_TRUE(
        CompareMatrices(Vector3d(render_mesh.normals.row(tri[1])), n_M));
    EXPECT_TRUE(
        CompareMatrices(Vector3d(render_mesh.normals.row(tri[2])), n_M));
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
