#include "drake/geometry/render/render_mesh.h"

#include <filesystem>
#include <fstream>
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

/* The tests for the function LoadRenderMeshFromObj are partitioned into two
 broad categories indicated by prefix:

   - Geometry: evaluation of the geometry produced.
     - The tests confirm that the data gets processed as expected, including
       expected error conditions.
   - Material: evaluation of the material generation logic.
     - LoadRenderMeshFromObj is responsible for defining a material from a
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
        LoadRenderMeshFromObj("__garbage_doesn't_exist__.obj", empty_props(),
                              kDefaultDiffuse, diagnostic_policy_),
        "Failed parsing the obj file[^]*");
  }
  {
    // Case: Vertices only reports no faces found.
    const fs::path obj_path =
        WriteFile("v 1 2 3", fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshFromObj(obj_path, empty_props(), kDefaultDiffuse,
                              diagnostic_policy_),
        "The OBJ data appears to have no faces.*");
  }
  {
    // Case: Not an obj in any way reports as no faces.
    const fs::path obj_path =
        WriteFile("Not an obj\njust some\nmeaningles text.\n",
                  fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshFromObj(obj_path, empty_props(), kDefaultDiffuse,
                              diagnostic_policy_),
        "The OBJ data appears to have no faces.* might not be an OBJ file.+");
  }
  {
    // Case: The obj has no normals.
    const fs::path obj_path =
        WriteFile("v 1 2 3\nf 1 1 1\n", fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshFromObj(obj_path, empty_props(), kDefaultDiffuse,
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
        LoadRenderMeshFromObj(obj_path, empty_props(), kDefaultDiffuse,
                              diagnostic_policy_),
        "Not all faces reference normals.+");
  }
  {
    // Case: Not all faces reference uvs.
    const fs::path obj_path = WriteFile(R"""(
v 1 2 3
vn 0 0 1
vt 0 0
f 1//1 1//1 1//1
)""",
                                        fmt::format("error{}.obj", ++error));
    DRAKE_EXPECT_THROWS_MESSAGE(
        LoadRenderMeshFromObj(obj_path, empty_props(), kDefaultDiffuse,
                              diagnostic_policy_),
        "Not all faces reference texture.+");
  }
}

// Confirms that non-triangular faces get triangulated. This also confirms that
// duplication occurs due to associations of vertex positions with multiple
// normals or texture coordinates.
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
    const RenderMesh mesh_data = LoadRenderMeshFromObj(
        obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
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
        EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(index),
                                    params.uv0.transpose()));
      }
    }
    // The last two triangles come from f1 and all have the expected normal and
    // texture coordinate.
    for (int t = 3; t < 5; ++t) {
      for (int i = 0; i < 3; ++i) {
        const int index = mesh_data.indices(t, i);
        EXPECT_TRUE(CompareMatrices(mesh_data.normals.row(index),
                                    params.n1.transpose()));
        EXPECT_TRUE(CompareMatrices(mesh_data.uvs.row(index),
                                    params.uv1.transpose()));
      }
    }
  }
}

// Geometry already triangulated gets preserved. This doesn't do variations on
// whether vertex positions need copying due to associations with multiple
// normals/uvs. It relies on `TriangulatePolygons` to handle that.
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
  )""", "preserve_triangulation.obj");

  const RenderMesh mesh_data = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh_data.positions.rows(), 7);
  EXPECT_EQ(mesh_data.normals.rows(), 7);
  EXPECT_EQ(mesh_data.uvs.rows(), 7);
  EXPECT_EQ(mesh_data.indices.rows(), 5);
}

// The RenderMesh produces *new* vertices from what was in the OBJ based on what
// vertex gets referenced by which faces. A vertex that doesn't get referenced
// gets omitted.
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
  )""", "unreferenced_vertices.obj");
  const RenderMesh mesh_data = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh_data.positions.rows(), 5);
  EXPECT_EQ(mesh_data.normals.rows(), 5);
  EXPECT_EQ(mesh_data.uvs.rows(), 5);
  EXPECT_EQ(mesh_data.indices.rows(), 2);
}

/* The OBJ has a single intrinsic material which only defines diffuse color. The
 resulting material is the defined color. */
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, Rgba(0.5, 1, 1));
  EXPECT_EQ(mesh.material.diffuse_map, "");
}

/* The OBJ has a single intrinsic material which only defines diffuse texture.
 The texture is in the same directory as the .mtl file. The resulting material
 has a white diffuse color and the named texture. */
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, Rgba(1, 1, 1));
  EXPECT_THAT(mesh.material.diffuse_map,
              testing::EndsWith("diag_gradient.png"));
}

/* The OBJ has a single intrinsic material which defines color and texture. The
 resulting material includes both color and texture. */
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, Rgba(1, 1, 0));
  EXPECT_THAT(mesh.material.diffuse_map,
              testing::EndsWith("diag_gradient.png"));
}

/* The OBJ has a single intrinsic material which defines a diffuse texture.
 The texture is in a *different* directory from the mtl file. But the mtl must
 be in the same directory as the obj. (See notes in implementation.)
 The resulting material has a *white* diffuse color and the named texture. */
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, Rgba(1, 1, 1));
  EXPECT_EQ(mesh.material.diffuse_map,
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, Rgba(1, 0, 1));
  EXPECT_EQ(mesh.material.diffuse_map, "");
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, Rgba(1, 0.5, 1));
  EXPECT_EQ(mesh.material.diffuse_map, "");
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex(".*requested a diffuse texture.*doesn't "
                                    "define texture coordinates.*"));
  EXPECT_EQ(mesh.material.diffuse, Rgba(1, 1, 0));
  EXPECT_EQ(mesh.material.diffuse_map, "");
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, Rgba(1, 1, 0));
  EXPECT_EQ(mesh.material.diffuse_map, "");
  // Because of our limited access to parsing information, this will look like
  // the image is unavailable.
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("requested an unavailable diffuse texture image"));
}

/* The OBJ has multiple intrinsic materials *applied* to the mesh. The material
 should be the fallback material and should be accompanied by a warning. When
 we expand material support, this test can and should change. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackMultipleAppliedIntrinsic) {
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, kDefaultDiffuse);
  EXPECT_EQ(mesh.material.diffuse_map, "");
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("use a single material across the whole mesh"));
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, kDefaultDiffuse);
  EXPECT_EQ(mesh.material.diffuse_map, "");
  // The tinyobj warning that the material library can't be found.
  EXPECT_THAT(TakeWarning(), testing::HasSubstr("not found in a path"));
}

/* The obj explicitly applies a material to *some* of the faces. The material
 gets defaulted and a warning given. */
TEST_F(LoadRenderMeshFromObjTest, MaterialFallbackDefaultedFaces) {
  const fs::path obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 1 1 1
        v 2 2 2
        vn 0 1 0
        f 1//1 2//1 3//1
        usemtl test_material
        f 1//1 2//1 3//1)""",
                                                  basic_mtl),
                                      "default_material_faces.obj");
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, kDefaultDiffuse);
  EXPECT_EQ(mesh.material.diffuse_map, "");
  EXPECT_THAT(
      TakeWarning(),
      testing::HasSubstr("use a single material across the whole mesh"));
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, kDefaultDiffuse);
  EXPECT_EQ(mesh.material.diffuse_map, "");
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, kDefaultDiffuse);
  EXPECT_EQ(mesh.material.diffuse_map, "");
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, kDefaultDiffuse);
  EXPECT_EQ(mesh.material.diffuse_map, "");
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, empty_props(), kDefaultDiffuse, diagnostic_policy_);
  EXPECT_EQ(mesh.material.diffuse, kDefaultDiffuse);
  EXPECT_EQ(mesh.material.diffuse_map, "");
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
  const RenderMesh mesh = LoadRenderMeshFromObj(
      obj_path, props, kDefaultDiffuse, diagnostic_policy_);
  // We still get the intrinsic material.
  EXPECT_EQ(mesh.material.diffuse, Rgba(0.5, 1, 1));
  EXPECT_EQ(mesh.material.diffuse_map, "");
  EXPECT_THAT(
      TakeWarning(),
      testing::MatchesRegex(".*has its own materials.*'phong', 'diffuse'.*"));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
