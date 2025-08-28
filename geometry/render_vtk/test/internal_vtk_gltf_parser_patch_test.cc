/* This test supports the patch in internal_vtk/patches/gltf_parser.patch. A
 condition of removing that patch is that these tests should still pass.

 The patch does several things -- not all of which are tested here.

   Defect 1:
     - When a glTF has a material with textures *but no color texture*, VTK
       throws out all of the textures.
     - Test by loading such a glTF and confirm that the specified textures are
       present.
   Defect 2:
     - For the glTF file described in defect 1, the glTF importer would
       incorrectly conclude that the glTF had multiple sets of texture
       coordinates.
     - This is *not* tested because it would manifest itself as a warning
       written to the console (which we can't detect here). When the TODO in
       RenderEngineVtk::ImplementGltf is resolved, we can test it here. (There
       should be no logged warning). However, if the patch under test is not
       applied, the test fails and the resultant console dump includes the
       warning.
    Defect 3:
      - The VTK importer would force the warnings to go to the console, ignoring
        any event observers registered with the importer.
      - This, again, can only be tested once the TODO in
        RenderEngineVtk::ImplementGltf is resolved. */

#include <string>

#include <gtest/gtest.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkGLTFImporter.h>  // vtkIOImport
#include <vtkNew.h>           // vtkCommonCore
#include <vtkProperty.h>      // vtkRenderingCore
#include <vtkRenderer.h>      // vtkRenderingCore

#include "drake/common/find_resource.h"
#include "drake/geometry/render_vtk/factory.h"

namespace drake {
namespace {

// The test for defect 1 (described above).
GTEST_TEST(VtkGltfParserPatchTest, ColorTextureNotRequired) {
  // Instantiating RenderEngineVtk implicitly initializes VTK to use OpenGL.
  geometry::MakeRenderEngineVtk({});

  // This glTF uses all supported glTF textures types *except* base color.
  const std::string gltf_path = FindResourceOrThrow(
      "drake/geometry/render_vtk/test/pyramid_no_color_texture.gltf");
  vtkNew<vtkGLTFImporter> importer;
  importer->SetFileName(gltf_path.c_str());
  importer->Update();

  vtkRenderer* renderer = importer->GetRenderer();
  DRAKE_DEMAND(renderer != nullptr);
  ASSERT_EQ(renderer->VisibleActorCount(), 1);

  auto* actors = renderer->GetActors();
  actors->InitTraversal();
  vtkActor* actor = actors->GetNextActor();

  // We expect no color texture (aka "albedo"), but all of the other supported
  // textures. These names are lifted from VTK's code -- it is the name VTK
  // gives to the various textures instantiated by the glTF parser.
  EXPECT_EQ(actor->GetProperty()->GetTexture("albedoTex"), nullptr);
  EXPECT_NE(actor->GetProperty()->GetTexture("normalTex"), nullptr);
  EXPECT_NE(actor->GetProperty()->GetTexture("emissiveTex"), nullptr);
  // Material includes ambient occlusion in R, roughness in G, and metallic in
  // B. We don't need to check the values; the presence of the texture is
  // enough.
  EXPECT_NE(actor->GetProperty()->GetTexture("materialTex"), nullptr);
}

}  // namespace
}  // namespace drake
