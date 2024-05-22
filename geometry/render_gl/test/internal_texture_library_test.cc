#include "drake/geometry/render_gl/internal_texture_library.h"

#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {
namespace {

namespace fs = std::filesystem;

/* This test doesn't test the following:

 - Thread safety of calling GetTextureId() - see multithread_safety_test.cc
   for that test.
 - Doesn't test the image parsing and OpenGl texture loading logic of
   GetTextureId() or AddInMemoryImages(). If those failed, it would be apparent
   in the RenderEngineGl tests -- textures would not get loaded properly. */

/* Confirms that the texture key is generated as expected, handling symlinks
 and actual paths. */
GTEST_TEST(TextureLibraryTest, TextureKey) {
  const std::string image_link =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");
  // In the bazel ecosystem, all of our resources are symlinks.
  ASSERT_TRUE(fs::is_symlink(image_link));
  const std::string real_image = fs::read_symlink(image_link).string();
  ASSERT_NE(image_link, real_image);

  // A hard path to the file is its own key.
  EXPECT_EQ(TextureLibrary::GetTextureKey(real_image), real_image);

  // The symlink gets resolved.
  EXPECT_EQ(TextureLibrary::GetTextureKey(image_link), real_image);

  // Non-existent is its own key.
  const std::string bad_path("/not/a/path/to/stuff");
  EXPECT_EQ(TextureLibrary::GetTextureKey(bad_path), bad_path);
}

/* Confirms the logic for deciding if an image is supported. */
GTEST_TEST(TextureLibraryTest, SupportedImage) {
  const std::string image_link =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");
  const fs::path dir = temp_directory();

  // Lower case.
  const fs::path f1 = dir / "local.png";
  fs::copy_file(image_link, f1);
  EXPECT_TRUE(TextureLibrary::IsSupportedImage(f1.string()));

  // Upper case.
  const fs::path f2 = dir / "local.PNG";
  fs::rename(f1, f2);
  EXPECT_TRUE(TextureLibrary::IsSupportedImage(f2.string()));

  // Unsupported extension.
  const fs::path f3 = dir / "local.jpg";
  fs::rename(f2, f3);
  EXPECT_FALSE(TextureLibrary::IsSupportedImage(f3.string()));

  // Almost supported extension.
  const fs::path f4 = dir / "localpng";
  fs::rename(f3, f4);
  EXPECT_FALSE(TextureLibrary::IsSupportedImage(f4.string()));

  // Non-existent file.
  EXPECT_FALSE(
      TextureLibrary::IsSupportedImage((dir / "not_real.png").string()));

  // Badly named directory.
  const fs::path local_dir = dir / "dir.png";
  fs::create_directory(local_dir);
  EXPECT_FALSE(TextureLibrary::IsSupportedImage(local_dir.string()));
}

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
