#include <future>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/ssize.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/render_gl/internal_texture_library.h"

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {

class TextureLibraryTester {
 public:
  static int NumTextures(const TextureLibrary& library) {
    return ssize(library.textures_);
  }
};

}  // namespace internal
}  // namespace render_gl

namespace {

using render_gl::internal::TextureLibrary;
using render_gl::internal::TextureLibraryTester;

/* We should be able to safely call GetTextureId in multiple threads. */
GTEST_TEST(ThreadSafetyTest, TextureLibrary) {
  TextureLibrary library;
  ASSERT_EQ(TextureLibraryTester::NumTextures(library), 0);

  const std::string image_name =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");

  std::atomic<int> num_errors{0};
  auto work = [&library, &image_name, &num_errors] (int i) {
    try {
      std::optional<GLuint> id = library.GetTextureId(image_name);
      if (!id.has_value()) {
        throw std::runtime_error("no valid texture id returned.");
      }
    } catch (std::exception& e) {
      log()->error("Worker {} unexpected error: {}", i, e.what());
      ++num_errors;
    }
  };

  std::vector<std::future<void>> futures;
  futures.push_back(std::async(std::launch::async, work, 0));
  futures.push_back(std::async(std::launch::async, work, 1));
  futures.push_back(std::async(std::launch::async, work, 2));

  for (auto& future : futures) {
    future.get();
  }

  /* All invocations successfully returned an id. */
  ASSERT_EQ(num_errors, 0);
  /* There is still only a single texture in the library. */
  EXPECT_EQ(TextureLibraryTester::NumTextures(library), 1);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
