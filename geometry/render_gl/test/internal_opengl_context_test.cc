#include "drake/geometry/render_gl/internal_opengl_context.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace render_gl {
namespace internal {
namespace {

// Tests the context binding APIs: IsCurrent(), MakeCurrent(), and
// ClearCurrent().
//
//   - A created context isn't bound.
//   - IsCurrent() correctly reports a context's status *for the current
//     thread*.
//   - MakeCurrent() binds the current thread.
//     - Replacing any previously bound context.
//   - ClearCurrent() clears any bound context.
GTEST_TEST(OpenGlContextTest, BindingContexts) {
  const OpenGlContext c1;
  EXPECT_FALSE(c1.IsCurrent());
  const OpenGlContext c2;
  EXPECT_FALSE(c2.IsCurrent());

  // Binding one means it is the only one that reports as bound.
  c1.MakeCurrent();
  EXPECT_TRUE(c1.IsCurrent());
  EXPECT_FALSE(c2.IsCurrent());

  // Binding another replaces the first.
  c2.MakeCurrent();
  EXPECT_FALSE(c1.IsCurrent());
  EXPECT_TRUE(c2.IsCurrent());

  // Calling ClearCurrent() clears them both.
  OpenGlContext::ClearCurrent();
  EXPECT_FALSE(c1.IsCurrent());
  EXPECT_FALSE(c2.IsCurrent());

  // Calling ClearCurrent() with nothing bound has no effect.
  OpenGlContext::ClearCurrent();
  EXPECT_FALSE(c1.IsCurrent());
  EXPECT_FALSE(c2.IsCurrent());
}

GTEST_TEST(OpenGlContext, WindowVisibility) {
  OpenGlContext opengl_context;

  // Initially, there is no viewable window.
  EXPECT_FALSE(opengl_context.IsWindowViewable());

  // Calling once, makes it viewable.
  opengl_context.DisplayWindow(640, 480);
  EXPECT_TRUE(opengl_context.IsWindowViewable());

  // Calling it again doesn't change that.
  opengl_context.DisplayWindow(640, 480);
  EXPECT_TRUE(opengl_context.IsWindowViewable());

  // Hiding it makes it no longer viewable.
  opengl_context.HideWindow();
  EXPECT_FALSE(opengl_context.IsWindowViewable());

  // Calling it again doesn't have any effect.
  opengl_context.HideWindow();
  EXPECT_FALSE(opengl_context.IsWindowViewable());
}

// Confirm that a context clone shares objects with its source.
GTEST_TEST(OpenGlContextTest, CloneShares) {
  const OpenGlContext source;
  source.MakeCurrent();

  // Create a buffer in source's context.
  GLuint buffer;
  glCreateFramebuffers(1, &buffer);
  ASSERT_TRUE(glIsFramebuffer(buffer));
  // Reject the null hypothesis -- not everything is a frame buffer.
  ASSERT_FALSE(glIsFramebuffer(buffer + 1));

  // Clone shares the frame buffer.
  const OpenGlContext clone(source);
  clone.MakeCurrent();
  // It's still a frame buffer for the clone of the source context.
  EXPECT_TRUE(glIsFramebuffer(buffer));

  // However, a newly created OpenGlContext does *not* have the frame buffer.
  const OpenGlContext independent;
  independent.MakeCurrent();
  ASSERT_FALSE(glIsFramebuffer(buffer));
}

GTEST_TEST(OpenGlContextTest, SourceSurvivesCloneDeath) {
  const OpenGlContext source;
  source.MakeCurrent();

  // Create a buffer in source's context.
  GLuint buffer;
  glCreateFramebuffers(1, &buffer);

  {
    // Create and destroy the clone; the source isn't harmed.
    const OpenGlContext clone(source);
    clone.MakeCurrent();
    // It's still a frame buffer with the clone of the source context.
    ASSERT_TRUE(glIsFramebuffer(buffer));
    // We'll destroy this context as we leave. Making sure the context isn't
    // bound when the destructor is called will lead to its immediate
    // destruction.
    OpenGlContext::ClearCurrent();
  }

  // Even after the destruction of the sharing context, the buffer still exists
  // in the original.
  source.MakeCurrent();
  ASSERT_TRUE(glIsFramebuffer(buffer));
}

GTEST_TEST(OpenGlContextTest, CloneSurvivesSourceDeath) {
  // This will get defined from a source that gets destroyed.
  std::unique_ptr<OpenGlContext> clone = nullptr;
  GLuint buffer;

  {
    const OpenGlContext source;
    source.MakeCurrent();

    // Create a buffer in source's context.
    glCreateFramebuffers(1, &buffer);

    clone = std::make_unique<OpenGlContext>(source);

    // Make sure the source isn't bound so its OpenGl objects get destroyed
    // immediately.
    OpenGlContext::ClearCurrent();
  }

  clone->MakeCurrent();
  ASSERT_TRUE(glIsFramebuffer(buffer));
}

}  // namespace
}  // namespace internal
}  // namespace render_gl
}  // namespace geometry
}  // namespace drake
