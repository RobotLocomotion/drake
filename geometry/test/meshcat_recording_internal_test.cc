#include "drake/geometry/meshcat_recording_internal.h"

#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using math::RigidTransformd;

using Vec = std::vector<double>;

GTEST_TEST(MeshcatRecordingInternalTest, LifecycleNoCrash) {
  MeshcatRecording dut;
  EXPECT_NO_THROW(dut.StartRecording(64.0, true));
  EXPECT_NO_THROW(dut.SetProperty("path", "bravo", true, 0.0));
  EXPECT_NO_THROW(dut.SetTransform("path", RigidTransformd::Identity(), 0.0));
  EXPECT_NO_THROW(dut.StopRecording());
  EXPECT_NO_THROW(dut.get_animation());
  EXPECT_NO_THROW(dut.get_mutable_animation());
  EXPECT_NO_THROW(dut.DeleteRecording());
}

enum RecordingMode : int {
  kNotStarted,
  kLive,
  kMute,
  kStopped,
};

// Check the code for SetProperty, SetTransform, and CalcDetail that governs
// when whether a given operation is added to the animation and/or shown live.
GTEST_TEST(MeshcatRecordingInternalTest, Properties) {
  const Vec vec({1, 2, 3});
  const RigidTransformd X_I(Eigen::Vector3d(1, 2, 3));

  // Sweep across several different times (as well as one nullopt time, first).
  for (const double test_time : {-1.0, 0.0, 0.125}) {
    const std::optional<double> rec_time =
        (test_time >= 0) ? std::optional<double>(test_time) : std::nullopt;

    // Sweep across state machine modes.
    for (int mode : {kNotStarted, kLive, kMute, kStopped}) {
      SCOPED_TRACE(fmt::format("test_time = {} mode = {}", test_time, mode));

      // Prepare the device under test, and our test expectations.
      MeshcatRecording dut;
      bool animate{};
      bool live{};
      const double fps = 64.0;
      switch (mode) {
        case kNotStarted:
          animate = false;
          live = true;
          break;
        case kLive:
          EXPECT_NO_THROW(dut.StartRecording(fps, true));
          animate = rec_time.has_value();
          live = true;
          break;
        case kMute:
          EXPECT_NO_THROW(dut.StartRecording(fps, false));
          animate = rec_time.has_value();
          live = !animate;
          break;
        case kStopped:
          EXPECT_NO_THROW(dut.StartRecording(fps, false));
          EXPECT_NO_THROW(dut.StopRecording());
          animate = false;
          live = true;
          break;
      }

      // Set one of each type of property. (Recall that setting a RigidTransform
      // de-sugars into setting two properties: "position" and "quaternion".)
      EXPECT_EQ(dut.SetProperty("path", "bravo", true, rec_time), live);
      EXPECT_EQ(dut.SetProperty("path", "delta", 22.22, rec_time), live);
      EXPECT_EQ(dut.SetProperty("path", "victor", vec, rec_time), live);
      EXPECT_EQ(dut.SetTransform("path", X_I, rec_time), live);

      // Check if the property was added to the animation.
      const MeshcatAnimation& animation = dut.get_animation();
      const int frame = rec_time.value_or(0.0) * fps;
      auto bravo = animation.get_key_frame<bool>(frame, "path", "bravo");
      auto delta = animation.get_key_frame<double>(frame, "path", "delta");
      auto victor = animation.get_key_frame<Vec>(frame, "path", "victor");
      auto position = animation.get_key_frame<Vec>(frame, "path", "position");
      if (animate) {
        EXPECT_EQ(bravo, true);
        EXPECT_EQ(delta, 22.22);
        EXPECT_EQ(victor, vec);
        EXPECT_EQ(position, vec);
      } else {
        EXPECT_EQ(bravo, std::nullopt);
        EXPECT_EQ(delta, std::nullopt);
        EXPECT_EQ(victor, std::nullopt);
        EXPECT_EQ(position, std::nullopt);
      }
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
