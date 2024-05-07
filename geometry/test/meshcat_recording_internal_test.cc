#include "drake/geometry/meshcat_recording_internal.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using math::RigidTransformd;

GTEST_TEST(MeshcatRecordingInternalTest, BasicLifecycle) {
  MeshcatRecording dut;
  EXPECT_NO_THROW(dut.StartRecording(64.0, true));
  dut.SetProperty("path", "bravo", true, 0.0);
  dut.SetProperty("path", "delta", 22.22, 0.0);
  dut.SetProperty("path", "victor", std::vector<double>({1, 2, 3}), 0.0);
  dut.SetTransform("path", RigidTransformd::Identity(), 0.0);
  EXPECT_NO_THROW(dut.get_animation());
  EXPECT_NO_THROW(dut.get_mutable_animation());
  EXPECT_NO_THROW(dut.StopRecording());
  EXPECT_NO_THROW(dut.DeleteRecording());
}

GTEST_TEST(MeshcatRecordingInternalTest, Properties) {
  // XXX
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
