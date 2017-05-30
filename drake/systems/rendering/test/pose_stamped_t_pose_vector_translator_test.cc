#include "drake/systems/rendering/pose_stamped_t_pose_vector_translator.h"

#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "robotlocomotion/pose_stamped_t.hpp"

#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace systems {
namespace rendering {

namespace {
// This is empirically determined.
const double kTolerance = 1e-12;
}

class PoseStampedTPoseVectorTranslatorTest : public ::testing::Test {
 public:
  PoseStampedTPoseVectorTranslatorTest()
      : kFrameName("foo"),
        kPosition(1., 2., 3.),
        // This is from rpy2quat(PI / 3, PI / 2, PI / 4).
        kOrientation(0.70105738464997791048, 0.09229595564125714358,
                     0.70105738464997791048, -0.09229595564125722684),
        dut_(kFrameName),
        actual_pose_vector_(new PoseVector<double>()) {
    expected_pose_vector_.set_translation(kPosition);
    expected_pose_vector_.set_rotation(kOrientation);
  }

 protected:
  const double kTimestamp{0.001};
  const std::string kFrameName;
  const Eigen::Translation<double, 3> kPosition;
  const Eigen::Quaterniond kOrientation;

  PoseVector<double> expected_pose_vector_;
  PoseStampedTPoseVectorTranslator dut_;
  std::unique_ptr<VectorBase<double>> actual_pose_vector_;
  std::vector<uint8_t> buffer_;
};


TEST_F(PoseStampedTPoseVectorTranslatorTest, SerializeTest) {
  dut_.Serialize(kTimestamp, expected_pose_vector_, &buffer_);

  robotlocomotion::pose_stamped_t actual_pose_msg;
  actual_pose_msg.decode(buffer_.data(), 0, buffer_.size());

  // Verifies the pose.
  auto position = actual_pose_msg.pose.position;
  EXPECT_NEAR(expected_pose_vector_.GetAtIndex(0), position.x, kTolerance);
  EXPECT_NEAR(expected_pose_vector_.GetAtIndex(1), position.y, kTolerance);
  EXPECT_NEAR(expected_pose_vector_.GetAtIndex(2), position.z, kTolerance);
  auto orientation = actual_pose_msg.pose.orientation;
  EXPECT_NEAR(expected_pose_vector_.GetAtIndex(3), orientation.w, kTolerance);
  EXPECT_NEAR(expected_pose_vector_.GetAtIndex(4), orientation.x, kTolerance);
  EXPECT_NEAR(expected_pose_vector_.GetAtIndex(5), orientation.y, kTolerance);
  EXPECT_NEAR(expected_pose_vector_.GetAtIndex(6), orientation.z, kTolerance);
  // Verifies the frame name.
  EXPECT_EQ(kFrameName, actual_pose_msg.header.frame_name);
  // Verifies the timestamp.
  EXPECT_EQ(static_cast<int64_t>(kTimestamp * 1000000),
            actual_pose_msg.header.utime);
  // Verifies the seq. It's always zero for now.
  EXPECT_EQ(0, actual_pose_msg.header.seq);
}

TEST_F(PoseStampedTPoseVectorTranslatorTest, SerializeAndDeserializeTest) {
  dut_.Serialize(kTimestamp, expected_pose_vector_, &buffer_);
  dut_.Deserialize(static_cast<void*>(buffer_.data()), buffer_.size(),
                   actual_pose_vector_.get());

  // Verifies each element of the pose vector.
  for (int i = 0; i < PoseVector<double>::kSize; ++i) {
    EXPECT_NEAR(expected_pose_vector_.GetAtIndex(i),
                actual_pose_vector_->GetAtIndex(i),
                kTolerance);
  }
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake
