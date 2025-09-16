#include <string>
#include <vector>

#include <benchmark/benchmark.h>

#include "drake/lcmt_image.hpp"
#include "drake/lcmt_image_array.hpp"
#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/tools/performance/fixture_common.h"

/* A collection of LCM encoding / decoding scenarios. */

namespace drake {
namespace {

class LcmFixture : public benchmark::Fixture {
 public:
  LcmFixture() {
    tools::performance::AddMinMaxStatistics(this);
    this->Unit(benchmark::kMicrosecond);
  }

  static lcmt_image MakeImage(std::string frame_name, int bytes_per_pixel) {
    lcmt_image message{};
    message.header.frame_name = frame_name, message.width = 848;
    message.height = 480;
    message.data.resize(message.width * message.height * bytes_per_pixel);
    message.size = message.data.size();
    return message;
  }

  static lcmt_image_array MakeImageArray() {
    lcmt_image_array message{};
    message.header.seq = 1;
    message.header.utime = 2;
    message.header.frame_name = "frame";
    message.images.push_back(MakeImage("color", 4));
    message.images.push_back(MakeImage("depth", 2));
    message.num_images = message.images.size();
    return message;
  }

  static lcmt_panda_command MakePandaCommand() {
    lcmt_panda_command message{};
    const int n = 7;
    message.num_joint_position = n;
    message.joint_position.resize(n, 0);
    message.num_joint_torque = n;
    message.joint_torque.resize(n, 0);
    return message;
  }

  static lcmt_panda_status MakePandaStatus() {
    lcmt_panda_status message{};
    const int n = 7;
    message.num_joints = n;
    message.joint_position.resize(n, 0);
    message.joint_position_desired.resize(n, 0);
    message.joint_velocity.resize(n, 0);
    message.joint_velocity_desired.resize(n, 0);
    message.joint_acceleration_desired.resize(n, 0);
    message.joint_torque.resize(n, 0);
    message.joint_torque_desired.resize(n, 0);
    message.joint_torque_external.resize(n, 0);
    return message;
  }

  static lcmt_viewer_draw MakeViewerDraw() {
    lcmt_viewer_draw message{};
    const int n = 500;
    message.num_links = n;
    message.link_name.resize(n, "parsers_make_very_long_names");
    message.robot_num.resize(n, 0);
    message.position.resize(n, {0, 0, 0});
    message.quaternion.resize(n, {0, 0, 0, 0});
    return message;
  }
};

template <typename Message>
__attribute__((noinline)) bool Encode(const Message& message,
                                      std::vector<uint8_t>* bytes) {
  const int64_t num_bytes = message.getEncodedSize();
  bytes->resize(num_bytes);
  message.encode(bytes->data(), 0, num_bytes);
  return true;
}

template <typename Message>
__attribute__((noinline)) bool Decode(const std::vector<uint8_t>& bytes,
                                      Message* message) {
  message->decode(bytes.data(), 0, bytes.size());
  return true;
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, ImageArrayEncode)(benchmark::State& state) {
  const auto message = MakeImageArray();
  for (auto _ : state) {
    std::vector<uint8_t> bytes;
    benchmark::DoNotOptimize(Encode(message, &bytes));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, ImageArrayEncode)
    ->Unit(benchmark::kMicrosecond);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, ImageArrayDecode)(benchmark::State& state) {
  std::vector<uint8_t> bytes;
  Encode(MakeImageArray(), &bytes);
  for (auto _ : state) {
    lcmt_image_array message{};
    benchmark::DoNotOptimize(Decode(bytes, &message));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, ImageArrayDecode)
    ->Unit(benchmark::kMicrosecond);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, PandaCommandEncode)(benchmark::State& state) {
  const auto message = MakePandaCommand();
  for (auto _ : state) {
    std::vector<uint8_t> bytes;
    benchmark::DoNotOptimize(Encode(message, &bytes));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, PandaCommandEncode)
    ->Unit(benchmark::kNanosecond);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, PandaCommandDecode)(benchmark::State& state) {
  std::vector<uint8_t> bytes;
  Encode(MakePandaCommand(), &bytes);
  for (auto _ : state) {
    lcmt_image_array message{};
    benchmark::DoNotOptimize(Decode(bytes, &message));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, PandaCommandDecode)
    ->Unit(benchmark::kNanosecond);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, PandaStatusEncode)(benchmark::State& state) {
  const auto message = MakePandaStatus();
  for (auto _ : state) {
    std::vector<uint8_t> bytes;
    benchmark::DoNotOptimize(Encode(message, &bytes));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, PandaStatusEncode)
    ->Unit(benchmark::kNanosecond);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, PandaStatusDecode)(benchmark::State& state) {
  std::vector<uint8_t> bytes;
  Encode(MakePandaStatus(), &bytes);
  for (auto _ : state) {
    lcmt_image_array message{};
    benchmark::DoNotOptimize(Decode(bytes, &message));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, PandaStatusDecode)
    ->Unit(benchmark::kNanosecond);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, ViewerDrawEncode)(benchmark::State& state) {
  const auto message = MakeViewerDraw();
  for (auto _ : state) {
    std::vector<uint8_t> bytes;
    benchmark::DoNotOptimize(Encode(message, &bytes));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, ViewerDrawEncode)
    ->Unit(benchmark::kMicrosecond);

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_DEFINE_F(LcmFixture, ViewerDrawDecode)(benchmark::State& state) {
  std::vector<uint8_t> bytes;
  Encode(MakeViewerDraw(), &bytes);
  for (auto _ : state) {
    lcmt_image_array message{};
    benchmark::DoNotOptimize(Decode(bytes, &message));
  }
}

BENCHMARK_REGISTER_F(LcmFixture, ViewerDrawDecode)
    ->Unit(benchmark::kNanosecond);

}  // namespace
}  // namespace drake
