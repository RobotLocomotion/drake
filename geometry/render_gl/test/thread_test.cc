#include <future>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/planning/robot_diagram_builder.h"

namespace {
enum AsyncMode {
  /* No asynchrony; run serially. */
  kNone,
  /* Use std::async(policy=std::launch::async). */
  kTask,
  /* Use std::thread. */
  kThread,
};
std::string_view to_string(AsyncMode mode) {
  switch (mode) {
    case kNone:
      return "kNone";
    case kTask:
      return "kTask";
    case kThread:
      return "kThread";
  }
  DRAKE_UNREACHABLE();
}
}  // namespace
DRAKE_FORMATTER_AS(, , AsyncMode, x, to_string(x))

namespace drake {
namespace geometry {
namespace {

using planning::RobotDiagramBuilder;
using render::ClippingRange;
using render::ColorRenderCamera;
using render::RenderCameraCore;
using render::RenderEngine;
using systems::Context;
using systems::Diagram;
using systems::System;
using systems::sensors::CameraInfo;
using systems::sensors::ImageRgba8U;

struct Params {
  /* Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(async_mode));
    a->Visit(DRAKE_NVP(num_workers));
    a->Visit(DRAKE_NVP(num_repeats));
  }

  AsyncMode async_mode{kNone};
  int num_workers{1};
  int num_repeats{1};

  friend std::ostream& operator<<(std::ostream& os, const Params& params) {
    os << yaml::SaveJsonString(params);
    return os;
  }
};

class ThreadTest : public testing::TestWithParam<Params> {};

TEST_P(ThreadTest, Run) {
  const auto& [async_mode, num_workers, num_repeats] = GetParam();
  DRAKE_DEMAND(num_workers >= 0);
  DRAKE_DEMAND(num_repeats >= 0);

  // Create the plant and scene_graph.
  auto builder = std::make_unique<RobotDiagramBuilder<double>>();

  // Prepare the cameras, one per thread.
  std::vector<std::string> renderer_names;
  for (int i = 0; i < num_workers; ++i) {
    renderer_names.push_back(fmt::format("gl[{}]", i));
    builder->scene_graph().AddRenderer(renderer_names.back(),
                                       MakeRenderEngineGl());
  }
  const ColorRenderCamera camera_params(RenderCameraCore(
      "color", CameraInfo(640, 480, 1.0), ClippingRange(0.25, 5.0), {}));

  // Add model(s).
  builder->parser().AddModelsFromUrl(
      "package://drake/geometry/render/test/box.sdf");

  // Finalize and create per-thread diagram contexts.
  std::unique_ptr<Diagram<double>> mutable_diagram = builder->Build();
  const Diagram<double>& const_diagram = *mutable_diagram;
  std::vector<std::unique_ptr<Context<double>>> contexts;
  for (int i = 0; i < num_workers; ++i) {
    contexts.push_back(const_diagram.CreateDefaultContext());
  }

  // The worker functor for the i'th camera.
  std::atomic<int> num_errors{0};
  auto work = [&num_errors, num_repeats = num_repeats, &renderer_names,
               &camera_params, &const_diagram, &contexts](int i) {
    Context<double>& diagram_context = *contexts.at(i);
    const std::string& renderer_name = renderer_names.at(i);
    try {
      for (int j = 0; j < num_repeats; ++j) {
        // Invalidate the state to force re-calculation of the output.
        diagram_context.get_mutable_state();
        // Grab the QueryObject.
        const auto& scene_graph =
            const_diagram.GetSubsystemByName("scene_graph");
        const auto& query_object =
            scene_graph.GetOutputPort("query")
                .template Eval<QueryObject<double>>(
                    scene_graph.GetMyContextFromRoot(diagram_context));
        // Render an image.
        const RenderEngine* const renderer =
            query_object.GetRenderEngineByName(renderer_name);
        DRAKE_THROW_UNLESS(renderer != nullptr);
        ImageRgba8U dummy(camera_params.core().intrinsics().width(),
                          camera_params.core().intrinsics().height());
        renderer->RenderColorImage(camera_params, &dummy);
      }
    } catch (std::exception& e) {
      drake::log()->error("Worker {} exception: {}", i, e.what());
      ++num_errors;
    }
  };

  // Render on multiple threads concurrently.
  switch (async_mode) {
    case kNone:
      drake::log()->info("Runnning {} workers serially", num_workers);
      break;
    case kTask:
      drake::log()->info("Launching {} std::async workers", num_workers);
      break;
    case kThread:
      drake::log()->info("Launching {} std::thread workers", num_workers);
      break;
  }
  std::vector<std::future<void>> futures;
  std::vector<std::thread> threads;
  for (int i = 0; i < num_workers; ++i) {
    switch (async_mode) {
      case kNone:
        work(i);
        break;
      case kTask:
        futures.push_back(std::async(std::launch::async, work, i));
        break;
      case kThread:
        threads.push_back(std::thread(work, i));
        break;
    }
  }

  // Wait for all workers to finish.
  for (auto& thread : threads) {
    thread.join();
  }
  for (auto& future : futures) {
    future.get();
  }

  EXPECT_EQ(num_errors, 0);
}

constexpr Params kParams[] = {
    {.async_mode = kNone, .num_workers = 1, .num_repeats = 2},
    {.async_mode = kTask, .num_workers = 3, .num_repeats = 2},
    {.async_mode = kThread, .num_workers = 3, .num_repeats = 2},
};

INSTANTIATE_TEST_SUITE_P(All, ThreadTest, testing::ValuesIn(kParams));

}  // namespace
}  // namespace geometry
}  // namespace drake
