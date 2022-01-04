#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

using std::optional;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;

const char* const kTestDir =
    "drake/multibody/parsing/test/process_model_directives_test";

// Our unit test's package is not normally loaded; construct a parser that
// has it and can resolve package://process_model_directives_test urls.
std::unique_ptr<Parser> MakeTestParser(MultibodyPlant<double>* plant) {
  auto parser = std::make_unique<Parser>(plant);
  const drake::filesystem::path abspath_xml = FindResourceOrThrow(
      std::string(kTestDir) + "/package.xml");
  parser->package_map().AddPackageXml(abspath_xml.string());
  return parser;
}

// Parity test for add_scoped_sub.
GTEST_TEST(ProcessModelDirectivesTest, BasicSmokeTestFromSDF) {
  ModelDirectives station_directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_sub.yaml"));
  const MultibodyPlant<double> md_empty_plant(0.0);

  // md_ prefixes are added to artifacts generated using Model Directives.
  MultibodyPlant<double> md_plant(0.0);
  ProcessModelDirectives(station_directives, &md_plant,
                         nullptr, MakeTestParser(&md_plant).get());
  md_plant.Finalize();
  auto md_context = md_plant.CreateDefaultContext();

  // sdf_ prefixes are added to artifacts generated using sdformat.
  MultibodyPlant<double> sdf_plant(0.0);

  const std::string sdf_name = FindResourceOrThrow(
      std::string(kTestDir) + "/add_scoped_sub.sdf");
  auto parser = MakeTestParser(&sdf_plant);
  parser->AddAllModelsFromFile(sdf_name);
  sdf_plant.Finalize();
  auto sdf_context = sdf_plant.CreateDefaultContext();

  // The number of model instances for sdf_plant is now 3, due to the overall
  // top level model add_scoped_sub.
  const MultibodyPlant<double> empty_plant(0.0);
  EXPECT_EQ(md_plant.num_model_instances() - empty_plant.num_model_instances(),
            2);
  EXPECT_EQ(sdf_plant.num_model_instances() - empty_plant.num_model_instances(),
            3);

  // Expect the two bodies added by the directives.
  EXPECT_EQ(md_plant.num_bodies() - empty_plant.num_bodies(), 2);
  EXPECT_EQ(sdf_plant.num_bodies() - empty_plant.num_bodies(), 2);

  // A great many frames are added in model directives processing, but we
  // should at least expect that our named ones are present.
  EXPECT_TRUE(sdf_plant.HasFrameNamed("sub_added_frame"));
  EXPECT_TRUE(sdf_plant.HasFrameNamed(
      "sub_added_frame",
      sdf_plant.GetModelInstanceByName("add_scoped_sub::simple_model")));
  EXPECT_TRUE(sdf_plant.HasFrameNamed(
      "frame",
      sdf_plant.GetModelInstanceByName("add_scoped_sub::simple_model")));
  EXPECT_TRUE(sdf_plant.HasFrameNamed(
      "frame",
      sdf_plant.GetModelInstanceByName("add_scoped_sub::extra_model")));

  // Compare all the frames
  EXPECT_TRUE(CompareMatrices(
      md_plant.GetFrameByName("sub_added_frame")
          .CalcPoseInWorld(*md_context)
          .GetAsMatrix4(),
      sdf_plant.GetFrameByName("sub_added_frame")
          .CalcPoseInWorld(*sdf_context)
          .GetAsMatrix4()));
  EXPECT_TRUE(CompareMatrices(
      md_plant.GetFrameByName(
          "frame",
          md_plant.GetModelInstanceByName("simple_model"))
              .CalcPoseInWorld(*md_context)
              .GetAsMatrix4(),
      sdf_plant.GetFrameByName(
          "frame",
          sdf_plant.GetModelInstanceByName("add_scoped_sub::simple_model"))
              .CalcPoseInWorld(*sdf_context)
              .GetAsMatrix4()));
  EXPECT_TRUE(CompareMatrices(
      md_plant.GetFrameByName(
          "frame",
          md_plant.GetModelInstanceByName("extra_model"))
              .CalcPoseInWorld(*md_context)
              .GetAsMatrix4(),
      sdf_plant.GetFrameByName(
          "frame",
          sdf_plant.GetModelInstanceByName("add_scoped_sub::extra_model"))
              .CalcPoseInWorld(*sdf_context)
              .GetAsMatrix4()));
}

// Parity test for add_scoped_top.
GTEST_TEST(ProcessModelDirectivesTest, AddScopedSmokeTestFromWorldFile) {
  ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow(std::string(kTestDir) + "/add_scoped_top.yaml"));

  // Ensure that we have a SceneGraph present so that we test relevant visual
  // pieces.
  DiagramBuilder<double> builder;

  // md_ prefixes are added to artifacts generated using Model Directives.
  MultibodyPlant<double>& md_plant = AddMultibodyPlantSceneGraph(&builder, 0.);
  ProcessModelDirectives(directives, &md_plant,
                         nullptr, MakeTestParser(&md_plant).get());
  md_plant.Finalize();
  auto diagram = builder.Build();
  auto md_context = md_plant.CreateDefaultContext();

  // sdf_ prefixes are added to artifacts generated using sdformat.
  MultibodyPlant<double> sdf_plant(0.0);
  const std::string sdf_name = FindResourceOrThrow(
      std::string(kTestDir) + "/add_scoped_top.sdf");
  auto parser = MakeTestParser(&sdf_plant);
  parser->AddAllModelsFromFile(sdf_name);
  sdf_plant.Finalize();
  auto sdf_context = sdf_plant.CreateDefaultContext();

  // The number of model instances for sdf_plant is now 13, due to the overall
  // top level model add_scoped_top.
  const MultibodyPlant<double> empty_plant(0.0);
  EXPECT_EQ(md_plant.num_model_instances() - empty_plant.num_model_instances(),
            12);
  EXPECT_EQ(sdf_plant.num_model_instances() - empty_plant.num_model_instances(),
            13);

  // Expect the 8 bodies added by the directives.
  EXPECT_EQ(md_plant.num_bodies() - empty_plant.num_bodies(), 8);
  EXPECT_EQ(sdf_plant.num_bodies() - empty_plant.num_bodies(), 8);

  // Query information and ensure we have expected results.
  // - Manually spell out one example.
  ASSERT_EQ(
      &GetScopedFrameByName(sdf_plant,
                            "add_scoped_top::left::simple_model::frame"),
      &sdf_plant.GetFrameByName(
          "frame",
          sdf_plant.GetModelInstanceByName(
              "add_scoped_top::left::simple_model")));

  // - Automate remaining tests.
  auto check_frame = [&md_plant, &md_context, &sdf_plant, &sdf_context](
      const std::string md_instance, const std::string frame) {
    // TODO(aaronchongth): We currently do not have the ability to read a top
    // level SDFormat model/world without its own scope included.
    const std::string sdf_instance = "add_scoped_top::" + md_instance;

    drake::log()->debug("Check instance: {}", sdf_instance);
    ASSERT_TRUE(md_plant.HasModelInstanceNamed(md_instance));
    ASSERT_TRUE(sdf_plant.HasModelInstanceNamed(sdf_instance));

    drake::log()->debug("Check frame: {}", frame);
    ASSERT_TRUE(
        md_plant.HasFrameNamed(
            frame, md_plant.GetModelInstanceByName(md_instance)));
    ASSERT_TRUE(
        sdf_plant.HasFrameNamed(
            frame, sdf_plant.GetModelInstanceByName(sdf_instance)));

    const std::string scoped_frame = sdf_instance + "::" + frame;
    ASSERT_EQ(
        &GetScopedFrameByName(sdf_plant, scoped_frame),
        &sdf_plant.GetFrameByName(frame,
            sdf_plant.GetModelInstanceByName(sdf_instance)));

    EXPECT_TRUE(CompareMatrices(
        md_plant.GetFrameByName(
            frame,
            md_plant.GetModelInstanceByName(md_instance))
                .CalcPoseInWorld(*md_context)
                .GetAsMatrix4(),
        sdf_plant.GetFrameByName(
            frame,
            sdf_plant.GetModelInstanceByName(sdf_instance))
                .CalcPoseInWorld(*sdf_context)
                .GetAsMatrix4()));
  };
  for (const std::string prefix : {"", "left::", "right::", "mid::nested::"}) {
    const std::string simple_model = prefix + "simple_model";
    check_frame(simple_model, "base");
    check_frame(simple_model, "frame");
    check_frame(simple_model, "sub_added_frame");
    check_frame(simple_model, "top_added_frame");
    const std::string extra_model = prefix + "extra_model";
    check_frame(extra_model, "base");
    check_frame(extra_model, "frame");
  }
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
