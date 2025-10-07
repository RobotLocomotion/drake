#include "drake/planning/collision_checker_context.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace {

using systems::Context;

/* Simple derived context; it adds a value and is cloneable. */
class DummyContext final : public CollisionCheckerContext {
 public:
  DummyContext(const RobotDiagram<double>* model, int value)
      : CollisionCheckerContext(model), value_(value) {}

  int value() const { return value_; }
  void set_value(int value) { value_ = value; }

 private:
  /* Copy constructor for cloning. */
  DummyContext(const DummyContext& context)
      : CollisionCheckerContext(context), value_(context.value_) {}

  std::unique_ptr<CollisionCheckerContext> DoClone() const final {
    return std::unique_ptr<DummyContext>(new DummyContext(*this));
  }

  int value_{};
};

/* Simply confirms that providing *only* an (empty) model populates all the
 contexts (and the attendant query object). We configure the plant for "live"
 mode, to opt-out of the extra non-kinematic state that would be used the plant
 in "sampled" mode. This would be a typical choice for a collision checker,
 because its plant is not used for simulation. */
GTEST_TEST(CollisionCheckerContextTest, SimpleConstructor) {
  RobotDiagramBuilder<double> builder;
  builder.plant().SetUseSampledOutputPorts(false);
  auto diagram = builder.Build();
  CollisionCheckerContext dut(diagram.get());
  EXPECT_EQ(dut.model_context().num_total_states(), 0);
  EXPECT_EQ(dut.plant_context().num_total_states(), 0);
  EXPECT_EQ(dut.scene_graph_context().num_total_states(), 0);
  EXPECT_EQ(dut.GetQueryObject().inspector().num_frames(), 1);
}

/* Confirms that the internal context mutators do the "right thing" -- provide
 non-const access to contexts that are all wired together. */
GTEST_TEST(CollisionCheckerContextTest, InternalMutators) {
  RobotDiagramBuilder<double> builder;
  auto diagram = builder.Build();
  CollisionCheckerContext dut(diagram.get());

  /* Part of this test is confirming we can assign to a *non-const* reference.
   We'll actually write and make sure it's visible across the whole family
   of contexts. */
  Context<double>& model_context = dut.mutable_model_context();
  const double t = 3.0;
  model_context.SetTime(t);
  Context<double>& plant_context = dut.mutable_plant_context();
  EXPECT_EQ(plant_context.get_time(), t);
  Context<double>& scene_graph_context = dut.mutable_scene_graph_context();
  EXPECT_EQ(scene_graph_context.get_time(), t);
}

/* Confirms that the context properly clones itself; the clone got the source's
 values and are independent. Also, the clone's sub-contexts are correctly
 wired -- they are the contexts stored in the clone. */
GTEST_TEST(CollisionCheckerContextTest, Cloneable) {
  RobotDiagramBuilder<double> builder;
  auto diagram = builder.Build();
  CollisionCheckerContext source(diagram.get());

  const double t = 1.234;
  source.mutable_model_context().SetTime(t);

  auto cloned = source.Clone();
  ASSERT_NE(cloned, nullptr);

  /* The clone has unique instances. */
  EXPECT_NE(&cloned->model_context(), &source.model_context());
  EXPECT_NE(&cloned->plant_context(), &source.plant_context());
  EXPECT_NE(&cloned->scene_graph_context(), &source.scene_graph_context());

  /* The clone is wired; the plant/scene graph contexts belong to the model
   context. */
  EXPECT_EQ(&diagram->plant_context(cloned->model_context()),
            &cloned->plant_context());
  EXPECT_EQ(&diagram->scene_graph_context(cloned->model_context()),
            &cloned->scene_graph_context());

  /* The cloned context were *copied* and not simply default allocated. */
  EXPECT_EQ(cloned->model_context().get_time(),
            source.model_context().get_time());
}

/* Confirms that a further context can be derived and successfully cloned. */
GTEST_TEST(CollisionCheckerContextTest, Derived) {
  RobotDiagramBuilder<double> builder;
  auto diagram = builder.Build();
  DummyContext source_context(diagram.get(), 7);
  std::unique_ptr<CollisionCheckerContext> cloned = source_context.Clone();
  DummyContext* cloned_raw = dynamic_cast<DummyContext*>(cloned.get());
  ASSERT_NE(cloned_raw, nullptr);
  EXPECT_EQ(cloned_raw->value(), source_context.value());
  source_context.set_value(17);
  EXPECT_NE(cloned_raw->value(), source_context.value());
}

}  // namespace
}  // namespace planning
}  // namespace drake
