#include "drake/systems/framework/system_visitor.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/adder.h"

namespace drake {
namespace systems {

namespace {

typedef std::vector<const System<double>*> Systems;

class MyVisitor : public SystemVisitor<double> {
 public:
  void VisitSystem(const System<double>& system) final {
    visited_systems_.push_back(&system);
  }
  void VisitDiagram(const Diagram<double>& diagram) final {
    visited_diagrams_.push_back(&diagram);
  }

  const Systems& visited_systems() const { return visited_systems_; }

  const Systems& visited_diagrams() const { return visited_diagrams_; }

 private:
  Systems visited_systems_{};
  Systems visited_diagrams_{};
};

void VisitAndCheck(const System<double>& system,
                   const Systems& expected_visited_systems,
                   const Systems& expected_visited_diagrams) {
  MyVisitor visitor;
  system.Accept(&visitor);
  EXPECT_EQ(visitor.visited_systems(), expected_visited_systems);
  EXPECT_EQ(visitor.visited_diagrams(), expected_visited_diagrams);
}

}  // namespace

// Construct a nested diagram and ensure the subcomponents are visited
// correctly.
GTEST_TEST(SystemVisitorTest, NestedDiagram) {
  DiagramBuilder<double> builder;
  const Adder<double>* adder = builder.AddSystem<Adder<double>>(1, 1);
  DiagramBuilder<double> builder2;
  const Diagram<double>* diagram = builder2.AddSystem(builder.Build());
  auto diagram2 = builder2.Build();

  VisitAndCheck(*adder, {adder}, {});
  VisitAndCheck(*diagram, {}, {diagram});

  // Confirm that we do NOT recurse automatically.
  VisitAndCheck(*diagram2, {}, {diagram2.get()});
}

}  // namespace systems
}  // namespace drake
