#include <atomic>
#include <memory>

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/builder_life_support_pybind.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_system.h"

using std::unique_ptr;

namespace drake {

using systems::BasicVector;
using systems::ContextBase;
using systems::LeafContext;
using systems::LeafSystem;
using systems::Simulator;

namespace pydrake {
namespace {

template <typename T>
class DiagramBuilderTestAdversary final : public systems::DiagramBuilder<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramBuilderTestAdversary);

  DiagramBuilderTestAdversary() = default;
  // The int-accepting constructor will be annotated in the wrapper.
  explicit DiagramBuilderTestAdversary(int) {}
  ~DiagramBuilderTestAdversary() final = default;
};

// The Arbitrary type exists to test annotations pointing to a wrong-typed
// argument.
class Arbitrary {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Arbitrary);

  Arbitrary() = default;
  ~Arbitrary() = default;
};

void DeclareBuilderLifeSupportTestHelpers(py::module m) {
  // the Arbitrary type exists to test annotations pointing to a wrong-typed
  // argument.
  py::class_<Arbitrary>(m, "Arbitrary").def(py::init<>());

  type_visit(
      [&m](auto dummy) {
        using systems::DiagramBuilder;
        using T = decltype(dummy);
        // Use this in tests to approximate a mixed-language program that
        // populates a DiagramBuilder in Python, but calls the Build() step from
        // C++. This version of Build() deliberately ignores any python-specific
        // object lifetime issues.
        m.def("call_build_from_cpp",
            [](DiagramBuilder<T>* builder) { return builder->Build(); });

        // Use this in tests to probe the failure modes of programming errors
        // with builder life support annotations.
        using internal::builder_life_support_stash;
        DefineTemplateClassWithDefault<DiagramBuilderTestAdversary<T>,
            DiagramBuilder<T>>(m, "DiagramBuilderTestAdversary",
            GetPyParam<T>(), "for testing", std::nullopt, py::dynamic_attr())
            .def(py::init<>())
            // The int-accepting constructor exists to test init-time use of
            // argument index 1.
            .def(py::init<int>(), builder_life_support_stash<T, 1>())
            .def(
                "StashSelf", [](DiagramBuilderTestAdversary<T>*) { return; },
                builder_life_support_stash<T, 1>())
            .def(
                "StashReturnedSelf",
                [](DiagramBuilderTestAdversary<T>* self) { return self; },
                builder_life_support_stash<T, 0>())
            .def(
                "StashReturnedNull",
                [](DiagramBuilderTestAdversary<T>*) { return nullptr; },
                builder_life_support_stash<T, 0>())
            .def(
                "StashBadIndex",
                [](DiagramBuilderTestAdversary<T>*) { return; },
                builder_life_support_stash<T, 2>())
            .def(
                "StashWrongType",
                [](DiagramBuilderTestAdversary<T>*, Arbitrary*) { return; },
                builder_life_support_stash<T, 2>())
            // Use this in case it is needed to undo any test-only immortality
            // problems.
            .def("Abandon", [](DiagramBuilderTestAdversary<T>* self) {
              internal::BuilderLifeSupport<T>::abandon(self);
            });
      },
      CommonScalarPack{});
}

using T = double;

// Informs listener when this class is deleted.
class DeleteListenerSystem : public LeafSystem<T> {
 public:
  explicit DeleteListenerSystem(std::function<void()> delete_callback)
      : LeafSystem<T>(), delete_callback_(delete_callback) {}

  ~DeleteListenerSystem() override { delete_callback_(); }

 private:
  std::function<void()> delete_callback_;
};

class DeleteListenerVector : public BasicVector<T> {
 public:
  explicit DeleteListenerVector(std::function<void()> delete_callback)
      : BasicVector(VectorX<T>::Constant(1, 0.)),
        delete_callback_(delete_callback) {}

  ~DeleteListenerVector() override { delete_callback_(); }

 private:
  std::function<void()> delete_callback_;
};

// A simple 2-dimensional subclass of BasicVector for testing.
template <typename T>
class MyVector2 : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyVector2);

  explicit MyVector2(const Vector2<T>& data) : BasicVector<T>(data) {}

 private:
  MyVector2* DoClone() const override {
    return new MyVector2(this->get_value());
  }
};

// DummySystemA is an interface class, and DummySystemB provides its
// implementation. MakeDummySystem() is a factory function for DummySystemA. In
// the pybind11 bindings below, we only bind the interface class (DummySystemA)
// and the factory function, not the concrete implementation (DummySystemB).
// This setup is intentional for testing a specific bug, as exercised in
// general_test.py -> test_system_builder_add_system().
class DummySystemA : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySystemA);
  DummySystemA() = default;
  virtual ~DummySystemA() = 0;
};
DummySystemA::~DummySystemA() = default;
class DummySystemB : public DummySystemA {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySystemB);
  DummySystemB() = default;
  ~DummySystemB() override = default;
};
std::unique_ptr<DummySystemA> MakeDummySystem() {
  return std::make_unique<DummySystemB>();
}

// This counter tracks the number of live CountingLeafContext instances. It is
// used to verify that contexts are properly freed when a Simulator is deleted.
std::atomic<int> number_of_live_counting_contexts{0};

// A LeafContext subclass that increments and decrements the global counter
// upon construction and destruction, respectively. This allows tests to
// verify that contexts created by the Simulator are properly garbage collected.
class CountingLeafContext final : public LeafContext<T> {
 public:
  CountingLeafContext() { ++number_of_live_counting_contexts; }

  ~CountingLeafContext() final { --number_of_live_counting_contexts; }

  CountingLeafContext& operator=(const CountingLeafContext&) = delete;
  CountingLeafContext(CountingLeafContext&&) = delete;
  CountingLeafContext& operator=(CountingLeafContext&&) = delete;

 private:
  CountingLeafContext(const CountingLeafContext& source)
      : LeafContext<T>(source) {
    ++number_of_live_counting_contexts;
  }

  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::unique_ptr<ContextBase>(new CountingLeafContext(*this));
  }
};

// A LeafSystem subclass that creates CountingLeafContext instances. Use this
// system in tests to verify proper context lifetime management.
class CountingContextSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CountingContextSystem);

  CountingContextSystem() = default;

  // Returns the current number of live CountingLeafContext instances.
  static int GetNumberOfLiveContexts() {
    return number_of_live_counting_contexts.load();
  }

 private:
  std::unique_ptr<LeafContext<T>> DoMakeLeafContext() const final {
    return std::make_unique<CountingLeafContext>();
  }
};

}  // namespace

PYBIND11_MODULE(test_util, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace systems;

  // Import dependencies.
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  py::class_<DeleteListenerSystem, LeafSystem<T>>(m, "DeleteListenerSystem")
      .def(py::init<std::function<void()>>());
  py::class_<DeleteListenerVector, BasicVector<T>>(m, "DeleteListenerVector")
      .def(py::init<std::function<void()>>());

  // A 2-dimensional subclass of BasicVector.
  py::class_<MyVector2<T>, BasicVector<T>>(m, "MyVector2")
      .def(py::init<const Eigen::Vector2d&>(), py::arg("data"));

  // Call overrides to ensure a custom Python class can override these methods.

  m.def("call_leaf_system_overrides", [](const LeafSystem<T>& system) {
    py::dict results;
    auto context = system.AllocateContext();
    {
      // Leverage simulator to call initialization events.
      // TODO(eric.cousineau): Simplify as part of #10015.
      Simulator<T> simulator(system);
      simulator.Initialize();
    }
    {
      // Call `Publish` to test `DoPublish`.
      auto events =
          LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection();
      const EventStatus status = system.Publish(*context, *events);
      DRAKE_DEMAND(status.did_nothing());
    }
    {
      // Call `HasDirectFeedthrough` to test `DoHasDirectFeedthrough`.
      results["has_direct_feedthrough"] = system.HasDirectFeedthrough(0, 0);
    }
    {
      // Call `CalcTimeDerivatives` to test `DoCalcTimeDerivatives`
      auto state_dot = system.AllocateTimeDerivatives();
      system.CalcTimeDerivatives(*context, state_dot.get());
    }
    {
      // Call `CalcForcedDiscreteVariableUpdate` to test
      // `DoCalcDiscreteVariableUpdates`.
      auto& state = context->get_mutable_discrete_state();
      auto state_copy = state.Clone();
      system.CalcForcedDiscreteVariableUpdate(*context, state_copy.get());

      // From t=0, return next update time for testing discrete time.
      // If there is an abstract / unrestricted update, this assumes that
      // `dt_discrete < dt_abstract`.
      auto events = system.AllocateCompositeEventCollection();
      results["discrete_next_t"] =
          system.CalcNextUpdateTime(*context, events.get());
    }
    return results;
  });

  m.def("call_vector_system_overrides",
      [](const VectorSystem<T>& system, Context<T>* context, bool is_discrete,
          double dt) {
        // While this is not convention, update state first to ensure that our
        // output incorporates it correctly, for testing purposes.
        // TODO(eric.cousineau): Add (Continuous|Discrete)State::Clone().
        if (is_discrete) {
          auto& state = context->get_mutable_discrete_state();
          auto state_copy = state.Clone();
          system.CalcForcedDiscreteVariableUpdate(*context, state_copy.get());
          state.SetFrom(*state_copy);
        } else {
          auto& state = context->get_mutable_continuous_state();
          auto state_dot = system.AllocateTimeDerivatives();
          system.CalcTimeDerivatives(*context, state_dot.get());
          state.SetFromVector(
              state.CopyToVector() + dt * state_dot->CopyToVector());
        }
        // Calculate output.
        auto output = system.AllocateOutput();
        system.CalcOutput(*context, output.get());
        return output;
      });

  DeclareBuilderLifeSupportTestHelpers(m);

  // We only bind the interface class (DummySystemA) and the factory function.
  py::class_<DummySystemA, LeafSystem<double>>(m, "DummySystemA");
  m.def("MakeDummySystem", &MakeDummySystem);

  py::class_<CountingContextSystem, LeafSystem<T>>(m, "CountingContextSystem")
      .def(py::init<>())
      .def_static("GetNumberOfLiveContexts",
          &CountingContextSystem::GetNumberOfLiveContexts);
}

}  // namespace pydrake
}  // namespace drake
