#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_system.h"

using std::unique_ptr;

namespace drake {

using systems::BasicVector;
using systems::LeafSystem;
using systems::Simulator;

namespace pydrake {
namespace {

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyVector2)

  explicit MyVector2(const Vector2<T>& data) : BasicVector<T>(data) {}

 private:
  MyVector2* DoClone() const override {
    return new MyVector2(this->get_value());
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
      // Do not publish at initialization because we want to track publishes
      // from only events of trigger type `kInitialization`.
      simulator.set_publish_at_initialization(false);
      simulator.Initialize();
    }
    {
      // Call `Publish` to test `DoPublish`.
      auto events =
          LeafEventCollection<PublishEvent<T>>::MakeForcedEventCollection();
      system.Publish(*context, *events);
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
      // Call `CalcDiscreteVariableUpdates` to test
      // `DoCalcDiscreteVariableUpdates`.
      auto& state = context->get_mutable_discrete_state();
      auto state_copy = state.Clone();
      system.CalcDiscreteVariableUpdates(*context, state_copy.get());

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
          system.CalcDiscreteVariableUpdates(*context, state_copy.get());
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
}

}  // namespace pydrake
}  // namespace drake
