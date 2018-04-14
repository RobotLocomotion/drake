#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_system.h"

using std::unique_ptr;

namespace drake {

using systems::BasicVector;
using systems::Context;
using systems::LeafSystem;
using systems::Simulator;
using systems::System;

namespace pydrake {
namespace {

using D = double;

// Informs listener when this class is deleted.
class DeleteListenerSystem : public LeafSystem<D> {
 public:
  explicit DeleteListenerSystem(std::function<void()> delete_callback)
      : LeafSystem<D>(), delete_callback_(delete_callback) {}

  ~DeleteListenerSystem() override { delete_callback_(); }

 private:
  std::function<void()> delete_callback_;
};

class DeleteListenerVector : public BasicVector<D> {
 public:
  explicit DeleteListenerVector(std::function<void()> delete_callback)
      : BasicVector(VectorX<D>::Constant(1, 0.)),
        delete_callback_(delete_callback) {}

  ~DeleteListenerVector() override { delete_callback_(); }

 private:
  std::function<void()> delete_callback_;
};

class MoveOnlyType {
 public:
  explicit MoveOnlyType(int x) : x_(x) {}
  MoveOnlyType(MoveOnlyType&&) = default;
  MoveOnlyType& operator=(MoveOnlyType&&) = default;
  MoveOnlyType(const MoveOnlyType&) = delete;
  MoveOnlyType& operator=(const MoveOnlyType&) = delete;
  int x() const { return x_; }
  void set_x(int x) { x_ = x; }
  std::unique_ptr<MoveOnlyType> Clone() const {
    return std::make_unique<MoveOnlyType>(x_);
  }

 private:
  int x_{};
};

struct UnknownType {};

// A simple 2-dimensional subclass of BasicVector for testing.
template <typename D>
class MyVector2 : public BasicVector<D> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyVector2)

  explicit MyVector2(const Vector2<D>& data) : BasicVector<D>(data) {}

 private:
  MyVector2* DoClone() const override {
    return new MyVector2(this->get_value());
  }
};

// Stolen from `Simulator::Initialize`.
// TODO(eric.cousineau): Somehow leverage `Simulator` bits, factoring that
// functionality out.
template <typename D>
void DispatchInitializationEvents(const System<D>& system,
                                  const Context<D>& context) {
  // Process all the initialization events.
  auto init_events = system.AllocateCompositeEventCollection();
  system.GetInitializationEvents(context, init_events.get());
  DRAKE_DEMAND(!init_events->get_unrestricted_update_events().HasEvents());
  DRAKE_DEMAND(!init_events->get_discrete_update_events().HasEvents());
  auto& pub_events = init_events->get_publish_events();
  if (pub_events.HasEvents()) {
    system.Publish(context, pub_events);
  }
}

}  // namespace

PYBIND11_MODULE(test_util, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace systems;

  // Import dependencies.
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  py::class_<DeleteListenerSystem, LeafSystem<D>>(m, "DeleteListenerSystem")
      .def(py::init<std::function<void()>>());
  py::class_<DeleteListenerVector, BasicVector<D>>(m, "DeleteListenerVector")
      .def(py::init<std::function<void()>>());

  py::class_<MoveOnlyType>(m, "MoveOnlyType")
      .def(py::init<int>())
      .def("x", &MoveOnlyType::x)
      .def("set_x", &MoveOnlyType::set_x);
  // Define `Value` instantiation.
  pysystems::AddValueInstantiation<MoveOnlyType>(m);

  // A 2-dimensional subclass of BasicVector.
  py::class_<MyVector2<D>, BasicVector<D>>(m, "MyVector2")
      .def(py::init<const Eigen::Vector2d&>(), py::arg("data"));

  m.def("make_unknown_abstract_value",
        []() { return AbstractValue::Make(UnknownType{}); });

  // Call overrides to ensure a custom Python class can override these methods.
  auto bind_common_scalar_types = [&m](auto dummy) {
    using T = decltype(dummy);

    auto clone_vector = [](const VectorBase<T>& vector) {
      auto copy = std::make_unique<BasicVector<T>>(vector.size());
      copy->SetFrom(vector);
      return copy;
    };

    m.def("call_leaf_system_overrides", [clone_vector](
                                            const LeafSystem<T>& system) {
      py::dict results;
      auto context = system.AllocateContext();
      // Check initialization events.
      DispatchInitializationEvents(system, *context);

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
        auto& state = context->get_mutable_continuous_state();
        ContinuousState<T> state_copy(clone_vector(state.get_vector()));
        system.CalcTimeDerivatives(*context, &state_copy);
      }
      {
        // Call `CalcDiscreteVariableUpdates` to test
        // `DoCalcDiscreteVariableUpdates`.
        auto& state = context->get_mutable_discrete_state();
        DiscreteValues<T> state_copy(clone_vector(state.get_vector()));
        system.CalcDiscreteVariableUpdates(*context, &state_copy);

        // From t=0, return next update time for testing discrete time.
        // If there is an abstract / unrestricted update, this assumes that
        // `dt_discrete < dt_abstract`.
        systems::LeafCompositeEventCollection<T> events;
        results["discrete_next_t"] =
            system.CalcNextUpdateTime(*context, &events);
      }
      return results;
    });

    m.def("call_vector_system_overrides",
          [clone_vector](const VectorSystem<T>& system, Context<T>* context,
                         bool is_discrete, double dt) {
            // While this is not convention, update state first to ensure that
            // our output incorporates it correctly, for testing purposes.
            // TODO(eric.cousineau): Add (Continuous|Discrete)State::Clone().
            if (is_discrete) {
              auto& state = context->get_mutable_discrete_state();
              DiscreteValues<T> state_copy(clone_vector(state.get_vector()));
              system.CalcDiscreteVariableUpdates(*context, &state_copy);
              state.CopyFrom(state_copy);
            } else {
              auto& state = context->get_mutable_continuous_state();
              ContinuousState<T> state_dot(
                  clone_vector(state.get_vector()),
                  state.get_generalized_position().size(),
                  state.get_generalized_velocity().size(),
                  state.get_misc_continuous_state().size());
              system.CalcTimeDerivatives(*context, &state_dot);
              state.SetFromVector(state.CopyToVector() +
                                  dt * state_dot.CopyToVector());
            }
            // Calculate output.
            auto output = system.AllocateOutput();
            system.CalcOutput(*context, output.get());
            return output;
          });
  };
  type_visit(bind_common_scalar_types, pysystems::CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
