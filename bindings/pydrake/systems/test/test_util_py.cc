#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::unique_ptr;

namespace drake {

using systems::BasicVector;
using systems::ConstantVectorSource;

namespace pydrake {
namespace {

using T = double;

// Informs listener when this class is deleted.
class DeleteListenerSystem : public ConstantVectorSource<T> {
 public:
  explicit DeleteListenerSystem(std::function<void()> delete_callback)
    : ConstantVectorSource(VectorX<T>::Constant(1, 0.)),
      delete_callback_(delete_callback) {}

  ~DeleteListenerSystem() override {
    delete_callback_();
  }
 private:
  std::function<void()> delete_callback_;
};

class DeleteListenerVector : public BasicVector<T> {
 public:
  explicit DeleteListenerVector(std::function<void()> delete_callback)
    : BasicVector(VectorX<T>::Constant(1, 0.)),
      delete_callback_(delete_callback) {}

  ~DeleteListenerVector() override {
    delete_callback_();
  }
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

  py::class_<DeleteListenerSystem, ConstantVectorSource<T>>(
      m, "DeleteListenerSystem")
    .def(py::init<std::function<void()>>());
  py::class_<DeleteListenerVector, BasicVector<T>>(
      m, "DeleteListenerVector")
    .def(py::init<std::function<void()>>());

  py::class_<MoveOnlyType>(m, "MoveOnlyType")
    .def(py::init<int>())
    .def("x", &MoveOnlyType::x)
    .def("set_x", &MoveOnlyType::set_x);
  // Define `Value` instantiation.
  pysystems::AddValueInstantiation<MoveOnlyType>(m);

  // A 2-dimensional subclass of BasicVector.
  py::class_<MyVector2<T>, BasicVector<T>>(m, "MyVector2")
      .def(py::init<const Eigen::Vector2d&>(), py::arg("data"));

  m.def("make_unknown_abstract_value", []() {
    return AbstractValue::Make(UnknownType{});
  });

  // Call overrides to ensure a custom Python class can override these methods.

  auto clone_vector = [](const VectorBase<T>& vector) {
    auto copy = std::make_unique<BasicVector<T>>(vector.size());
    copy->SetFrom(vector);
    return copy;
  };

  m.def("call_leaf_system_overrides", [clone_vector](
      const LeafSystem<T>& system) {
    py::dict results;
    auto context = system.AllocateContext();
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
      systems::LeafCompositeEventCollection<double> events;
      results["discrete_next_t"] = system.CalcNextUpdateTime(*context, &events);
    }
    return results;
  });

  m.def("call_vector_system_overrides", [clone_vector](
      const VectorSystem<T>& system, Context<T>* context,
      bool is_discrete, double dt) {
    // While this is not convention, update state first to ensure that our
    // output incorporates it correctly, for testing purposes.
    // TODO(eric.cousineau): Add (Continuous|Discrete)State::Clone().
    if (is_discrete) {
      auto& state = context->get_mutable_discrete_state();
      DiscreteValues<T> state_copy(
          clone_vector(state.get_vector()));
      system.CalcDiscreteVariableUpdates(
          *context, &state_copy);
      state.SetFrom(state_copy);
    } else {
      auto& state = context->get_mutable_continuous_state();
      ContinuousState<T> state_dot(
          clone_vector(state.get_vector()),
          state.get_generalized_position().size(),
          state.get_generalized_velocity().size(),
          state.get_misc_continuous_state().size());
      system.CalcTimeDerivatives(*context, &state_dot);
      state.SetFromVector(
          state.CopyToVector() + dt * state_dot.CopyToVector());
    }
    // Calculate output.
    auto output = system.AllocateOutput(*context);
    system.CalcOutput(*context, output.get());
    return output;
  });
}

}  // namespace pydrake
}  // namespace drake
