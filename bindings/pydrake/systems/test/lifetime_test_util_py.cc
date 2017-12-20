#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace py = pybind11;

using std::unique_ptr;
using drake::VectorX;
using drake::systems::BasicVector;
using drake::systems::ConstantVectorSource;

using T = double;

namespace {

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

}  // namespace

PYBIND11_MODULE(lifetime_test_util, m) {
  // Import dependencies.
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  py::class_<DeleteListenerSystem, ConstantVectorSource<T>>(
      m, "DeleteListenerSystem")
    .def(py::init<std::function<void()>>());
  py::class_<DeleteListenerVector, BasicVector<T>>(
      m, "DeleteListenerVector")
    .def(py::init<std::function<void()>>());
}
