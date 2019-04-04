#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

// Forward declare for friendship below. Only System<T> may ever create
// a SystemOutput<T>.
template <typename T> class System;

/** Conveniently stores a snapshot of the values of every output port of
a System. There is framework support for allocating the right types and filling
them in but otherwise this is not used internally. Note that there is never any
live connection between a SystemOutput object and the System whose output values
it has captured.

A `SystemOutput<T>` object can only be obtained using
`System<T>::AllocateOutput()` or by copying an existing %SystemOutput object.

@tparam T The type of the output data. Must be a valid Eigen scalar. */
template <typename T>
class SystemOutput {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(SystemOutput);

  ~SystemOutput();

  /** Returns the number of output ports specified for this %SystemOutput
  during allocation. */
  int num_ports() const { return static_cast<int>(port_values_.size()); }

  // TODO(sherm1) All of these should return references. We don't need to
  // support missing entries.

  /** Returns the last-saved value of output port `index` as an AbstractValue.
  This works for any output port regardless of it actual type. */
  const AbstractValue* get_data(int index) const {
    DRAKE_ASSERT(0 <= index && index < num_ports());
    return port_values_[index].get();
  }

  /** Returns the last-saved value of output port `index` as a `BasicVector<T>`,
  although the actual concrete type is preserved from the actual output port.
  @throws std::bad_cast if the port is not vector-valued. */
  const BasicVector<T>* get_vector_data(int index) const {
    DRAKE_ASSERT(0 <= index && index < num_ports());
    return &port_values_[index]->template get_value<BasicVector<T>>();
  }

  /** (Advanced) Returns mutable access to an AbstractValue object that is
  suitable for holding the value of output port `index` of the allocating
  System. This works for any output port regardless of it actual type. Most
  users should just call `System<T>::CalcOutputs()` to get all the output
  port values at once. */
  AbstractValue* GetMutableData(int index) {
    DRAKE_ASSERT(0 <= index && index < num_ports());
    return port_values_[index].get_mutable();
  }

  /** (Advanced) Returns mutable access to a `BasicVector<T>` object that is
  suitable for holding the value of output port `index` of the allocating
  System. The object's concrete type is preserved from the output port. Most
  users should just call `System<T>::CalcOutputs()` to get all the output
  port values at once.
  @throws std::bad_cast if the port is not vector-valued. */
  BasicVector<T>* GetMutableVectorData(int index) {
    DRAKE_ASSERT(0 <= index && index < num_ports());
    return &port_values_[index]->template get_mutable_value<BasicVector<T>>();
  }

#ifndef DRAKE_DOXYGEN_CXX
  DRAKE_DEPRECATED("2019-07-01", "Use num_ports() instead.")
  int get_num_ports() const { return num_ports(); }
#endif

 private:
  friend class System<T>;
  friend class SystemOutputTest;

  SystemOutput();

  // Add a suitable object to hold values for the next output port.
  void add_port(std::unique_ptr<AbstractValue> model_value) {
    port_values_.emplace_back(std::move(model_value));
  }

  std::vector<copyable_unique_ptr<AbstractValue>> port_values_;
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T> SystemOutput<T>::SystemOutput() = default;
template <typename T> SystemOutput<T>::~SystemOutput() = default;
DRAKE_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(SystemOutput);

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemOutput)
