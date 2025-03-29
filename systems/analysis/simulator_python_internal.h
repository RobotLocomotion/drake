#pragma once

namespace drake {
namespace systems {

#ifndef DRAKE_DOXYGEN_CXX
template <typename T>
class Simulator;
#endif

namespace internal {

/* Offers a pydrake-specific private interface to the simulator.
The implementation of this header lives in `simulator.cc`.
@tparam_nonsymbolic_scalar */
template <typename T>
class SimulatorPythonInternal {
 public:
  SimulatorPythonInternal() = delete;

  /* Sets a python-specific `monitor` function callback for AdvanceTo(). The
  `monitor` is a plain function function (not std::function) for performance.
  Setting to nullptr removes the monitor. */
  static void set_python_monitor(Simulator<T>* simulator, void (*monitor)());
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
