#include "drake/lcm/drake_lcm_interface.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_DrakeSubscriptionInterface_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::lcm;

  py::class_<DrakeSubscriptionInterface> PyDrakeSubscriptionInterface(
      m, "DrakeSubscriptionInterface",
      R"""(/** 
 * A helper class returned by DrakeLcmInterface::Subscribe() that allows for 
 * (possibly automatic) unsubscription and/or queue capacity control.  Refer to 
 * that method for additional details. 
 * 
 * Instance of this object are always stored in `std::shared_ptr` to manage 
 * them as resources.  When a particular DrakeLcmInterface implementation does 
 * not support subscription controls, the managed pointer will be `nullptr` 
 * instead of an instance of this object. 
 * 
 * To unsubscribe, induce a call to the %DrakeSubscriptionInterface destructor 
 * by bringing the `std::shared_ptr` use count to zero.  That usually means 
 * either a call to `subscription.reset()` or by allowing it to go out of 
 * scope. 
 * 
 * To *disable* unsubscription so that the pointer loss *never* causes 
 * unsubscription, call `subscription->set_unsubscribe_on_delete(false)`. 
 * To *enable* unsubscription, set it to `true`.  Which choice is active by 
 * default is specified by whatever method returns this object. 
 */)""");

  PyDrakeSubscriptionInterface
      .def("set_queue_capacity",
           static_cast<void (DrakeSubscriptionInterface::*)(int)>(
               &DrakeSubscriptionInterface::set_queue_capacity),
           py::arg("capacity"),
           R"""(/** 
 * Sets this subscription's queue depth to store messages inbetween calls to 
 * DrakeLcmInterface::HandleSubscriptions.  When the queue becomes full, new 
 * received messages will be discarded.  The default depth is 1. 
 * 
 * @warning The memq:// LCM URL does not support per-channel queues, so this 
 * method has no effect when memq is being used, e.g., in Drake unit tests. 
 */)""")
      .def("set_unsubscribe_on_delete",
           static_cast<void (DrakeSubscriptionInterface::*)(bool)>(
               &DrakeSubscriptionInterface::set_unsubscribe_on_delete),
           py::arg("enabled"),
           R"""(/** 
 * Sets whether or not the subscription on DrakeLcmInterface will be 
 * terminated when this object is deleted.  It is permitted to call this 
 * method many times, with a new `enabled` value each time. 
 */)""")

      ;
}
