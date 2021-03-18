#include "drake/common/symbolic.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

namespace py = pybind11;
void apb11_pydrake_Environment_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::symbolic;

  py::class_<Environment> PyEnvironment(
      m, "Environment",
      R"""(/** Represents a symbolic environment (mapping from a variable to a value). 
 * 
 * This class is used when we evaluate symbolic expressions or formulas which 
 * include unquantified (free) variables. Here are examples: 
 * 
 * \code{.cpp} 
 *   const Variable var_x{"x"}; 
 *   const Variable var_y{"y"}; 
 *   const Expression x{var_x}; 
 *   const Expression y{var_x}; 
 *   const Expression e1{x + y}; 
 *   const Expression e2{x - y}; 
 *   const Formula f{e1 > e2}; 
 * 
 *   // env maps var_x to 2.0 and var_y to 3.0 
 *   const Environment env{{var_x, 2.0}, {var_y, 3.0}}; 
 * 
 *   const double res1 = e1.Evaluate(env);  // x + y => 2.0 + 3.0 =>  5.0 
 *   const double res2 = e2.Evaluate(env);  // x - y => 2.0 - 3.0 => -1.0 
 *   const bool res = f.Evaluate(env);  // x + y > x - y => 5.0 >= -1.0 => True 
 * \endcode 
 * 
 * Note that it is not allowed to have a dummy variable in an environment. It 
 * throws std::runtime_error for the attempts to create an environment with a 
 * dummy variable, to insert a dummy variable to an existing environment, or to 
 * take a reference to a value mapped to a dummy variable. See the following 
 * examples. 
 * 
 * \code{.cpp} 
 *   Variable    var_dummy{};           // OK to have a dummy variable 
 *   Environment e1{var_dummy};         // throws std::runtime_error exception 
 *   Environment e2{{var_dummy, 1.0}};  // throws std::runtime_error exception 
 *   Environment e{}; 
 *   e.insert(var_dummy, 1.0);          // throws std::runtime_error exception 
 *   e[var_dummy] = 3.0;                // throws std::runtime_error exception 
 * \endcode 
 * 
 */)""");

  PyEnvironment.def(py::init<Environment const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<
               ::std::initializer_list<std::pair<const Variable, double>>>(),
           py::arg("init"))
      .def(py::init<::std::initializer_list<Variable>>(), py::arg("vars"))
      .def(py::init<Environment::map>(), py::arg("m"))
      .def("begin",
           static_cast<Environment::iterator (Environment::*)()>(
               &Environment::begin),
           R"""(/** Returns an iterator to the beginning. */)""")
      .def("begin",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::begin),
           R"""(/** Returns a const iterator to the beginning. */)""")
      .def("cbegin",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::cbegin),
           R"""(/** Returns a const iterator to the beginning. */)""")
      .def("cend",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::cend),
           R"""(/** Returns a const iterator to the end. */)""")
      .def(
          "domain",
          static_cast<Variables (Environment::*)() const>(&Environment::domain),
          R"""(/** Returns the domain of this environment. */)""")
      .def("empty",
           static_cast<bool (Environment::*)() const>(&Environment::empty),
           R"""(/** Checks whether the container is empty.  */)""")
      .def("end",
           static_cast<Environment::iterator (Environment::*)()>(
               &Environment::end),
           R"""(/** Returns an iterator to the end. */)""")
      .def("end",
           static_cast<Environment::const_iterator (Environment::*)() const>(
               &Environment::end),
           R"""(/** Returns a const iterator to the end. */)""")
      .def("find",
           static_cast<Environment::iterator (Environment::*)(
               Environment::key_type const &)>(&Environment::find),
           py::arg("key"), R"""(/** Finds element with specific key. */)""")
      .def("find",
           static_cast<Environment::const_iterator (Environment::*)(
               Environment::key_type const &) const>(&Environment::find),
           py::arg("key"), R"""(/** Finds element with specific key. */)""")
      .def("insert",
           static_cast<void (Environment::*)(Environment::key_type const &,
                                             Environment::mapped_type const &)>(
               &Environment::insert),
           py::arg("key"), py::arg("elem"),
           R"""(/** Inserts a pair (@p key, @p elem). */)""")
      .def("insert",
           [](Environment &self,
              ::Eigen::Ref<const Eigen::Matrix<Variable, -1, -1, 0, -1, -1>, 0,
                           Eigen::OuterStride<-1>> const &keys,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                           Eigen::OuterStride<-1>> const &elements) {
             return self.insert(keys, elements);
           })
      .def("size",
           static_cast<::size_t (Environment::*)() const>(&Environment::size),
           R"""(/** Returns the number of elements. */)""")
      .def("to_string",
           static_cast<::std::string (Environment::*)() const>(
               &Environment::to_string),
           R"""(/** Returns string representation. */)""")

      .def(
          "__str__",
          +[](Environment const &env) {
            std::ostringstream oss;
            oss << env;
            std::string s(oss.str());

            return s;
          })
      .def(
          "__getitem__",
          static_cast<Environment::mapped_type &(
              Environment::*)(Environment::key_type const &)>(
              &Environment::operator[]),
          py::arg("key"),
          R"""(/** Returns a reference to the value that is mapped to a key equivalent to 
 *  @p key, performing an insertion if such key does not already exist. 
 */)""")
      .def(
          "__getitem__",
          static_cast<Environment::mapped_type const &(
              Environment::*)(Environment::key_type const &)const>(
              &Environment::operator[]),
          py::arg("key"),
          R"""(/** As above, but returns a constref and does not perform an insertion 
 * (throwing a runtime error instead) if the key does not exist. */)""");
}
