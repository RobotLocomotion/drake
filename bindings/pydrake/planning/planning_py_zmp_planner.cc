#include "drake/bindings/generated_docstrings/planning_locomotion.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/locomotion/zmp_planner.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningZmpPlanner(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning_locomotion.drake.planning;

  {
    using Class = ZmpPlanner;
    constexpr auto& cls_doc = doc.ZmpPlanner;
    auto cls = py::class_<Class>(m, "ZmpPlanner", cls_doc.doc)
                   .def(py::init<>(), cls_doc.ctor.doc);
    cls.def("Plan", &Class::Plan, py::arg("zmp_d"), py::arg("x0"),
           py::arg("height"), py::arg("gravity") = 9.81,
           py::arg("Qy") = Eigen::Matrix2d::Identity(),
           py::arg("R") = 0.1 * Eigen::Matrix2d::Identity(), cls_doc.ctor.doc)
        .def("has_planned", &Class::has_planned, cls_doc.has_planned.doc)
        .def("ComputeOptimalCoMdd", &Class::ComputeOptimalCoMdd,
            py::arg("time"), py::arg("x"), cls_doc.ComputeOptimalCoMdd.doc)
        .def("comdd_to_cop", &Class::comdd_to_cop, py::arg("x"), py::arg("u"),
            cls_doc.comdd_to_cop.doc)
        .def("get_A", &Class::get_A, cls_doc.get_A.doc)
        .def("get_B", &Class::get_B, cls_doc.get_B.doc)
        .def("get_C", &Class::get_C, cls_doc.get_C.doc)
        .def("get_D", &Class::get_D, cls_doc.get_D.doc)
        .def("get_Qy", &Class::get_Qy, cls_doc.get_Qy.doc)
        .def("get_R", &Class::get_R, cls_doc.get_R.doc)
        .def("get_desired_zmp",
            overload_cast_explicit<Eigen::Vector2d, double>(
                &Class::get_desired_zmp),
            py::arg("time"), cls_doc.get_desired_zmp.doc_at_time)
        .def("get_nominal_com",
            overload_cast_explicit<Eigen::Vector2d, double>(
                &Class::get_nominal_com),
            py::arg("time"), cls_doc.get_nominal_com.doc_at_time)
        .def("get_nominal_comd",
            overload_cast_explicit<Eigen::Vector2d, double>(
                &Class::get_nominal_comd),
            py::arg("time"), cls_doc.get_nominal_comd.doc_at_time)
        .def("get_nominal_comdd",
            overload_cast_explicit<Eigen::Vector2d, double>(
                &Class::get_nominal_comdd),
            py::arg("time"), cls_doc.get_nominal_comdd.doc_at_time)
        .def("get_final_desired_zmp", &Class::get_final_desired_zmp,
            cls_doc.get_final_desired_zmp.doc)
        .def("get_desired_zmp",
            overload_cast_explicit<
                const trajectories::PiecewisePolynomial<double>&>(
                &Class::get_desired_zmp),
            py_rvp::copy, cls_doc.get_desired_zmp.doc)
        .def("get_nominal_com",
            overload_cast_explicit<const trajectories::
                    ExponentialPlusPiecewisePolynomial<double>&>(
                &Class::get_nominal_com),
            py_rvp::copy, cls_doc.get_nominal_com.doc)
        .def("get_nominal_comd",
            overload_cast_explicit<const trajectories::
                    ExponentialPlusPiecewisePolynomial<double>&>(
                &Class::get_nominal_comd),
            py_rvp::copy, cls_doc.get_nominal_comd.doc)
        .def("get_nominal_comdd",
            overload_cast_explicit<const trajectories::
                    ExponentialPlusPiecewisePolynomial<double>&>(
                &Class::get_nominal_comdd),
            py_rvp::copy, cls_doc.get_nominal_comdd.doc)
        .def("get_Vxx", &Class::get_Vxx, cls_doc.get_Vxx.doc)
        .def("get_Vx",
            overload_cast_explicit<const Eigen::Vector4d, double>(
                &Class::get_Vx),
            py::arg("time"), cls_doc.get_Vx.doc_at_time)
        .def("get_Vx",
            overload_cast_explicit<const trajectories::
                    ExponentialPlusPiecewisePolynomial<double>&>(
                &Class::get_Vx),
            py_rvp::copy, cls_doc.get_Vx.doc);
    DefCopyAndDeepCopy(&cls);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
