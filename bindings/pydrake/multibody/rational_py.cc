#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace pydrake {
namespace {

PYBIND11_MODULE(rational, m) {
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "RationalForwardKinematics module";
  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  {
    using Class = RationalForwardKinematics;
    constexpr auto& cls_doc = doc.RationalForwardKinematics;
    py::class_<Class>(m, "RationalForwardKinematics")
        .def(py::init<const MultibodyPlant<double>*>(), py::arg("plant"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>()  // BR
            )
        .def("CalcBodyPoseAsMultilinearPolynomial",
            &Class::CalcBodyPoseAsMultilinearPolynomial, py::arg("q_star"),
            py::arg("body_index"), py::arg("expressed_body_index"), cls_doc.ctor.doc)
        .def("ConvertMultilinearPolynomialToRationalFunction",
            &Class::ConvertMultilinearPolynomialToRationalFunction, py::arg("e"), cls_doc.ctor.doc)
        .def("plant", &Class::plant, cls_doc.ctor.doc)
        .def("s", &Class::s, cls_doc.ctor.doc)
//        .def("ComputeSValue",
//             &Class::ComputeSValue<const Eigen::VectorXd&,  const Eigen::Ref<const Eigen::VectorXd>>,
//            py::arg("q_val"), py::arg("q_star_val")
//            //             cls_doc.CalcLinkPoses
//            )

//        .def("ComputeQValue",
//            overload_cast_explicit<Eigen::VectorXd,
//                const Eigen::Ref<const Eigen::VectorXd>&,
//                const Eigen::Ref<const Eigen::VectorXd>&>(
//                &Class::ComputeQValue),
//            py::arg("s_val"), py::arg("q_star_val")
//            //             cls_doc.CalcLinkPoses
//        )
        ;
  }
}
}  // namespace
}  // namespace pydrake
}  // namespace drake
