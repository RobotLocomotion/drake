#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace pydrake {
namespace {

template <typename T>
void DoPoseDeclaration(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  using Class = multibody::RationalForwardKinematics::Pose<T>;
  auto cls = DefineTemplateClassWithDefault<Class>(
      m, "RationalForwardKinematicsPose", param);
  cls.def("position", [](const Class& self) { return self.position; })
      .def("rotation", [](const Class& self) { return self.rotation; });

  DefCopyAndDeepCopy(&cls);
}

PYBIND11_MODULE(rational, m) {
  using drake::multibody::MultibodyPlant;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "RationalForwardKinematics module";
  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  {
    using Class = drake::multibody::RationalForwardKinematics;
    constexpr auto& cls_doc = doc.RationalForwardKinematics;
    py::class_<Class>(m, "RationalForwardKinematics")
        .def(py::init<const MultibodyPlant<double>*>(), py::arg("plant"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>()  // BR
            )
        .def("CalcBodyPoseAsMultilinearPolynomial",
            &Class::CalcBodyPoseAsMultilinearPolynomial, py::arg("q_star"),
            py::arg("body_index"), py::arg("expressed_body_index"),
            cls_doc.CalcBodyPoseAsMultilinearPolynomial.doc)
        .def("ConvertMultilinearPolynomialToRationalFunction",
            &Class::ConvertMultilinearPolynomialToRationalFunction,
            py::arg("e"),
            cls_doc.ConvertMultilinearPolynomialToRationalFunction.doc)
        .def("plant", &Class::plant, py_rvp::reference_internal,
            cls_doc.plant.doc)
        .def("s", &Class::s, py_rvp::copy, cls_doc.s.doc)
        .def(
            "ComputeSValue",
            [](const Class& self, const Eigen::VectorXd& q_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
                bool angles_wrap_to_inf) {
              return self.ComputeSValue(q_val, q_star_val, angles_wrap_to_inf);
            },
            py::arg("q_val"), py::arg("q_star_val"),
            py::arg("angles_wrap_to_inf") = false, cls_doc.ComputeSValue.doc)
        .def(
            "ComputeSValue",
            [](const Class& self, const AutoDiffVecXd& q_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
              return self.ComputeSValue(q_val, q_star_val);
            },
            py::arg("q_val"), py::arg("q_star_val"), cls_doc.ComputeSValue.doc)
        .def(
            "ComputeSValue",
            [](const Class& self, const VectorX<symbolic::Expression>& q_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
              return self.ComputeSValue(q_val, q_star_val);
            },
            py::arg("q_val"), py::arg("q_star_val"), cls_doc.ComputeSValue.doc)
        .def(
            "ComputeQValue",
            [](const Class& self, const Eigen::VectorXd& s_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
              return self.ComputeQValue(s_val, q_star_val);
            },
            py::arg("s_val"), py::arg("q_star_val"), cls_doc.ComputeQValue.doc)
        .def(
            "ComputeQValue",
            [](const Class& self, const AutoDiffVecXd& s_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
              return self.ComputeQValue(s_val, q_star_val);
            },
            py::arg("s_val"), py::arg("q_star_val"), cls_doc.ComputeQValue.doc)
        .def(
            "ComputeQValue",
            [](const Class& self, const VectorX<symbolic::Expression>& s_val,
                const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
              return self.ComputeQValue(s_val, q_star_val);
            },
            py::arg("s_val"), py::arg("q_star_val"), cls_doc.ComputeQValue.doc);
  }

  type_pack<symbolic::Polynomial, symbolic::RationalFunction> sym_pack;
  type_visit([m](auto dummy) { DoPoseDeclaration(m, dummy); }, sym_pack);
}
}  // namespace
}  // namespace pydrake
}  // namespace drake
