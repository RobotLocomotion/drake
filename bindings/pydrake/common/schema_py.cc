#include <memory>
#include <vector>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/common/schema/rotation.h"
#include "drake/common/schema/stochastic.h"
#include "drake/common/schema/transform.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(schema, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::schema;
  constexpr auto& doc = pydrake_doc.drake.schema;

  m.doc() = "Bindings for the common.schema package.";

  py::module::import("pydrake.common");
  py::module::import("pydrake.math");

  // Bindings for stochastic.h.

  {
    using Class = Distribution;
    constexpr auto& cls_doc = doc.Distribution;
    py::class_<Class>(m, "Distribution", cls_doc.doc)
        .def("Sample", &Class::Sample, py::arg("generator"), cls_doc.Sample.doc)
        .def("Mean", &Class::Mean, cls_doc.Mean.doc)
        .def("ToSymbolic", &Class::ToSymbolic, cls_doc.ToSymbolic.doc);
  }

  {
    using Class = Deterministic;
    constexpr auto& cls_doc = doc.Deterministic;
    py::class_<Class, Distribution>(m, "Deterministic", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<double>(), py::arg("value"), cls_doc.ctor.doc)
        .def_readwrite("value", &Class::value, cls_doc.value.doc);
  }

  {
    using Class = Gaussian;
    constexpr auto& cls_doc = doc.Gaussian;
    py::class_<Class, Distribution>(m, "Gaussian", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<double, double>(), py::arg("mean"), py::arg("stddev"),
            cls_doc.ctor.doc)
        .def_readwrite("mean", &Class::mean, cls_doc.mean.doc)
        .def_readwrite("stddev", &Class::stddev, cls_doc.stddev.doc);
  }

  {
    using Class = Uniform;
    constexpr auto& cls_doc = doc.Uniform;
    py::class_<Class, Distribution>(m, "Uniform", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<double, double>(), py::arg("min"), py::arg("max"),
            cls_doc.ctor.doc)
        .def_readwrite("min", &Class::min, cls_doc.min.doc)
        .def_readwrite("max", &Class::max, cls_doc.max.doc);
  }

  {
    using Class = UniformDiscrete;
    constexpr auto& cls_doc = doc.UniformDiscrete;
    py::class_<Class, Distribution>(m, "UniformDiscrete", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<std::vector<double>>(), py::arg("values"),
            cls_doc.ctor.doc)
        .def_readwrite("values", &Class::values, cls_doc.values.doc);
  }

  {
    m  // BR
        .def("ToDistribution",
            pydrake::overload_cast_explicit<std::unique_ptr<Distribution>,
                const DistributionVariant&>(ToDistribution),
            py::arg("var"), doc.ToDistribution.doc)
        .def("Sample",
            pydrake::overload_cast_explicit<double, const DistributionVariant&,
                drake::RandomGenerator*>(Sample),
            py::arg("var"), py::arg("generator"),
            doc.Sample.doc_2args_var_generator)
        .def("Mean",
            pydrake::overload_cast_explicit<double, const DistributionVariant&>(
                Mean),
            py::arg("var"), doc.Mean.doc_1args_var)
        .def("ToSymbolic",
            pydrake::overload_cast_explicit<drake::symbolic::Expression,
                const DistributionVariant&>(ToSymbolic),
            py::arg("var"), doc.ToSymbolic.doc_1args_var)
        .def("IsDeterministic",
            pydrake::overload_cast_explicit<bool, const DistributionVariant&>(
                IsDeterministic),
            py::arg("var"), doc.IsDeterministic.doc_1args_var)
        .def("GetDeterministicValue",
            pydrake::overload_cast_explicit<double, const DistributionVariant&>(
                GetDeterministicValue),
            py::arg("var"), doc.GetDeterministicValue.doc_1args_var);
  }

  // TODO(jwnimmer-tri) In the below, we hard-code the template argument `int
  // Size` to be `Eigen::Dynamic`, i.e., we only bind one of the allowed
  // values.  In the future, we should also bind Size == 1 through Size == 6
  // (to match the C++ code).  When we do that, we'll replace the suffix X with
  // the template pattern `[Size]` or `_[Size]`, e.g., `DeterministicVector[3]`
  // or `DeterministicVector_[3]`.

  {
    using Class = DistributionVector;
    constexpr auto& cls_doc = doc.DistributionVector;
    py::class_<Class>(m, "DistributionVector", cls_doc.doc)
        .def("Sample", &Class::Sample, py::arg("generator"), cls_doc.Sample.doc)
        .def("Mean", &Class::Mean, cls_doc.Mean.doc)
        .def("ToSymbolic", &Class::ToSymbolic, cls_doc.ToSymbolic.doc);
  }

  {
    constexpr int size = Eigen::Dynamic;
    using Class = DeterministicVector<size>;
    constexpr auto& cls_doc = doc.DeterministicVector;
    py::class_<Class, DistributionVector>(
        m, "DeterministicVectorX", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const drake::Vector<double, size>&>(), py::arg("value"),
            cls_doc.ctor.doc)
        .def_readwrite("value", &Class::value, cls_doc.value.doc);
  }

  {
    constexpr int size = Eigen::Dynamic;
    using Class = GaussianVector<size>;
    constexpr auto& cls_doc = doc.GaussianVector;
    py::class_<Class, DistributionVector>(m, "GaussianVectorX", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const drake::Vector<double, size>&,
                 const drake::VectorX<double>&>(),
            py::arg("mean"), py::arg("stddev"), cls_doc.ctor.doc)
        .def_readwrite("mean", &Class::mean, cls_doc.mean.doc)
        .def_readwrite("stddev", &Class::stddev, cls_doc.stddev.doc);
  }

  {
    constexpr int size = Eigen::Dynamic;
    using Class = UniformVector<size>;
    constexpr auto& cls_doc = doc.UniformVector;
    py::class_<Class, DistributionVector>(m, "UniformVectorX", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const drake::Vector<double, size>&,
                 const drake::Vector<double, size>&>(),
            py::arg("min"), py::arg("max"), cls_doc.ctor.doc)
        .def_readwrite("min", &Class::min, cls_doc.min.doc)
        .def_readwrite("max", &Class::max, cls_doc.max.doc);
  }

  {
    constexpr int size = Eigen::Dynamic;
    m  // BR
        .def("ToDistributionVector",
            pydrake::overload_cast_explicit<std::unique_ptr<DistributionVector>,
                const DistributionVectorVariant<size>&>(ToDistributionVector),
            py::arg("vec"), doc.ToDistributionVector.doc)
        .def("IsDeterministic",
            pydrake::overload_cast_explicit<bool,
                const DistributionVectorVariant<size>&>(IsDeterministic),
            py::arg("vec"),
            doc.IsDeterministic.doc_1args_constDistributionVectorVariant)
        .def("GetDeterministicValue",
            pydrake::overload_cast_explicit<Eigen::VectorXd,
                const DistributionVectorVariant<size>&>(GetDeterministicValue),
            py::arg("vec"),
            doc.GetDeterministicValue.doc_1args_constDistributionVectorVariant);
  }

  // Bindings for rotation.h.

  {
    using Class = Rotation;
    constexpr auto& cls_doc = doc.Rotation;
    py::class_<Class> cls(m, "Rotation", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const math::RotationMatrixd&>(), cls_doc.ctor.doc_1args)
        .def(py::init<const math::RollPitchYawd&>(), cls_doc.ctor.doc_1args)
        .def("IsDeterministic", &Class::IsDeterministic,
            cls_doc.IsDeterministic.doc)
        .def("GetDeterministicValue", &Class::GetDeterministicValue,
            cls_doc.GetDeterministicValue.doc)
        .def("ToSymbolic", &Class::ToSymbolic, cls_doc.ToSymbolic.doc)
        .def("set_rpy_deg", &Class::set_rpy_deg, py::arg("rpy_deg"),
            cls_doc.set_rpy_deg.doc);
    // TODO(jwnimmer-tri) Bind the .value field.
    // In the meantime, a work-around for read access is to call ToSymbolic or
    // GetDeterministicValue.
  }

  // Bindings for transform.h.

  {
    using Class = Transform;
    constexpr auto& cls_doc = doc.Transform;
    py::class_<Class> cls(m, "Transform", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const math::RigidTransformd&>(), cls_doc.ctor.doc_1args)
        .def("set_rotation_rpy_deg", &Class::set_rotation_rpy_deg,
            py::arg("rpy_deg"), cls_doc.set_rotation_rpy_deg.doc)
        .def("IsDeterministic", &Class::IsDeterministic,
            cls_doc.IsDeterministic.doc)
        .def("GetDeterministicValue", &Class::GetDeterministicValue,
            cls_doc.GetDeterministicValue.doc)
        .def("ToSymbolic", &Class::ToSymbolic, cls_doc.ToSymbolic.doc)
        .def("Sample", &Class::Sample, py::arg("generator"), cls_doc.Sample.doc)
        .def_readwrite(
            "base_frame", &Class::base_frame, cls_doc.base_frame.doc);
    // TODO(jwnimmer-tri) Bind the .translation and .rotation fields.
    // In the meantime, a work-around for read access is to call ToSymbolic or
    // GetDeterministicValue.
  }
}

}  // namespace pydrake
}  // namespace drake
