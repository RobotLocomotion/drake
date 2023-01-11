#include <memory>
#include <vector>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/serialize_pybind.h"
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
    py::class_<Class, Distribution> cls(m, "Deterministic", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<double>(), py::arg("value"), cls_doc.ctor.doc);
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Gaussian;
    constexpr auto& cls_doc = doc.Gaussian;
    py::class_<Class, Distribution> cls(m, "Gaussian", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<double, double>(), py::arg("mean"), py::arg("stddev"),
            cls_doc.ctor.doc);
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Uniform;
    constexpr auto& cls_doc = doc.Uniform;
    py::class_<Class, Distribution> cls(m, "Uniform", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<double, double>(), py::arg("min"), py::arg("max"),
            cls_doc.ctor.doc);
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = UniformDiscrete;
    constexpr auto& cls_doc = doc.UniformDiscrete;
    py::class_<Class, Distribution> cls(m, "UniformDiscrete", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<std::vector<double>>(), py::arg("values"),
            cls_doc.ctor.doc);
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefCopyAndDeepCopy(&cls);
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

  {
    // We need to bind these to placate some runtime type checks, but we should
    // not expose them to users.
    py::class_<schema::internal::InvalidVariantSelection<Deterministic>>(
        m, "_InvalidVariantSelectionDeterministic");
    py::class_<schema::internal::InvalidVariantSelection<Gaussian>>(
        m, "_InvalidVariantSelectionGaussian");
    py::class_<schema::internal::InvalidVariantSelection<Uniform>>(
        m, "_InvalidVariantSelectionUniform");
  }

  {
    using Class = DistributionVector;
    constexpr auto& cls_doc = doc.DistributionVector;
    py::class_<Class>(m, "DistributionVector", cls_doc.doc)
        .def("Sample", &Class::Sample, py::arg("generator"), cls_doc.Sample.doc)
        .def("Mean", &Class::Mean, cls_doc.Mean.doc)
        .def("ToSymbolic", &Class::ToSymbolic, cls_doc.ToSymbolic.doc);
  }

  // Here, we bind all supported values of the 'Size' template argument.
  //
  // TODO(jwnimmer-tri) For convenience, it would be nice if we could also
  // offer a type array, e.g., 'DeterministicVector[i]' to look up a fixed-size
  // vector type.
  auto bind_distribution_vectors = [&]<int Size>(
                                       std::integral_constant<int, Size>) {
    const std::string size_suffix =
        (Size == Eigen::Dynamic) ? std::string("X") : std::to_string(Size);

    {
      using Class = DeterministicVector<Size>;
      constexpr auto& cls_doc = doc.DeterministicVector;
      py::class_<Class, DistributionVector> cls(
          m, ("DeterministicVector" + size_suffix).c_str(), cls_doc.doc);
      cls  // BR
          .def(py::init<>(), cls_doc.ctor.doc)
          .def(py::init<const Class&>(), py::arg("other"))
          .def(py::init<const drake::Vector<double, Size>&>(), py::arg("value"),
              cls_doc.ctor.doc);
      DefAttributesUsingSerialize(&cls, cls_doc);
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = GaussianVector<Size>;
      constexpr auto& cls_doc = doc.GaussianVector;
      py::class_<Class, DistributionVector> cls(
          m, ("GaussianVector" + size_suffix).c_str(), cls_doc.doc);
      cls  // BR
          .def(py::init<>(), cls_doc.ctor.doc)
          .def(py::init<const Class&>(), py::arg("other"))
          .def(py::init<const drake::Vector<double, Size>&,
                   const drake::VectorX<double>&>(),
              py::arg("mean"), py::arg("stddev"), cls_doc.ctor.doc);
      DefAttributesUsingSerialize(&cls, cls_doc);
      DefCopyAndDeepCopy(&cls);
    }

    {
      using Class = UniformVector<Size>;
      constexpr auto& cls_doc = doc.UniformVector;
      py::class_<Class, DistributionVector> cls(
          m, ("UniformVector" + size_suffix).c_str(), cls_doc.doc);
      cls  // BR
          .def(py::init<>(), cls_doc.ctor.doc)
          .def(py::init<const Class&>(), py::arg("other"))
          .def(py::init<const drake::Vector<double, Size>&,
                   const drake::Vector<double, Size>&>(),
              py::arg("min"), py::arg("max"), cls_doc.ctor.doc);
      DefAttributesUsingSerialize(&cls, cls_doc);
      DefCopyAndDeepCopy(&cls);
    }

    {
      m  // BR
          .def("ToDistributionVector",
              pydrake::overload_cast_explicit<
                  std::unique_ptr<DistributionVector>,
                  const DistributionVectorVariant<Size>&>(ToDistributionVector),
              py::arg("vec"), doc.ToDistributionVector.doc)
          .def("IsDeterministic",
              pydrake::overload_cast_explicit<bool,
                  const DistributionVectorVariant<Size>&>(IsDeterministic),
              py::arg("vec"),
              doc.IsDeterministic.doc_1args_constDistributionVectorVariant)
          .def("GetDeterministicValue",
              pydrake::overload_cast_explicit<Eigen::VectorXd,
                  const DistributionVectorVariant<Size>&>(
                  GetDeterministicValue),
              py::arg("vec"),
              doc.GetDeterministicValue
                  .doc_1args_constDistributionVectorVariant);
    }
  };  // NOLINT
  bind_distribution_vectors(std::integral_constant<int, Eigen::Dynamic>{});
  bind_distribution_vectors(std::integral_constant<int, 1>{});
  bind_distribution_vectors(std::integral_constant<int, 2>{});
  bind_distribution_vectors(std::integral_constant<int, 3>{});
  bind_distribution_vectors(std::integral_constant<int, 4>{});
  bind_distribution_vectors(std::integral_constant<int, 5>{});
  bind_distribution_vectors(std::integral_constant<int, 6>{});

  // Bindings for rotation.h.

  {
    using Class = Rotation;
    constexpr auto& cls_doc = doc.Rotation;
    py::class_<Class> cls(m, "Rotation", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
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
    DefCopyAndDeepCopy(&cls);
  }

  // Bindings for transform.h.

  {
    using Class = Transform;
    constexpr auto& cls_doc = doc.Transform;
    py::class_<Class> cls(m, "Transform", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
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
    DefCopyAndDeepCopy(&cls);
  }
}

}  // namespace pydrake
}  // namespace drake
