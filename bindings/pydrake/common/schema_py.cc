#include <memory>
#include <vector>

#include "pybind11/eval.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/common/schema/rotation.h"
#include "drake/common/schema/stochastic.h"
#include "drake/common/schema/transform.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineModuleSchema(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::schema;
  constexpr auto& doc = pydrake_doc.drake.schema;

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
  auto bind_distribution_vectors = [&]<int Size>(
                                       std::integral_constant<int, Size>) {
    // Prepare our python template argument (i.e., the Size).
    // For Eigen::Dynamic, we use None (instead of the magical "-1").
    py::tuple py_param;
    if constexpr (Size == Eigen::Dynamic) {
      py_param = py::make_tuple(py::none());
    } else {
      py_param = py::make_tuple(Size);
    }

    {
      using Class = DeterministicVector<Size>;
      constexpr auto& cls_doc = doc.DeterministicVector;
      py::class_<Class, DistributionVector> cls(
          m, TemporaryClassName<Class>().c_str(), cls_doc.doc);
      AddTemplateClass(m, "DeterministicVector", cls, py_param);
      if constexpr (Size == Eigen::Dynamic) {
        m.attr("DeterministicVectorX") = cls;
      }
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
          m, TemporaryClassName<Class>().c_str(), cls_doc.doc);
      AddTemplateClass(m, "GaussianVector", cls, py_param);
      if constexpr (Size == Eigen::Dynamic) {
        m.attr("GaussianVectorX") = cls;
      }
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
          m, TemporaryClassName<Class>().c_str(), cls_doc.doc);
      AddTemplateClass(m, "UniformVector", cls, py_param);
      if constexpr (Size == Eigen::Dynamic) {
        m.attr("UniformVectorX") = cls;
      }
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
    // To bind nested serializable structs without errors, we declare the outer
    // struct first, then bind its inner structs, then bind the outer struct.
    using Class = Rotation;
    constexpr auto& cls_doc = doc.Rotation;
    py::class_<Class> cls(m, "Rotation", cls_doc.doc);

    // Inner structs.
    {
      using Inner = Class::Identity;
      py::class_<Inner> inner(cls, "Identity", cls_doc.Identity.doc);
      inner.def(py::init<const Inner&>(), py::arg("other"));
      inner.def(ParamInit<Inner>());
      DefAttributesUsingSerialize(&inner, cls_doc.Identity);
      DefReprUsingSerialize(&inner);
      DefCopyAndDeepCopy(&inner);
    }
    {
      using Inner = Class::Rpy;
      py::class_<Inner> inner(cls, "Rpy", cls_doc.Rpy.doc);
      inner.def(py::init<const Inner&>(), py::arg("other"));
      inner.def(ParamInit<Inner>());
      DefAttributesUsingSerialize(&inner, cls_doc.Rpy);
      DefReprUsingSerialize(&inner);
      DefCopyAndDeepCopy(&inner);
    }
    {
      using Inner = Class::AngleAxis;
      py::class_<Inner> inner(cls, "AngleAxis", cls_doc.AngleAxis.doc);
      inner.def(py::init<const Inner&>(), py::arg("other"));
      inner.def(ParamInit<Inner>());
      DefAttributesUsingSerialize(&inner, cls_doc.AngleAxis);
      DefReprUsingSerialize(&inner);
      DefCopyAndDeepCopy(&inner);
    }
    {
      using Inner = Class::Uniform;
      py::class_<Inner> inner(cls, "Uniform", cls_doc.Uniform.doc);
      inner.def(py::init<const Inner&>(), py::arg("other"));
      inner.def(ParamInit<Inner>());
      DefAttributesUsingSerialize(&inner, cls_doc.Uniform);
      DefReprUsingSerialize(&inner);
      DefCopyAndDeepCopy(&inner);
    }

    // Now we can finish binding the outermost struct (Rotation).
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const math::RotationMatrixd&>(), cls_doc.ctor.doc_1args)
        .def(py::init<const math::RollPitchYawd&>(), cls_doc.ctor.doc_1args)
        .def(ParamInit<Class>())
        .def("IsDeterministic", &Class::IsDeterministic,
            cls_doc.IsDeterministic.doc)
        .def("GetDeterministicValue", &Class::GetDeterministicValue,
            cls_doc.GetDeterministicValue.doc)
        .def("ToSymbolic", &Class::ToSymbolic, cls_doc.ToSymbolic.doc)
        .def("set_rpy_deg", &Class::set_rpy_deg, py::arg("rpy_deg"),
            cls_doc.set_rpy_deg.doc);
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);

    // To support the atypical C++ implementation of Transform::Serialize, we
    // need to support attribute operations on Rotation that should actually
    // apply to the Rotation::value member field instead. We'll achieve that by
    // adding special-cases to getattr and setattr.
    cls.def("__getattr__", [](const Class& self, py::str name) -> py::object {
      if (std::holds_alternative<Rotation::Rpy>(self.value)) {
        const std::string name_cxx = name;
        if (name_cxx == "deg") {
          py::object self_py = py::cast(self, py_rvp::reference);
          return self_py.attr("value").attr(name);
        }
      }
      if (std::holds_alternative<Rotation::AngleAxis>(self.value)) {
        const std::string name_cxx = name;
        if ((name_cxx == "angle_deg") || (name_cxx == "axis")) {
          py::object self_py = py::cast(self, py_rvp::reference);
          return self_py.attr("value").attr(name);
        }
      }
      return py::eval("object.__getattr__")(self, name);
    });
    cls.def("__setattr__", [](Class& self, py::str name, py::object value) {
      if (std::holds_alternative<Rotation::Rpy>(self.value)) {
        const std::string name_cxx = name;
        if (name_cxx == "deg") {
          py::object self_py = py::cast(self, py_rvp::reference);
          self_py.attr("value").attr(name) = value;
          return;
        }
      }
      if (std::holds_alternative<Rotation::AngleAxis>(self.value)) {
        const std::string name_cxx = name;
        if ((name_cxx == "angle_deg") || (name_cxx == "axis")) {
          py::object self_py = py::cast(self, py_rvp::reference);
          self_py.attr("value").attr(name) = value;
          return;
        }
      }
      py::eval("object.__setattr__")(self, name, value);
    });
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
        .def(ParamInit<Class>())
        .def("set_rotation_rpy_deg", &Class::set_rotation_rpy_deg,
            py::arg("rpy_deg"), cls_doc.set_rotation_rpy_deg.doc)
        .def("IsDeterministic", &Class::IsDeterministic,
            cls_doc.IsDeterministic.doc)
        .def("GetDeterministicValue", &Class::GetDeterministicValue,
            cls_doc.GetDeterministicValue.doc)
        .def("ToSymbolic", &Class::ToSymbolic, cls_doc.ToSymbolic.doc)
        .def("Mean", &Class::Mean, cls_doc.Mean.doc)
        .def(
            "Sample", &Class::Sample, py::arg("generator"), cls_doc.Sample.doc);
    DefAttributesUsingSerialize(&cls);
    // The Transform::Serialize does something sketchy for the "rotation" field.
    // We'll undo that damage for the attribute getter and setter functions, but
    // notably we must leave the __fields__ manifest unchanged to match the C++
    // serialization convention and the setter needs to accept either a Rotation
    // (the actual type of the property) or any of the allowed Rotation::Variant
    // types (which will occur during YAML deserialization).
    using RotationOrNestedValue = std::variant<Rotation, Rotation::Identity,
        Rotation::Rpy, Rotation::AngleAxis, Rotation::Uniform>;
    static_assert(
        std::variant_size_v<RotationOrNestedValue> ==
        1 /* for Rotation */ + std::variant_size_v<Rotation::Variant>);
    cls.def_property(
        "rotation",
        // The getter is just the usual, no special magic.
        [](const Class& self) { return &self.rotation; },
        // The setter accepts a more generous allowed set of argument types.
        [](Class& self, RotationOrNestedValue value_variant) {
          std::visit(
              [&self]<typename T>(const T& new_value) {
                if constexpr (std::is_same_v<T, Rotation>) {
                  self.rotation = new_value;
                } else {
                  self.rotation.value = new_value;
                }
              },
              value_variant);
        },
        py_rvp::reference_internal, cls_doc.rotation.doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
