#include <cmath>
#include <stdexcept>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_assertion_error.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace pydrake {
namespace {

// N.B. This could potentially interfere with another library's bindings of
// Eigen types. If/when this happens, this should be addressed for both double
// and AutoDiff types, most likely using `py::module_local()`.

constexpr char kCastDoc[] = "Cast to desired scalar type.";

}  // namespace

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  // At present, does not return references to matrices (e.g. `Eigen::Ref<>`).

  py::tuple param = GetPyParam<T>();

  // Isometry3d.
  // @note `linear` implies rotation, and `affine` implies translation.
  {
    using Class = Isometry3<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "Isometry3", param, "Provides bindings of Eigen::Isometry3<>.");
    cls  // BR
        .def(py::init([]() { return Class::Identity(); }))
        .def_static("Identity", []() { return Class::Identity(); })
        .def(py::init([](const Matrix4<T>& matrix) { return Class(matrix); }),
            py::arg("matrix"))
        .def(py::init(
                 [](const Matrix3<T>& rotation, const Vector3<T>& translation) {
                   Class out = Class::Identity();
                   out.linear() = rotation;
                   out.translation() = translation;
                   return out;
                 }),
            py::arg("rotation"), py::arg("translation"))
        .def(py::init([](const Eigen::Quaternion<T>& q,
                          const Vector3<T>& translation) {
          Class out = Class::Identity();
          out.linear() = q.toRotationMatrix();
          out.translation() = translation;
          return out;
        }),
            py::arg("quaternion"), py::arg("translation"))
        .def(py::init([](const Class& other) {
          return other;  // pybind11 will handle copying the object.
        }),
            py::arg("other"))
        .def("matrix",
            [](const Class* self) -> Matrix4<T> { return self->matrix(); })
        .def("set_matrix",
            [](Class* self, const Matrix4<T>& matrix) { *self = matrix; })
        .def("translation",
            [](const Class* self) -> Vector3<T> { return self->translation(); })
        .def("set_translation",
            [](Class* self, const Vector3<T>& translation) {
              self->translation() = translation;
            })
        .def("rotation",
            [](const Class* self) -> Matrix3<T> { return self->linear(); })
        .def("set_rotation",
            [](Class* self, const Matrix3<T>& rotation) {
              self->linear() = rotation;
            })
        .def("quaternion",
            [](const Class* self) {
              return Eigen::Quaternion<T>(self->linear());
            })
        .def("set_quaternion",
            [](Class* self, const Eigen::Quaternion<T>& q) {
              self->linear() = q.toRotationMatrix();
            })
        .def("__str__",
            [](py::object self) { return py::str(self.attr("matrix")()); })
        .def(
            "multiply",
            [](const Class& self, const Class& other) { return self * other; },
            py::arg("other"), "RigidTransform multiplication")
        .def(
            "multiply",
            [](const Class& self, const Vector3<T>& position) {
              return self * position;
            },
            py::arg("position"), "Position vector multiplication")
        .def(
            "multiply",
            [](const Class& self, const Matrix3X<T>& position) {
              return self * position;
            },
            py::arg("position"), "Position vector list multiplication")
        .def("inverse", [](const Class* self) { return self->inverse(); })
        .def(py::pickle([](const Class& self) { return self.matrix(); },
            [](const Matrix4<T>& matrix) { return Class(matrix); }));
    cls.attr("multiply") = WrapToMatchInputShape(cls.attr("multiply"));
    cls.attr("__matmul__") = cls.attr("multiply");
    py::implicitly_convertible<Matrix4<T>, Class>();
    DefCopyAndDeepCopy(&cls);
    DefCast<T>(&cls, kCastDoc);
    // TODO(eric): Consider deprecating / removing `Value[Isometry3]` pending
    // resolution of #9865.
    AddValueInstantiation<Isometry3<T>>(m);
  }

  // Quaternion.
  // Since the Eigen API for Quaternion is insufficiently explicit, we will
  // deviate some from the API to maintain clarity.
  {
    using Class = Eigen::Quaternion<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "Quaternion", param, "Provides bindings of Eigen::Quaternion<>.");
    py::object py_class_obj = cls;
    cls  // BR
        .def(py::init([]() { return Class::Identity(); }))
        .def_static("Identity", []() { return Class::Identity(); })
        .def(py::init([](const Vector4<T>& wxyz) {
          return Class(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
        }),
            py::arg("wxyz"))
        .def(py::init([](T w, T x, T y, T z) { return Class(w, x, y, z); }),
            py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def(py::init(
                 [](const Matrix3<T>& rotation) { return Class(rotation); }),
            py::arg("rotation"))
        .def(py::init([](const Class& other) {
          return other;  // N.B. pybind11 will create the copy.
        }),
            py::arg("other"))
        .def("w", [](const Class* self) { return self->w(); })
        .def("x", [](const Class* self) { return self->x(); })
        .def("y", [](const Class* self) { return self->y(); })
        .def("z", [](const Class* self) { return self->z(); })
        .def("xyz", [](const Class* self) { return Vector3<T>(self->vec()); })
        .def("wxyz",
            [](Class* self) {
              Vector4<T> wxyz;
              wxyz << self->w(), self->vec();
              return wxyz;
            })
        .def(
            "set_wxyz",
            [](Class* self, const Vector4<T>& wxyz) {
              self->w() = wxyz(0);
              self->vec() = wxyz.tail(3);
            },
            py::arg("wxyz"))
        .def(
            "set_wxyz",
            [](Class* self, T w, T x, T y, T z) { *self = Class(w, x, y, z); },
            py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def("rotation",
            [](const Class* self) { return self->toRotationMatrix(); })
        .def("set_rotation",
            [](Class* self, const Matrix3<T>& rotation) { *self = rotation; })
        .def("__str__",
            [py_class_obj](const Class* self) {
              return py::str("{}(w={}, x={}, y={}, z={})")
                  .format(py_class_obj.attr("__name__"), self->w(), self->x(),
                      self->y(), self->z());
            })
        .def(
            "multiply",
            [](const Class& self, const Class& other) { return self * other; },
            "Quaternion multiplication")
        .def(
            "slerp",
            [](const Class& self, double t, const Class& other) {
              return self.slerp(t, other);
            },
            py::arg("t"), py::arg("other"),
            "The spherical linear interpolation between the two quaternions "
            "(self and other) at the parameter t in [0;1].");
    auto multiply_vector = [](const Class& self, const Vector3<T>& vector) {
      return self * vector;
    };
    auto multiply_vector_list = [](const Class& self,
                                    const Matrix3X<T>& vector) {
      Matrix3X<T> out(vector.rows(), vector.cols());
      for (int i = 0; i < vector.cols(); ++i) {
        out.col(i) = self * vector.col(i);
      }
      return out;
    };
    cls  // BR
        .def("multiply", multiply_vector, py::arg("vector"),
            "Multiplication by a vector expressed in a frame")
        .def("multiply", multiply_vector_list, py::arg("vector"),
            "Multiplication by a list of vectors expressed in the same frame")
        .def("inverse", [](const Class* self) { return self->inverse(); })
        .def("conjugate", [](const Class* self) { return self->conjugate(); })
        .def(py::pickle(
            // Leverage Python API so we can easily use `wxyz` form.
            [](py::object self) { return self.attr("wxyz")(); },
            [py_class_obj](py::object wxyz) -> Class {
              return py_class_obj(wxyz).cast<Class>();
            }));
    cls.attr("multiply") = WrapToMatchInputShape(cls.attr("multiply"));
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
    DefCast<T>(&cls, kCastDoc);
  }

  // Angle-axis.
  {
    using Class = Eigen::AngleAxis<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "AngleAxis", param, "Provides bindings of Eigen::AngleAxis<>.");
    py::object py_class_obj = cls;
    cls  // BR
        .def(py::init([]() { return Class::Identity(); }))
        .def_static("Identity", []() { return Class::Identity(); })
        .def(py::init([](const T& angle, const Vector3<T>& axis) {
          return Class(angle, axis);
        }),
            py::arg("angle"), py::arg("axis"))
        .def(py::init([](const Eigen::Quaternion<T>& q) { return Class(q); }),
            py::arg("quaternion"))
        .def(py::init(
                 [](const Matrix3<T>& rotation) { return Class(rotation); }),
            py::arg("rotation"))
        .def(py::init([](const Class& other) {
          return other;  // pybind11 will handle copying the object.
        }),
            py::arg("other"))
        .def("angle", [](const Class* self) { return self->angle(); })
        .def("axis", [](const Class* self) { return self->axis(); })
        .def(
            "set_angle",
            [](Class* self, const T& angle) {
              // N.B. Since `axis` should already be valid, do not need to
              // check.
              self->angle() = angle;
            },
            py::arg("angle"))
        .def(
            "set_axis",
            [](Class* self, const Vector3<T>& axis) {
              *self = Class(self->angle(), axis);
            },
            py::arg("axis"))
        .def("rotation",
            [](const Class* self) { return self->toRotationMatrix(); })
        .def(
            "set_rotation",
            [](Class* self, const Matrix3<T>& rotation) { *self = rotation; },
            py::arg("rotation"))
        .def("quaternion",
            [](const Class* self) { return Eigen::Quaternion<T>(*self); })
        .def(
            "set_quaternion",
            [](Class* self, const Eigen::Quaternion<T>& q) { *self = q; },
            py::arg("q"))
        .def("__str__",
            [py_class_obj](const Class* self) {
              return py::str("{}(angle={}, axis={})")
                  .format(py_class_obj.attr("__name__"), self->angle(),
                      self->axis());
            })
        .def(
            "multiply",
            [](const Class& self, const Class& other) { return self * other; },
            py::arg("other"))
        .def("inverse", [](const Class* self) { return self->inverse(); })
        .def(py::pickle(
            [](const Class& self) {
              return py::make_tuple(self.angle(), self.axis());
            },
            [](py::tuple t) {
              DRAKE_THROW_UNLESS(t.size() == 2);
              return Class(t[0].cast<T>(), t[1].cast<Vector3<T>>());
            }));
    // N.B. This class does not support multiplication with vectors, so we do
    // not use `WrapToMatchInputShape` here.
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
    DefCast<T>(&cls, kCastDoc);
  }
}

PYBIND11_MODULE(eigen_geometry, m) {
  m.doc() = "Bindings for Eigen geometric types.";

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.symbolic");
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
