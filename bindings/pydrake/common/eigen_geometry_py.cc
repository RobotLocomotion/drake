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

using std::abs;

// TODO(eric.cousineau): Remove checks (see #8960).

// N.B. This could potentially interfere with another library's bindings of
// Eigen types. If/when this happens, this should be addressed for both double
// and AutoDiff types, most likely using `py::module_local()`.

// N.B. Use a loose tolerance, so that we don't have to be super strict with
// C++.
const double kCheckTolerance = 1e-5;
constexpr char kCastDoc[] = "Cast to desired scalar type.";

using symbolic::Expression;

template <typename T>
void CheckRotMat(const Matrix3<T>& R) {
  // See `ExpectRotMat`.
  const T identity_error =
      (R * R.transpose() - Matrix3<T>::Identity()).array().abs().maxCoeff();
  if (identity_error >= kCheckTolerance) {
    throw std::logic_error("Rotation matrix is not orthonormal");
  }
  const T det_error = abs(R.determinant() - 1);
  if (det_error >= kCheckTolerance) {
    throw std::logic_error("Rotation matrix violates right-hand rule");
  }
}

template <typename T>
void CheckSe3(const Isometry3<T>& X) {
  CheckRotMat<T>(X.linear());
  Eigen::Matrix<T, 1, 4> bottom_expected;
  bottom_expected << 0, 0, 0, 1;
  const T bottom_error =
      (X.matrix().bottomRows(1) - bottom_expected).array().abs().maxCoeff();
  if (bottom_error >= kCheckTolerance) {
    throw std::logic_error("Homogeneous matrix is improperly scaled.");
  }
}

template <typename T>
void CheckQuaternion(const Eigen::Quaternion<T>& q) {
  const T norm_error = abs(q.coeffs().norm() - 1);
  if (norm_error >= kCheckTolerance) {
    throw std::logic_error("Quaternion is not normalized");
  }
}

template <typename T>
void CheckAngleAxis(const Eigen::AngleAxis<T>& value) {
  const T norm_error = abs(value.axis().norm() - 1);
  if (norm_error >= kCheckTolerance) {
    throw std::logic_error("Axis is not normalized");
  }
}

// N.B. The following overloads are meant to disable symbolic checks, which are
// not easily achievable for non-numeric values. These can be removed once the
// checks are removed in their entirety (#8960).

void CheckRotMat(const Matrix3<Expression>&) {}

void CheckSe3(const Isometry3<Expression>&) {}

void CheckQuaternion(const Eigen::Quaternion<Expression>&) {}

void CheckAngleAxis(const Eigen::AngleAxis<Expression>&) {}

}  // namespace

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  // Do not return references to matrices (e.g. `Eigen::Ref<>`) so that we have
  // tighter control over validation.

  py::tuple param = GetPyParam<T>();

  // Isometry3d.
  // @note `linear` implies rotation, and `affine` implies translation.
  {
    using Class = Isometry3<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(m, "Isometry3", param,
        "Provides bindings of Eigen::Isometry3<> that only admit SE(3) "
        "(no reflections).");
    cls  // BR
        .def(py::init([]() { return Class::Identity(); }))
        .def_static("Identity", []() { return Class::Identity(); })
        .def(py::init([](const Matrix4<T>& matrix) {
          Class out(matrix);
          CheckSe3(out);
          return out;
        }),
            py::arg("matrix"))
        .def(py::init(
                 [](const Matrix3<T>& rotation, const Vector3<T>& translation) {
                   CheckRotMat(rotation);
                   Class out = Class::Identity();
                   out.linear() = rotation;
                   out.translation() = translation;
                   return out;
                 }),
            py::arg("rotation"), py::arg("translation"))
        .def(py::init([](const Eigen::Quaternion<T>& q,
                          const Vector3<T>& translation) {
          CheckQuaternion(q);
          Class out = Class::Identity();
          out.linear() = q.toRotationMatrix();
          out.translation() = translation;
          return out;
        }),
            py::arg("quaternion"), py::arg("translation"))
        .def(py::init([](const Class& other) {
          CheckSe3(other);
          return other;
        }),
            py::arg("other"))
        .def("matrix",
            [](const Class* self) -> Matrix4<T> { return self->matrix(); })
        .def("set_matrix",
            [](Class* self, const Matrix4<T>& matrix) {
              Class update(matrix);
              CheckSe3(update);
              *self = update;
            })
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
              CheckRotMat(rotation);
              self->linear() = rotation;
            })
        .def("quaternion",
            [](const Class* self) {
              return Eigen::Quaternion<T>(self->linear());
            })
        .def("set_quaternion",
            [](Class* self, const Eigen::Quaternion<T>& q) {
              CheckQuaternion(q);
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
  // TODO(eric.cousineau): Should this not be restricted to a unit quaternion?
  {
    using Class = Eigen::Quaternion<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(m, "Quaternion", param,
        "Provides a unit quaternion binding of Eigen::Quaternion<>.");
    py::object py_class_obj = cls;
    cls  // BR
        .def(py::init([]() { return Class::Identity(); }))
        .def_static("Identity", []() { return Class::Identity(); })
        .def(py::init([](const Vector4<T>& wxyz) {
          Class out(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
          CheckQuaternion(out);
          return out;
        }),
            py::arg("wxyz"))
        .def(py::init([](T w, T x, T y, T z) {
          Class out(w, x, y, z);
          CheckQuaternion(out);
          return out;
        }),
            py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def(py::init([](const Matrix3<T>& rotation) {
          Class out(rotation);
          CheckQuaternion(out);
          return out;
        }),
            py::arg("rotation"))
        .def(py::init([](const Class& other) {
          CheckQuaternion(other);
          return other;
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
              Class update;
              update.w() = wxyz(0);
              update.vec() = wxyz.tail(3);
              CheckQuaternion(update);
              *self = update;
            },
            py::arg("wxyz"))
        .def(
            "set_wxyz",
            [](Class* self, T w, T x, T y, T z) {
              Class update(w, x, y, z);
              CheckQuaternion(update);
              *self = update;
            },
            py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def("rotation",
            [](const Class* self) { return self->toRotationMatrix(); })
        .def("set_rotation",
            [](Class* self, const Matrix3<T>& rotation) {
              Class update(rotation);
              CheckQuaternion(update);
              *self = update;
            })
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
        m, "AngleAxis", param, "Bindings for Eigen::AngleAxis<>.");
    py::object py_class_obj = cls;
    cls  // BR
        .def(py::init([]() { return Class::Identity(); }))
        .def_static("Identity", []() { return Class::Identity(); })
        .def(py::init([](const T& angle, const Vector3<T>& axis) {
          Class out(angle, axis);
          CheckAngleAxis(out);
          return out;
        }),
            py::arg("angle"), py::arg("axis"))
        .def(py::init([](const Eigen::Quaternion<T>& q) {
          Class out(q);
          CheckAngleAxis(out);
          return out;
        }),
            py::arg("quaternion"))
        .def(py::init([](const Matrix3<T>& rotation) {
          Class out(rotation);
          CheckAngleAxis(out);
          return out;
        }),
            py::arg("rotation"))
        .def(py::init([](const Class& other) {
          CheckAngleAxis(other);
          return other;
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
              Class update(self->angle(), axis);
              CheckAngleAxis(update);
              *self = update;
            },
            py::arg("axis"))
        .def("rotation",
            [](const Class* self) { return self->toRotationMatrix(); })
        .def(
            "set_rotation",
            [](Class* self, const Matrix3<T>& rotation) {
              Class update(rotation);
              CheckAngleAxis(update);
              *self = update;
            },
            py::arg("rotation"))
        .def("quaternion",
            [](const Class* self) { return Eigen::Quaternion<T>(*self); })
        .def(
            "set_quaternion",
            [](Class* self, const Eigen::Quaternion<T>& q) {
              CheckQuaternion(q);
              Class update(q);
              CheckAngleAxis(update);
              *self = update;
            },
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
