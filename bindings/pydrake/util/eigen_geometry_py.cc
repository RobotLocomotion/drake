#include <cmath>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/common/drake_assertion_error.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

using std::fabs;

namespace drake {
namespace pydrake {
namespace {

// TODO(eric.cousineau): There is validation from Python to C++, but no
// validation in the other direction. Consider intercepting this.

// TODO(eric.cousineau): Add operator overloads.

// TODO(eric.cousineau): Disable tolerance checks for the symbolic case.

// N.B. This could potentially interfere with another library's bindings of
// Eigen types. If/when this happens, this should be addressed for both these
// and AutoDiff types.

// N.B. Use a loose tolerance, so that we don't have to be super strict with
// C++.
const double kCheckTolerance = 1e-5;

template <typename T>
void CheckRotMat(const Matrix3<T>& R) {
  // See `ExpectRotMat`.
  const T identity_error =
      (R * R.transpose() - Eigen::Matrix<T, 3, 3>::Identity())
      .array().abs().maxCoeff();
  DRAKE_THROW_UNLESS(
      identity_error < kCheckTolerance &&
      "Rotation matrix is not orthonormal");
  const T det_error = fabs(R.determinant() - 1);
  DRAKE_THROW_UNLESS(
    det_error < kCheckTolerance &&
    "Rotation matrix violates right-hand rule");
}

template <typename T>
void CheckIsometry(const Isometry3<T>& X) {
  CheckRotMat<T>(X.linear());
  Eigen::Matrix<T, 1, 4> bottom_expected;
  bottom_expected << 0, 0, 0, 1;
  const T bottom_error =
      (X.matrix().bottomRows(1) - bottom_expected).array().abs().maxCoeff();
  DRAKE_THROW_UNLESS(
      bottom_error < kCheckTolerance &&
      "Homogeneous matrix is improperly scaled.");
}

template <typename T>
void CheckQuaternion(const Eigen::Quaternion<T>& q) {
  const T norm_error = fabs(q.coeffs().norm() - 1);
  DRAKE_THROW_UNLESS(
      norm_error < kCheckTolerance &&
      "Quaternion is not normalized");
}

template <typename T>
void CheckAngleAxis(const Eigen::AngleAxis<T>& value) {
  const T norm_error = fabs(value.axis().norm() - 1);
  DRAKE_THROW_UNLESS(
      norm_error < kCheckTolerance &&
      "Axis is not normalized");
}

}  // namespace

PYBIND11_MODULE(eigen_geometry, m) {
  m.doc() = "Bindings for Eigen geometric types.";

  using T = double;

  // Do not return references to matrices (e.g. `Eigen::Ref<>`) so that we have
  // tighter control over validation.

  // Isometry3d.
  // @note `linear` implies rotation, and `affine` implies translation.
  {
    using Class = Isometry3<T>;
    py::class_<Class> py_class(m, "Isometry3");
    py_class
      .def(py::init([]() {
        return Class::Identity();
      }))
      .def_static("Identity", []() {
        return Class::Identity();
      })
      .def(py::init([](const Matrix4<T>& matrix) {
        Class out(matrix);
        CheckIsometry(out);
        return out;
      }), py::arg("matrix"))
      .def(py::init([](
          const Matrix3<T>& rotation,
          const Vector3<T>& translation) {
        CheckRotMat(rotation);
        Class out = Class::Identity();
        out.linear() = rotation;
        out.translation() = translation;
        return out;
      }), py::arg("rotation"), py::arg("translation"))
      .def(py::init([](
          const Eigen::Quaternion<T>& q,
          const Vector3<T>& translation) {
        CheckQuaternion(q);
        Class out = Class::Identity();
        out.linear() = q.toRotationMatrix();
        out.translation() = translation;
        return out;
      }), py::arg("quaternion"), py::arg("translation"))
      .def(py::init([](const Class& other) {
        CheckIsometry(other);
        return other;
      }), py::arg("other"))
      .def("matrix", [](const Class* self) -> Matrix4<T> {
        return self->matrix();
      })
      .def("set_matrix", [](Class* self, const Matrix4<T>& matrix) {
        Class update(matrix);
        CheckIsometry(update);
        *self = update;
      })
      .def("translation", [](const Class* self) -> Vector3<T> {
        return self->translation();
      })
      .def("set_translation", [](Class* self, const Vector3<T>& translation) {
        self->translation() = translation;
      })
      .def("rotation", [](const Class* self) -> Matrix3<T> {
        return self->linear();
      })
      .def("set_rotation", [](Class* self, const Matrix3<T>& rotation) {
        CheckRotMat(rotation);
        self->linear() = rotation;
      })
      .def("quaternion", [](const Class* self) {
        return Eigen::Quaternion<T>(self->linear());
      })
      .def("set_quaternion", [](Class* self, const Eigen::Quaternion<T>& q) {
        CheckQuaternion(q);
        self->linear() = q.toRotationMatrix();
      })
      .def("__str__", [](py::object self) {
        return py::str(self.attr("matrix")());
      })
      // Do not define operator `__mul__` until we have the Python3 `@`
      // operator so that operations are similar to those of arrays.
      .def("multiply", [](const Class& self, const Class& other) {
        return self * other;
      })
      .def("inverse", [](const Class* self) {
        return self->inverse();
      });
    py::implicitly_convertible<Matrix4<T>, Class>();
  }

  // Quaternion.
  // Since the Eigen API for Quaternion is insufficiently explicit, we will
  // deviate some from the API to maintain clarity.
  // TODO(eric.cousineau): Should this not be restricted to a unit quaternion?
  {
    using Class = Eigen::Quaternion<T>;
    py::class_<Class> py_class(m, "Quaternion");
    py_class.attr("__doc__") =
        "Provides a unit quaternion binding of Eigen::Quaternion<>.";
    py::object py_class_obj = py_class;
    py_class
      .def(py::init([]() {
        return Class::Identity();
      }))
      .def_static("Identity", []() {
        return Class::Identity();
      })
      .def(py::init([](const Vector4<T>& wxyz) {
        Class out(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
        CheckQuaternion(out);
        return out;
      }), py::arg("wxyz"))
      .def(py::init([](T w, T x, T y, T z) {
        Class out(w, x, y, z);
        CheckQuaternion(out);
        return out;
      }), py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
      .def(py::init([](const Matrix3<T>& rotation) {
        Class out(rotation);
        CheckQuaternion(out);
        return out;
      }), py::arg("rotation"))
      .def(py::init([](const Class& other) {
        CheckQuaternion(other);
        return other;
      }), py::arg("other"))
      .def("w", [](const Class* self) { return self->w(); })
      .def("x", [](const Class* self) { return self->x(); })
      .def("y", [](const Class* self) { return self->y(); })
      .def("z", [](const Class* self) { return self->z(); })
      .def("xyz", [](const Class* self) { return self->vec(); })
      .def("wxyz", [](Class* self) {
        Vector4<T> wxyz;
        wxyz << self->w(), self->vec();
        return wxyz;
      })
      .def("set_wxyz", [](Class* self, const Vector4<T>& wxyz) {
        Class update;
        update.w() = wxyz(0);
        update.vec() = wxyz.tail(3);
        CheckQuaternion(update);
        *self = update;
      }, py::arg("wxyz"))
      .def("set_wxyz", [](Class* self, T w, T x, T y, T z) {
        Class update(w, x, y, z);
        CheckQuaternion(update);
        *self = update;
      }, py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
      .def("rotation", [](const Class* self) {
        return self->toRotationMatrix();
      })
      .def("set_rotation", [](Class* self, const Matrix3<T>& rotation) {
        Class update(rotation);
        CheckQuaternion(update);
        *self = update;
      })
      .def("__str__", [py_class_obj](const Class* self) {
        return py::str("{}(w={}, x={}, y={}, z={})").format(
            py_class_obj.attr("__name__"),
            self->w(), self->x(), self->y(), self->z());
      })
      // Do not define operator `__mul__` until we have the Python3 `@`
      // operator so that operations are similar to those of arrays.
      .def("multiply", [](const Class& self, const Class& other) {
        return self * other;
      })
      .def("inverse", [](const Class* self) {
        return self->inverse();
      });
  }

  // Angle-axis.
  {
    using Class = Eigen::AngleAxis<T>;
    py::class_<Class> py_class(m, "AngleAxis");
    py_class.attr("__doc__") =
        "Bindings for Eigen::AngleAxis<>.";
    py::object py_class_obj = py_class;
    py_class
      .def(py::init([]() {
        return Class::Identity();
      }))
      .def_static("Identity", []() {
        return Class::Identity();
      })
      .def(py::init([](const T& angle, const Vector3<T>& axis) {
        Class out(angle, axis);
        CheckAngleAxis(out);
        return out;
      }), py::arg("angle"), py::arg("axis"))
      .def(py::init([](const Eigen::Quaternion<T>& q) {
        Class out(q);
        CheckAngleAxis(out);
        return out;
      }), py::arg("quaternion"))
      .def(py::init([](const Matrix3<T>& rotation) {
        Class out(rotation);
        CheckAngleAxis(out);
        return out;
      }), py::arg("rotation"))
      .def(py::init([](const Class& other) {
        CheckAngleAxis(other);
        return other;
      }), py::arg("other"))
      .def("angle", [](const Class* self) { return self->angle(); })
      .def("axis", [](const Class* self) { return self->axis(); })
      .def("set_angle", [](Class* self, const T& angle) {
        // N.B. Since `axis` should already be valid, do not need to check.
        self->angle() = angle;
      }, py::arg("angle"))
      .def("set_axis", [](Class* self, const Vector3<T>& axis) {
        Class update(self->angle(), axis);
        CheckAngleAxis(update);
        *self = update;
      }, py::arg("axis"))
      .def("rotation", [](const Class* self) {
        return self->toRotationMatrix();
      })
      .def("set_rotation", [](Class* self, const Matrix3<T>& rotation) {
        Class update(rotation);
        CheckAngleAxis(update);
        *self = update;
      })
      .def("quaternion", [](const Class* self) {
        return Eigen::Quaternion<T>(*self);
      })
      .def("set_quaternion", [](Class* self, const Eigen::Quaternion<T>& q) {
        CheckQuaternion(q);
        Class update(q);
        CheckAngleAxis(update);
        *self = update;
      })
      .def("__str__", [py_class_obj](const Class* self) {
        return py::str("{}(angle={}, axis={})").format(
            py_class_obj.attr("__name__"),
            self->angle(), self->axis());
      })
      // Do not define operator `__mul__` until we have the Python3 `@`
      // operator so that operations are similar to those of arrays.
      .def("multiply", [](const Class& self, const Class& other) {
        return self * other;
      })
      .def("inverse", [](const Class* self) {
        return self->inverse();
      });
  }
}

}  // namespace pydrake
}  // namespace drake
