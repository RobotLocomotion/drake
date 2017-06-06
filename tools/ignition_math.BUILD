# -*- python -*-

load("@drake//tools:cmake_configure_file.bzl", "cmake_configure_file")
load("@drake//tools:install.bzl", "cmake_config", "install", "install_cmake_config")

package(default_visibility = ["//visibility:public"])

# Generates config.hh based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/config.hh.in",
    out = "include/ignition/math/config.hh",
    cmakelists = ["CMakeLists.txt"],
    defines = [
        # It would be nice to get this information directly from CMakeLists.txt,
        # but it ends up being too hard.  ignition-math sets a project name
        # as "ignition-math<version>", and then uses CMake substring to pick
        # that version out.  We'd have to extend the cmake_configure_file
        # functionality to do the same, and I'm not sure it is worth it.  We
        # just hard code the major version here.
        "PROJECT_NAME_NO_VERSION=ignition-math",
        "PROJECT_MAJOR_VERSION=3",
        "PROJECT_VERSION_FULL=3.2.0",
    ],
    visibility = ["//visibility:private"],
)

public_headers = [
    "include/ignition/math/Angle.hh",
    "include/ignition/math/Box.hh",
    "include/ignition/math/Color.hh",
    "include/ignition/math/Filter.hh",
    "include/ignition/math/Frustum.hh",
    "include/ignition/math/Helpers.hh",
    "include/ignition/math/Inertial.hh",
    "include/ignition/math/Kmeans.hh",
    "include/ignition/math/Line2.hh",
    "include/ignition/math/Line3.hh",
    "include/ignition/math/MassMatrix3.hh",
    "include/ignition/math/Matrix3.hh",
    "include/ignition/math/Matrix4.hh",
    "include/ignition/math/OrientedBox.hh",
    "include/ignition/math/PID.hh",
    "include/ignition/math/Plane.hh",
    "include/ignition/math/Pose3.hh",
    "include/ignition/math/Quaternion.hh",
    "include/ignition/math/Rand.hh",
    "include/ignition/math/RotationSpline.hh",
    "include/ignition/math/SemanticVersion.hh",
    "include/ignition/math/SignalStats.hh",
    "include/ignition/math/SphericalCoordinates.hh",
    "include/ignition/math/Spline.hh",
    "include/ignition/math/System.hh",
    "include/ignition/math/Temperature.hh",
    "include/ignition/math/Triangle.hh",
    "include/ignition/math/Triangle3.hh",
    "include/ignition/math/Vector2.hh",
    "include/ignition/math/Vector3.hh",
    "include/ignition/math/Vector3Stats.hh",
    "include/ignition/math/Vector4.hh",
]

private_headers = [
    "include/ignition/math/BoxPrivate.hh",
    "include/ignition/math/FrustumPrivate.hh",
    "include/ignition/math/KmeansPrivate.hh",
    "include/ignition/math/RotationSplinePrivate.hh",
    "include/ignition/math/SignalStatsPrivate.hh",
    "include/ignition/math/SplinePrivate.hh",
    "include/ignition/math/Vector3StatsPrivate.hh",
]

# Generates math.hh, which consists of #include statements for all of the
# public headers in the library.  The first line is
# '#include <ignition/math/config.hh>' followed by one line like
# '#include <ignition/math/Angle.hh>' for each non-generated header.
genrule(
    name = "mathhh_genrule",
    srcs = public_headers,
    outs = ["include/ignition/math.hh"],
    # TODO: centralize this logic, as it is used here, in sdformat.BUILD, and
    # in fcl.BUILD
    cmd = "(" + (
        "echo '#include <ignition/math/config.hh>' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
    visibility = ["//visibility:private"],
)

ignition_math_cc_library(
    public_headers = public_headers,
    private_headers = private_headers,
)

CMAKE_PACKAGE = "ignition-math3"

cmake_config(
    package = CMAKE_PACKAGE,
    script = "@drake//tools:ignition_math-create-cps.py",
    version_file = ":config",
)

install_cmake_config(package = CMAKE_PACKAGE)  # Creates rule :install_cmake_config.

install(
    name = "install",
    hdrs = public_headers + [
        ":config",
        ":mathhh_genrule",
    ],
    doc_dest = "share/doc/" + CMAKE_PACKAGE,
    hdr_dest = "include",
    hdr_strip_prefix = ["include"],
    license_docs = ["LICENSE", "COPYING"],
    targets = [":ignition_math"],
    deps = [":install_cmake_config"],
)
