# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Lets other packages inspect the CMake code, e.g., for the version number.
filegroup(
    name = "cmakelists_with_version",
    srcs = ["CMakeLists.txt"],
    visibility = ["//visibility:public"],
)

# Generates config.h based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "cmake/config.hh.in",
    out = "include/ignition/math/config.hh",
    cmakelists = [
        ":cmakelists_with_version",
    ],
    defines = [
        # It would be nice to get this information directly from CMakeLists.txt,
        # but it ends up being too hard.  ignition-math sets a project name
        # as "ignition-math<version>", and then uses CMake substring to pick
        # that version out.  We'd have to extend the cmake_configure_file
        # functionality to do the same, and I'm not sure it is worth it.  We
        # just hard code the major version here.
        "PROJECT_NAME_NO_VERSION=ignition-math",
        "PROJECT_MAJOR_VERSION=3",
        "PROJECT_VERSION_FULL=3.0.0",
    ],
    visibility = [],
)

public_headers = [
    "include/ignition/math/Angle.hh",
    "include/ignition/math/Box.hh",
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

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources.  The explicitly listed hdrs= matches
# upstream's explicitly listed headers.
cc_library(
    name = "lib_without_mathhh",
    srcs = [
        "src/Angle.cc",
        "src/Box.cc",
        "src/Frustum.cc",
        "src/Helpers.cc",
        "src/Kmeans.cc",
        "src/PID.cc",
        "src/Rand.cc",
        "src/RotationSpline.cc",
        "src/RotationSplinePrivate.cc",
        "src/SemanticVersion.cc",
        "src/SignalStats.cc",
        "src/SphericalCoordinates.cc",
        "src/Spline.cc",
        "src/Temperature.cc",
        "src/Vector3Stats.cc",
    ],
    # We need to list the private headers along with the public ones so
    # that bazel copies them all into the right place during the build
    # phase.
    hdrs = public_headers + [
        "include/ignition/math/BoxPrivate.hh",
        "include/ignition/math/FrustumPrivate.hh",
        "include/ignition/math/KmeansPrivate.hh",
        "include/ignition/math/RotationSplinePrivate.hh",
        "include/ignition/math/SignalStatsPrivate.hh",
        "include/ignition/math/SplinePrivate.hh",
        "include/ignition/math/Vector3StatsPrivate.hh",
        "include/ignition/math/config.hh", # from cmake_configure_file above
    ],
    includes = ["include"],
    visibility = [],
)

# Generates math.hh, which consists of #include statements for all of the
# other headers in the library except the *Private.hh ones.  The first line is
# '#include <ignition/math/config.hh>' followed by one line like
# '#include <ignition/math/Angle.hh>' for each non-generated header.
genrule(
    name = "mathhh_genrule",
    srcs = public_headers,
    outs = ["include/ignition/math.hh"],
    cmd = "(" + (
        "echo '#include <ignition/math/config.hh>' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\<\\1\\>|g'"
    ) + ") > '$@'",
    visibility = [],
)

# Generates the library exported to users.
cc_library(
    name = "lib",
    hdrs = ["include/ignition/math.hh"],  # From :mathhh_genrule above.
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [":lib_without_mathhh"],
)
