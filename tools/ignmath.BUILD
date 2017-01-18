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
        # It would be nice to get this information directly from CMakeLists.txt, but it
        # ends up being too hard.  ignition-math sets a project name
        # as "ignition-math<version>", and then uses CMake substring to pick that version
        # out.  We'd have to extend the cmake_configure_file functionality to do the same,
        # and I'm not sure it is worth it.  We just hard code the major version here.
        "PROJECT_NAME_NO_VERSION=ignition-math",
        "PROJECT_MAJOR_VERSION=3",
        "PROJECT_VERSION_FULL=3.0.0",
    ],
    visibility = [],
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources.  The globbed hdrs= matches upstream's
# explicitly globbed headers.
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
    hdrs = glob([
        "include/ignition/math/*.hh",
    ]) + [
        "include/ignition/math/config.hh",
    ],
    includes = ["include"],
    visibility = [],
)

# Generates math.hh, which consists of #include statements for *all* of the other
# headers in the library (!!!).  The first line is '#include <ignition/math/config.hh>'
# followed by one line like '#include <ignition/math/Angle.hh>' for each non-generated header.
genrule(
    name = "mathhh_genrule",
    srcs = glob(["include/ignition/math/*.hh"]),
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
