# -*- python -*-

def ignition_math_cc_library(
        public_headers=None,
        private_headers=None,
        all_headers=None,
        **kwargs):
    """Declares a cc_library to check and build ignition_math library.

    """
    unknown_headers = [x for x in all_headers if x not in public_headers+private_headers ]
    if len(unknown_headers) != 0:
        fail("Update ignition_math.BUILD. Unknown headers in ignition_math: {}".format(unknown_headers))
    missing_headers = [x for x in public_headers+private_headers if x not in all_headers ]
    if len(missing_headers) != 0:
        fail("Update ignition_math.BUILD: Extra headers listed in ignition_math: {}".format(missing_headers))

    # Generates the library exported to users.  The explicitly listed srcs= matches
    # upstream's explicitly listed sources plus private headers.  The explicitly
    # listed hdrs= matches upstream's public headers.
    native.cc_library(
        name = "ignition_math",
        srcs = [
            "include/ignition/math.hh",
            "include/ignition/math/BoxPrivate.hh",
            "include/ignition/math/FrustumPrivate.hh",
            "include/ignition/math/KmeansPrivate.hh",
            "include/ignition/math/RotationSplinePrivate.hh",
            "include/ignition/math/SignalStatsPrivate.hh",
            "include/ignition/math/SplinePrivate.hh",
            "include/ignition/math/Vector3StatsPrivate.hh",
            "include/ignition/math/config.hh",  # from cmake_configure_file above
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
            "src/SplinePrivate.cc",
            "src/Temperature.cc",
            "src/Vector3Stats.cc",
        ],
        hdrs = public_headers,
        includes = ["include"],
        visibility = ["//visibility:public"],
    )
