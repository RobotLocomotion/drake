# -*- python -*-

def ignition_math_cc_library(
        public_headers=None,
        private_headers=None,
        **kwargs):
    """Declares a cc_library to check and build ignition_math library.

    Before building ignition_math library, verifies that all available header
    files are listed either in public_headers or in private_headers. If some
    headers are found and not listed, or if there are extra header listed,
    this macro fails.
    """
    listed_headers = public_headers + private_headers
    all_headers = native.glob([
        "include/**/*.hh",
    ])
    unknown_headers = [x for x in all_headers if x not in listed_headers ]
    if len(unknown_headers) != 0:
        fail(
            "Update ignition_math.BUILD. Unknown headers in ignition_math: {}"
            .format(unknown_headers)
            )
    missing_headers = [x for x in listed_headers if x not in all_headers ]
    if len(missing_headers) != 0:
        fail(
            "Update ignition_math.BUILD: Extra headers listed in "
            + "ignition_math: {}".format(missing_headers)
            )

    # Generates the library exported to users.  The explicitly listed srcs= matches
    # upstream's explicitly listed sources plus private headers.  The explicitly
    # listed hdrs= matches upstream's public headers.
    native.cc_library(
        name = "ignition_math",
        srcs = [
            "include/ignition/math.hh",
            "include/ignition/math/config.hh",  # created in ignition_math.BUILD
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
        ] + private_headers,
        hdrs = public_headers,
        includes = ["include"],
        visibility = ["//visibility:public"],
    )
