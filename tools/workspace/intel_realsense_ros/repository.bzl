# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def intel_realsense_ros_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "IntelRealSense/realsense-ros",
        # N.B. Even though 2.2.x series is not the highest-numbered release, we
        # are using it here because it aligns with the ROS Melodic version
        # released for Ubuntu 18.04.
        commit = "2.2.18",
        sha256 = "bf38a295e72838be0b08d6f1629150934eb95ed1d8b2632d40f2f45950e8c1c9",  # noqa
        build_file = "@drake//tools/workspace/intel_realsense_ros:package.BUILD.bazel",  # noqa
        patch_cmds = [
            "cp LICENSE realsense2_description/",
            "sed -i.orig -e 's|$(find realsense2_description)/urdf/||' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_nominal_extrinsics)|false|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg add_plug)|false|' realsense2_description/urdf/*.xacro",  # noqa
        ],
        mirrors = mirrors,
    )
