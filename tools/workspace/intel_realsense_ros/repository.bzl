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
        commit = "2.2.17",
        sha256 = "babffbd3b2d20c94431436fc50606fddf8712dd7e9392d9da3fd228b432f9cfe",  # noqa
        build_file = "@drake//tools/workspace/intel_realsense_ros:package.BUILD.bazel",  # noqa
        patch_cmds = [
            "cp LICENSE realsense2_description/",
            "sed -i.orig -e 's|$(find realsense2_description)/urdf/||' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_nominal_extrinsics)|false|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg add_plug)|false|' realsense2_description/urdf/*.xacro",  # noqa
        ],
        mirrors = mirrors,
    )
