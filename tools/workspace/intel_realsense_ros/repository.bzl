# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def intel_realsense_ros_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "IntelRealSense/realsense-ros",
        # N.B. Even though 2.3.x series is not the highest-numbered release, we
        # are using it here because it aligns with the ROS Melodic version
        # released for Ubuntu 18.04.
        commit = "2.3.1",
        sha256 = "c3ef2b614171d97cc1f5b73220e183679396fb6e6c9e3c29e13495c9280a3e22",  # noqa
        build_file = "@drake//tools/workspace/intel_realsense_ros:package.BUILD.bazel",  # noqa
        patch_cmds = [
            "cp LICENSE realsense2_description/",
            "sed -i.orig -e 's|$(find realsense2_description)/urdf/||' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_mesh)|true|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_nominal_extrinsics)|false|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg add_plug)|false|' realsense2_description/urdf/*.xacro",  # noqa
        ],
        mirrors = mirrors,
    )
