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
        commit = "2.3.0",
        sha256 = "0469bde721ed9db0f4d362ab2838237e8a8ccb40d35188504148b2d72843a474",  # noqa
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
