# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def intel_realsense_ros_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "IntelRealSense/realsense-ros",
        # N.B. Even though 2.3.x series is not the highest-numbered release, we
        # are using it here because it aligns with the ROS Noetic version
        # released for Ubuntu 20.04.  See:
        # https://index.ros.org/p/realsense2_camera/github-IntelRealSense-realsense-ros/#noetic
        # https://github.com/IntelRealSense/realsense-ros/releases
        commit = "2.3.2",
        sha256 = "18c0f90eeea2b64889388e2e441221931e220d1fea06fe6eff8d70442c456459",  # noqa
        build_file = "@drake//tools/workspace/intel_realsense_ros_internal:package.BUILD.bazel",  # noqa
        patch_cmds = [
            "cp LICENSE realsense2_description/",
            "sed -i.orig -e 's|$(find realsense2_description)/urdf/||' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_mesh)|true|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_nominal_extrinsics)|false|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg add_plug)|false|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|package://realsense2_description|package://drake/manipulation/models/realsense2_description|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|\\.stl|.obj|' realsense2_description/urdf/*.xacro",
        ],
        mirrors = mirrors,
    )
