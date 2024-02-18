load("//tools/workspace:github.bzl", "github_archive")

def intel_realsense_ros_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "IntelRealSense/realsense-ros",
        commit = "4.54.1",
        sha256 = "98f6d760d698c0b6752679665fc8a59f29a2e75e593049f207daaf49a0f44793",  # noqa
        build_file = ":package.BUILD.bazel",
        patch_cmds = [
            "cp LICENSE realsense2_description/",
            "sed -i.orig -e 's|$(find realsense2_description)/urdf/||' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i.orig -e 's|file://$(find realsense2_description)/meshes/|../meshes/|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_mesh)|true|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg use_nominal_extrinsics)|false|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|$(arg add_plug)|false|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|package://realsense2_description|package://drake/manipulation/models/realsense2_description|' realsense2_description/urdf/*.xacro",  # noqa
            "sed -i -e 's|\\.stl|.obj|' realsense2_description/urdf/*.xacro",
        ],
        mirrors = mirrors,
    )
