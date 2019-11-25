"""XXX
"""

from os.path import join
import subprocess

import install_test_helper


def main():
    install_dir = install_test_helper.get_install_dir()
    scratch_dir = install_test_helper.create_temporary_dir("scratch")

    with open(join(scratch_dir, "WORKSPACE"), "w") as f:
        DRAKE_BINARY_INSTALL_DIR = "/home/jwnimmer/jwnimmer-tri/drake/tmp"
        f.write(f"""
workspace(name = "scratch")
local_repository(
    name = "drake_bazel_workspace_loader",
    path = "{install_dir}/share/drake_bazel_workspace_loader",
)
load("@drake_bazel_workspace_loader//:repo.bzl", "drake_repository")
drake_repository(name = "drake")
""")

    with open(join(scratch_dir, "BUILD.bazel"), "w") as f:
        f.write(f"""
py_test(
    name = "find_resource_test",
    srcs = ["find_resource_test.py"],
    deps = ["@drake//bindings/pydrake"],
)
""")

    with open(join(scratch_dir, "find_resource_test.py"), "w") as f:
        f.write(f"""
from pydrake.common import FindResourceOrThrow, set_log_level
set_log_level("trace")
FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf")
""")

    # Use "release engineering" options for hermeticity.
    # https://docs.bazel.build/versions/master/user-manual.html#bazel-releng
    subprocess.check_call(
        ["bazel", "--bazelrc=/dev/null", "test", "//..."],
        cwd=scratch_dir,
    )


if __name__ == '__main__':
    main()
