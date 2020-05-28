"""The acceptance test for //tools/install/bazel:install.
"""

from os.path import join
import subprocess
import sys

import install_test_helper


def main():
    install_dir = install_test_helper.get_install_dir()

    # In scratch, mock up a drake_bazel_installed workspace.
    scratch_dir = install_test_helper.create_temporary_dir("scratch")

    # TODO(jamiesnape): Automatically keep this synchronized with the version
    # used by @drake (or the nearest stable version).
    rules_python_commit = "0.0.2"
    rules_python_url = f"https://github.com/bazelbuild/rules_python/archive/{rules_python_commit}.tar.gz"  # noqa
    rules_python_sha256 = "a1b2797d85abb395468915f2ab652cee65e9a44e31bff559cd7de0dedf056363"  # noqa

    with open(join(scratch_dir, "WORKSPACE"), "w") as f:
        f.write(f"""
workspace(name = "scratch")

load(
    "@bazel_tools//tools/build_defs/repo:http.bzl",
    "http_archive",
)
http_archive(
    name = "rules_python",
    sha256 = "{rules_python_sha256}",
    strip_prefix = "rules_python-{rules_python_commit}",
    url = "{rules_python_url}",
)
load(
    "@rules_python//python:repositories.bzl",
    "py_repositories",
)
py_repositories()

new_local_repository(
    name = "drake_binary",
    path = "{install_dir}",
    build_file_content = "#",
)
load(
    "@drake_binary//:share/drake/repo.bzl",
    "drake_repository",
)
drake_repository(name = "drake")
""")

    with open(join(scratch_dir, "BUILD.bazel"), "w") as f:
        f.write(f"""
load("@rules_python//python:defs.bzl", "py_test")

py_test(
    name = "find_resource_test",
    srcs = ["find_resource_test.py"],
    size = "small",
    deps = ["@drake//bindings/pydrake"],
)

py_test(
    name = "import_all_test",
    srcs = ["import_all_test.py"],
    size = "small",
    deps = ["@drake//bindings/pydrake"],
)

# A stub for unit testing; not required for end users.
filegroup(name = "dummy_filegroup")
""")

    with open(join(scratch_dir, "find_resource_test.py"), "w") as f:
        f.write(f"""
from pydrake.common import FindResourceOrThrow, set_log_level
set_log_level("trace")
FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf")
""")

    with open(join(scratch_dir, "import_all_test.py"), "w") as f:
        f.write(f"""
import pydrake.all
""")

    # Check that a full `bazel test` passes.
    command = "test"
    if sys.platform.startswith("darwin"):
        # TODO(jwnimmer-tri) A `test //...` doesn't pass yet on macOS.
        command = "build"
    subprocess.check_call(cwd=scratch_dir, args=[
        "bazel",
        # Use "release engineering" options for hermeticity.
        # https://docs.bazel.build/versions/master/user-manual.html#bazel-releng
        "--bazelrc=/dev/null",
        # Encourage the server to exit after the test completes.
        "--max_idle_secs=1",
        # Run all of the tests from the BUILD.bazel generated above.
        command, "//...", "--jobs=1",
        # Enable verbosity.
        "--announce_rc",
        # Use "release engineering" options for hermeticity.
        "--nokeep_state_after_build",
        # Nerf the coverage reporter to avoid downloading the entire JDK afresh
        # from the internet every time we run this test.
        "--coverage_report_generator=//:dummy_filegroup",
        ])


if __name__ == '__main__':
    main()
