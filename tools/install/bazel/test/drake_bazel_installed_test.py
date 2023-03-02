"""The acceptance test for //tools/install/bazel:install.
"""

import copy
import os
from os.path import join
import subprocess
import sys

import install_test_helper


def main():
    install_dir = install_test_helper.get_install_dir()

    # In scratch, mock up a drake_bazel_installed workspace.
    scratch_dir = install_test_helper.create_temporary_dir("scratch")

    # The commit (version) here should be identical to the commit listed in
    # drake/tools/workspace/rules_python/repository.bzl.
    rules_python_commit = "0.19.0"
    rules_python_urls = [
        f"https://github.com/bazelbuild/rules_python/archive/{rules_python_commit}.tar.gz",  # noqa
        f"https://drake-mirror.csail.mit.edu/github/bazelbuild/rules_python/{rules_python_commit}.tar.gz",  # noqa
    ]
    rules_python_sha256 = "ffc7b877c95413c82bfd5482c017edcf759a6250d8b24e82f41f3c8b8d9e287e"  # noqa

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
    urls = {rules_python_urls!r},
)

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
load("@drake//:.os.bzl", OS_NAME = "NAME")

cc_test(
    name = "text_logging_test",
    srcs = ["text_logging_test.cc"],
    copts = ["--std=c++17"],
    # TODO(jwnimmer-tri) On macOS, we need to pkg-config fmt for this to pass.
    # For the moment, we'll say that :drake_shared_library is Ubuntu-only.
    tags = ["manual"] if OS_NAME == "mac os x" else [],
    deps = ["@drake//:drake_shared_library"],
)

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

    with open(join(scratch_dir, "text_logging_test.cc"), "w") as f:
        f.write("""
#include <drake/common/text_logging.h>
int main(int argc, char** argv) {
  drake::log()->info("Hello {}", "world");
  return 0;
}
""")

    with open(join(scratch_dir, "find_resource_test.py"), "w") as f:
        f.write("""
from pydrake.common import FindResourceOrThrow, _set_log_level
_set_log_level("trace")
FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf")
""")

    with open(join(scratch_dir, "import_all_test.py"), "w") as f:
        f.write("""
import pydrake.all
""")

    # Check that a full `bazel test` passes.
    command = "test"
    env = copy.copy(os.environ)
    env["BAZELISK_HOME"] = env["TEST_TMPDIR"]
    if sys.platform.startswith("darwin"):
        # TODO(jwnimmer-tri) A `test //...` doesn't pass yet on macOS.
        command = "build"
    subprocess.check_call(cwd=scratch_dir, env=env, args=[
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
        "--test_output=streamed",
        # Use "release engineering" options for hermeticity.
        "--nokeep_state_after_build",
        # Nerf the coverage reporter to avoid downloading the entire JDK afresh
        # from the internet every time we run this test.
        "--coverage_report_generator=//:dummy_filegroup",
        ])


if __name__ == '__main__':
    main()
