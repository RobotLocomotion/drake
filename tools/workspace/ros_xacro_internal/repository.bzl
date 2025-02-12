load("//tools/workspace:github.bzl", "github_archive")

def ros_xacro_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ros/xacro",
        commit = "2.0.12",
        sha256 = "396cad4883b0214f1c77a57c2935ac46f3f8300626034bdd2e22ce3a5874e4f6",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/disable-import-warning.patch",
        ],
        mirrors = mirrors,
    )
