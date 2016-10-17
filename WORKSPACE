# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# http://bazel.io/ .

workspace(name = "drake_distro")

new_http_archive(
    name = "gtest",
    url = "https://googletest.googlecode.com/files/gtest-1.7.0.zip",
    sha256 = "247ca18dd83f53deb1328be17e4b1be31514cedfc1e3424f672bf11fd7e0d60d",
    build_file = "tools/gtest.BUILD",
    strip_prefix = "gtest-1.7.0",
)

new_git_repository(
    name = "eigen",
    remote = "https://github.com/RLovelett/eigen.git",
    tag = "3.3-beta1",
    build_file = "tools/eigen.BUILD",
)
