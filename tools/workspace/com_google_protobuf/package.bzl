# -*- mode: python -*-

# This rule imports the protobuf skylark code (the upstream protobuf.bzl) under
# the name @com_google_protobuf, as is conventional.  Within that repository,
# we then use a custom BUILD.bazel file to provide the well-known labels
# ":protoc", ":protobuf", and ":protobuf_python" as aliases to the operating
# system's default installation of protobuf, for improved compatibility across
# Drake's ecosystem.

def _impl(repository_ctx):
    # Bring in our hand-written BUILD file for @com_google_protobuf.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/com_google_protobuf:package.BUILD.bazel"),  # noqa
        "BUILD.bazel")
    # Bring in two vendored files from upstream.
    repository_ctx.symlink(
        Label("@drake//third_party:com_github_google_protobuf/protobuf.bzl"),
        "protobuf.bzl")
    repository_ctx.symlink(
        Label("@drake//third_party:com_github_google_protobuf/LICENSE"),
        "LICENSE")

com_google_protobuf_repository = repository_rule(
    implementation = _impl,
)
