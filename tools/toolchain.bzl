# -*- python -*-
# This is a Bazel repository_rule for auto-configuring the C++ toolchain.
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html
# It tweaks the default auto-configuration to add extra compiler flags.
# https://github.com/bazelbuild/bazel/blob/master/tools/cpp/cc_configure.bzl

def _drake_toolchain_impl(repository_ctx):
    repository_ctx.symlink(Label("@local_config_cc//:BUILD"), "BUILD")
    repository_ctx.symlink(Label("@local_config_cc//:cc_wrapper.sh"),
                           "cc_wrapper.sh")
    repository_ctx.symlink(Label("@local_config_cc//:CROSSTOOL"),
                           "CROSSTOOL.generic")
    repository_ctx.symlink(Label("//tools:amend_crosstool.sh"),
                                 "amend_crosstool.sh")
    repository_ctx.symlink(Label("//tools:gcc_options.txt"),
                           "gcc_options.txt")
    repository_ctx.symlink(Label("//tools:clang_options.txt"),
                           "clang_options.txt")
    res = repository_ctx.execute(
        ["bash", "amend_crosstool.sh", "CROSSTOOL.generic"])
    if res.return_code != 0:
        print("toolchain.bzl: Failed to detect supported compiler.")

drake_toolchain_configuration = repository_rule(
    local = True,
    implementation = _drake_toolchain_impl,
)

def drake_toolchain():
    drake_toolchain_configuration(name="local_config_drake")
    native.bind(name="drake_toolchain",
                actual="@local_config_drake//:toolchain")
