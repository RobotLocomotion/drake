load("//tools/workspace:github.bzl", "setup_github_repository")

# This repository rule is *almost* like our typical github_archive() rule,
# except that the `package.BUILD.bazel` file is copied from tensorflow instead
# of being written custom for Drake. Our only edit to the tensorflow file is
# to use a customized cc_library macro.

def _impl(repo_ctx):
    error = setup_github_repository(repo_ctx).error
    if error != None:
        fail(error)
    repo_ctx.file("BUILD.bazel", """
load("@drake//tools/workspace/libjpeg_turbo_internal:defs.bzl", "cc_library")
exports_files(["drake_repository_metadata.json"])
""" + repo_ctx.read(Label(
        "@drake//third_party:com_github_tensorflow_tensorflow/third_party/jpeg/jpeg.BUILD",  # noqa
    )))

libjpeg_turbo_internal_repository = repository_rule(
    attrs = {
        # These are the attributes for setup_github_repository.
        "repository": attr.string(default = "libjpeg-turbo/libjpeg-turbo"),
        "commit": attr.string(default = "2.1.4"),
        "commit_pin": attr.int(
            # We need to match our version of libjpeg-turbo to the BUILD file
            # we've adopted from tensorflow. Run `upgrade.py` to upgrade.
            default = 1,
        ),
        "sha256": attr.string(
            default = "a78b05c0d8427a90eb5b4eb08af25309770c8379592bb0b8a863373128e6143f",  # noqa
        ),
        "build_file": attr.label(),
        "patches": attr.label_list(),
        "extra_strip_prefix": attr.string(),
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
