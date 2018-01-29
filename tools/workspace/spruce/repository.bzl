# -*- python -*-

def _impl(repository_ctx):
    # Bring in the BUILD file.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/spruce:package.BUILD.bazel"),
        "BUILD.bazel")
    # Bring in the source files from upstream.
    repository_ctx.symlink(
        Label("@drake//third_party:josephdavisco_spruce/LICENSE"),
        "LICENSE")
    repository_ctx.symlink(
        Label("@drake//third_party:josephdavisco_spruce/spruce.cc"),
        "spruce.cc")
    repository_ctx.symlink(
        Label("@drake//third_party:josephdavisco_spruce/spruce.hh"),
        "spruce.hh")

spruce_repository = repository_rule(
    implementation = _impl,
)
