# -*- python -*-

load("@drake//tools/workspace:git.bzl", "git_clone")

def _impl(repo_ctx):
    # Download the snopt sources from an access-controlled git repository.
    # We'll use git operations directly (and not the github_archive helper)
    # because github_archive does not (yet, easily) support authentication.
    git_clone(
        repo_ctx,
        remote = repo_ctx.attr.remote,
        commit = repo_ctx.attr.commit,
    )
    if repo_ctx.attr.use_drake_build_rules:
        # Disable any files that came from the upstream snopt source archive.
        result = repo_ctx.execute(["bash", "-c", """
            set -ex
            find . -name BUILD -print0 -o -name BUILD.bazel -print0 |
                xargs -t -n1 -0 -I{} \
                mv {} {}.upstream-ignored
        """])
        if result.return_code:
            fail("Error {} during {}".format(
                result.return_code, result.stdout + result.stderr))
        # Link Drake's BUILD file into the snopt workspace.
        repo_ctx.symlink(
            Label("@drake//tools/workspace/snopt:package.BUILD.bazel"),
            "BUILD")

snopt_repository = repository_rule(
    attrs = {
        "remote": attr.string(default = "git@github.com:RobotLocomotion/snopt.git"),  # noqa
        "commit": attr.string(default = "0f475624131c9ca4d5624e74c3f8273ccc926f9b"),  # noqa
        "use_drake_build_rules": attr.bool(default = True),
    },
    local = False,
    implementation = _impl,
)
