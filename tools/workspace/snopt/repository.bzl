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

snopt_repository = repository_rule(
    attrs = {
        "remote": attr.string(default = "git@github.com:RobotLocomotion/snopt.git"),  # noqa
        "commit": attr.string(default = "0f475624131c9ca4d5624e74c3f8273ccc926f9b"),  # noqa
    },
    local = False,
    implementation = _impl,
)
