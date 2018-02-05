# -*- python -*-

def _run_git(repo_ctx, *args):
    # Runs the git operation named in *args.  We use "--git-dir" to prevent git
    # from looking in parent directories to find the repository.  We only ever
    # want it to look where we've told it.
    result = repo_ctx.execute(["git", "--git-dir=.git"] + list(args))
    if result.return_code:
        fail('Repository rule @{} error {} during git operation: {}'.format(
            repo_ctx.name,
            result.return_code,
            repr(result.stdout + result.stderr),
        ))

def _git_clone(
        repo_ctx,
        remote = None,
        commit = None):
    # Clones a git repository named by the given remote and reset it to the
    # given commit.
    #
    # The clone is placed into "." (which is the workspace being populated by
    # whatever repository_rule calls this function).  This function assumes
    # that "." is empty, which that's already been established by Bazel before
    # it invokes the repository_rule.
    #
    # On any error, terminates via fail() and does not return.
    #
    # We would prefer to load `@bazel_tools//tools/build_defs/repo:git.bzl` and
    # use its `git_repository` rule, but there is no flavor of that rule that
    # allows us to pass in the repo_ctx from our own repository_rule.
    (commit and remote) or fail("Missing commit or remote")
    _run_git(repo_ctx, "clone", remote, ".")
    _run_git(repo_ctx, "reset", "--hard", commit)

def _impl(repo_ctx):
    # Download the snopt sources from an access-controlled git repository.
    # We'll use git operations directly (and not the github_archive helper)
    # because github_archive does not (yet, easily) support authentication.
    #
    # TODO(#7240) Allow additional configuration to allow Drake developers and
    # users to choose archives other than git to provide the SNOPT source code.
    # This is why the current implementation here uses a repository_rule,
    # instead of simply calling Bazel's built-in new_git_repository.  Once we
    # need that additional configuration, we'll no longer be able to re-use
    # Bazel's git helpers.
    _git_clone(
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
