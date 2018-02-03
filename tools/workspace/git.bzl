# -*- python -*-

def _run_git(repo_ctx, *args):
    result = repo_ctx.execute(["git", "--git-dir=.git"] + list(args))
    if result.return_code:
        fail('Repository rule @{} error {} during git operation: {}'.format(
            repo_ctx.name,
            result.return_code,
            repr(result.stdout + result.stderr),
        ))

def git_clone(
        repo_ctx,
        remote = None,
        commit = None):
    """Clone a git repository named by the given remote and reset it to the
    given commit.  On any error, reports via fail() and does not return.

    For the same reasons Bazel's new_git_repository() documentation explains,
      https://docs.bazel.build/versions/master/be/workspace.html#new_git_repository
    this rule should generally not be used.  The recommemded alternative is the
    github.bzl rules (in the same folder as this rule).
    """
    (commit and remote) or fail("Missing commit or remote")
    _run_git(repo_ctx, "clone", remote, ".")
    _run_git(repo_ctx, "reset", "--hard", commit)
