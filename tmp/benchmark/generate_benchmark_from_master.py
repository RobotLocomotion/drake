#!/usr/bin/env python3

"""
Generates data from $(git merge-base upstream/master HEAD), and shuffle some
thingers around.
"""

from contextlib import contextmanager
import os
from os.path import abspath, dirname, isdir
from subprocess import run, PIPE
from textwrap import dedent
import sys


class UserError(RuntimeError):
    pass


def eprint(s):
    print(s, file=sys.stderr)


def shell(cmd, check=True):
    """Executes a shell command."""
    eprint(f"+ {cmd}")
    return run(cmd, shell=True, check=check)


def subshell(cmd, check=True, stderr=None, strip=True):
    """Executs a subshell in a capture."""
    eprint(f"+ $({cmd})")
    result = run(cmd, shell=True, stdout=PIPE, stderr=stderr, encoding="utf8")
    if result.returncode != 0 and check:
        if stderr == PIPE:
            eprint(result.stderr)
        eprint(result.stdout)
        raise UserError(f"Exit code {result.returncode}: {cmd}")
    out = result.stdout
    if strip:
        out = out.strip()
    return out


def cd(p):
    eprint(f"+ cd {p}")
    os.chdir(p)


def parent_dir(p, *, count):
    for _ in range(count):
        p = dirname(p)
    return p


@contextmanager
def safe_git_restore_context():
    # WARNING: This expects that your `gitignore` ignores a sufficient amount
    # of stuff to not get thrown off.
    # Ensure that there are no changes in Git that we will lose.
    status_text = subshell("git --no-pager status -s")
    if status_text != "":
        raise UserError(f"Dirty tree! Cannot proceed\n{status_text}")
    starting_ref = subshell("git rev-parse --abbrev-ref HEAD")
    if starting_ref == "HEAD":
        starting_ref = subshell("git rev-parse HEAD")
    eprint(f"Starting git ref: {starting_ref}")
    shell("git log -n 1 --oneline --no-decorate")
    try:
        yield starting_ref
    finally:
        # It is safe to do this since we've prevent usage of this script in a
        # dirty workspace.
        eprint(f"Returning to git ref: {starting_ref}")
        shell(f"git checkout -f {starting_ref}")


def checkout_merge_base_build_and_stash_regression_artifacts(starting_ref):
    local_tmp_dir = "tmp"
    code_dir = "tmp/benchmark"
    regression_data_dir = "tmp/benchmark/data"
    non_restorable_dir = "tmp/repos"

    if isdir(regression_data_dir):
        raise UserError(dedent(f"""\
        The regression data folder should not yet exist: {regression_data_dir}
        Please remove it and commit that state in git before
        running this script.
        """))

    if isdir(non_restorable_dir):
        raise UserError(dedent(f"""\
        The repo folder should be moved / removed, as it is not
        easily restored: {non_restorable_dir}
        """))

    merge_base = subshell("git merge-base upstream/master HEAD")
    shell(f"git checkout {merge_base}")
    shell("git log -n 1 --oneline --no-decorate")

    # Copy code directory (and have it be staged).
    assert not isdir(code_dir), code_dir
    shell(f"git checkout {starting_ref} -- {code_dir}")

    shell("bazel build //tmp/benchmark:py/model_regression_test")
    shell("./bazel-bin/tmp/benchmark/py/model_regression_test --regenerate")

    assert isdir(regression_data_dir), regression_data_dir

    # Stash the newly generated regression data.
    shell(f"git stash push -u --keep-index -- {regression_data_dir}")

    # Remove code directory.
    shell(f"git reset")
    shell(f"rm -rf {local_tmp_dir}")

    # Ensure we're not leaving anything.
    status_text = subshell("git --no-pager status -s")
    assert status_text == "", status_text

    return merge_base, regression_data_dir


def main():
    source_tree = parent_dir(abspath(__file__), count=3)
    cd(source_tree)

    with safe_git_restore_context() as starting_ref:
        merge_base, regression_data_dir = (
            checkout_merge_base_build_and_stash_regression_artifacts(
                starting_ref
            )
        )

    # Rebuild benchmark ('cause it's confusing otherwise...)
    shell("bazel build //tmp/benchmark:py/model_regression_test")

    shell("git stash pop")

    print(dedent(FR"""
        For new artifacts, suggested staging and commit commands:

            ( set -eux;
            cd {source_tree}
            git add -A {regression_data_dir}
            git commit -m \
                "Regenerate model_regression_test with merge-base {merge_base}"
            )

        To then regenerate the newer stuff:

            (set -eux;
            cd {source_tree}
            ./bazel-bin/tmp/benchmark/py/model_regression_test --regenerate
            git add -A {regression_data_dir}
            git commit -m "Regenerate on latest version of code"
            )
    """))


if __name__ == "__main__":
    try:
        main()
    except UserError as e:
        eprint(e)
        sys.exit(1)
