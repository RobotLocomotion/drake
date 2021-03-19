#!/usr/bin/env python3

"""
Generates data from $(git merge-base upstream/master HEAD), and shuffle some
thingers around.
"""

from contextlib import contextmanager
import os
from os.path import abspath, dirname, isdir, join
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


def generate(regression_data_dir):
    # Ensure we remove the folder (in case there were ignored files).
    shell(f"rm -rf {regression_data_dir}")

    shell("bazel build //tmp/benchmark:py/model_regression_test //tmp/benchmark:repro_anzu_issue")
    shell("./bazel-bin/tmp/benchmark/py/model_regression_test --regenerate")
    assert isdir(regression_data_dir), regression_data_dir

    repro_output = subshell("./bazel-bin/tmp/benchmark/repro_anzu_issue")
    with open(join(regression_data_dir, "repro_anzu_issue.output.txt"), "w") as f:
        f.write(repro_output.strip() + "\n")


class Results:
    # Would be nice to use dataclasses, but alas, Python 3.6 on Ubuntu 18.04 :(
    merge_base: str = None
    regression_data_dir: str = None
    _stashed_regression_data_dir: str = "/tmp/drake_stash_data"

    def assert_valid(self):
        assert self.merge_base is not None
        assert self.regression_data_dir is not None

    def stash_data(self):
        self.assert_valid()
        # Don't use `git stash` to avoid conflicts. We're just gonna blow away the
        # results.
        shell(f"rm -rf {self._stashed_regression_data_dir}")
        shell(f"mv {self.regression_data_dir} {self._stashed_regression_data_dir}")

    def unstash_data(self):
        # Blantaly use the new results, removing all old stuff.
        self.assert_valid()
        shell(f"rm -rf {self.regression_data_dir}")
        shell(f"mv {self._stashed_regression_data_dir} {self.regression_data_dir}")


def checkout_merge_base_build_and_stash_regression_artifacts(starting_ref):
    local_tmp_dir = "tmp"
    code_dir = "tmp/benchmark"
    non_restorable_dir = "tmp/repos"

    out = Results()
    out.regression_data_dir = "tmp/benchmark/data"

    if isdir(non_restorable_dir):
        raise UserError(dedent(f"""\
        The repo folder should be moved / removed, as it is not
        easily restored: {non_restorable_dir}
        """))

    out.merge_base = subshell("git merge-base upstream/master HEAD")
    shell(f"git checkout {out.merge_base}")
    shell("git log -n 1 --oneline --no-decorate")

    # Copy code directory (and have it be staged).
    assert not isdir(code_dir), code_dir
    shell(f"git checkout {starting_ref} -- {code_dir}")

    generate(out.regression_data_dir)
    out.stash_data()

    # Remove code directory to prevent too many conflicts.
    shell(f"git reset")
    shell(f"rm -rf {local_tmp_dir}")

    # Ensure we're not leaving anything.
    status_text = subshell("git --no-pager status -s")
    assert status_text == "", status_text

    out.assert_valid()
    return out


def main():
    source_tree = parent_dir(abspath(__file__), count=2)
    cd(source_tree)

    with safe_git_restore_context() as starting_ref:
        out = (
            checkout_merge_base_build_and_stash_regression_artifacts(
                starting_ref
            )
        )

    try:
        out.unstash_data()
        shell(f"git add -A {out.regression_data_dir}")
        shell(f"git commit -m 'Regenerate model_regression_test with merge-base {out.merge_base}'")

        # Replay benchmark.
        generate(out.regression_data_dir)
        shell(f"git add -A {out.regression_data_dir}")
        shell(f"git commit -m 'Regenerate on latest version of code'")
    except:
        # Hard reset to starting ref.
        shell(f"git reset --hard {starting_ref}")
        raise


if __name__ == "__main__":
    try:
        main()
    except UserError as e:
        eprint(e)
        sys.exit(1)
