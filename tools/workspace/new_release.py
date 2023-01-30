"""Reports on which of Drake's external dependencies can be updated to a more
recent version.  This is intended for use by Drake maintainers (only).

This program is only supported on Ubuntu Jammy 22.04.

To query GitHub APIs, you'll need to authenticate yourself first.  There are
two ways to do this:

(1) Type in your password each time you run this program:

  bazel build //tools/workspace:new_release
  bazel-bin/tools/workspace/new_release --use_password

(2) Use a GitHub API token:

  bazel build //tools/workspace:new_release
  bazel-bin/tools/workspace/new_release

To create the ~/.config/readonly_github_api_token.txt file used by (2), open a
browser to https://github.com/settings/tokens and create a new token (it does
not need any extra permissions; the default "no checkboxes are set" is good),
and save the plaintext hexadecimal token to that file.

This program can also automatically prepare upgrades for our GitHub externals
by passing the name(s) of package(s) to upgrade as additional arguments:

  bazel build //tools/workspace:new_release
  bazel-bin/tools/workspace/new_release --lint --commit rules_python

Note that this program runs `bazel` as a subprocess, without any special
command line flags.  If you do need to use any flags when you run bazel,
then those must be added to an rcfile; they cannot be provided on the
command line.
"""

import argparse
import getpass
import git
import glob
import hashlib
import json
import logging
import os
import re
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import urllib

import github3

from drake.tools.workspace.metadata import read_repository_metadata

# We'll skip these repositories when making suggestions.
_IGNORED_REPOSITORIES = [
    # We don't know how to check non-default branches yet.
    "clang_cindex_python3_internal",
    "gym_py",  # Pinned at 0.21; see tools/workspace/gym_py/README.md.
    "pybind11",
]

# For these repositories, we only look at tags, not releases.  For the dict
# value, use a blank value to match the latest tag or a regex to only select
# tags that share the match with the tag currently in use; the parentheses
# group in the regex denotes the portion of the tag to lock as invariant.
# (This can be used to pin to a given major or major.minor release series.)
_OVERLOOK_RELEASE_REPOSITORIES = {
    "github3_py_internal": r"^(\d+.)",
    "gz_math_internal": "^(gz)",
    "gz_utils_internal": "^(gz)",
    "intel_realsense_ros_internal": r"^(\d+\.\d+\.)",
    "petsc": r"^(v)",
    "pycodestyle": "",
    "qhull_internal": r"^(2)",
    "ros_xacro_internal": r"^(\d+\.\d+\.)",
    "sdformat_internal": "",
}

# Packages in these cohorts should be upgraded together (in a single commit).
_COHORTS = (
    # The uwebsockets depends on usockets;
    # be sure to keep them in alignment.
    ("uwebsockets", "usockets"),
    # The sdformat depends on both gz libraries;
    # be sure to keep them in alignment.
    ("sdformat_internal", "gz_math_internal", "gz_utils_internal"),
)


def _check_output(args):
    return subprocess.check_output(args).decode("utf8")


def _get_default_username():
    origin_url = _check_output(
        ["git", "config", "--get", "remote.origin.url"]).strip()
    # Match one of these two cases:
    #  git@github.com:user/drake.git
    #  https://user@github.com/user/drake.git
    match = re.search(r"(github.com:(.*?)/|/(.*?)@github.com)", origin_url)
    if not match:
        return None
    _, git_user, http_user = match.groups()
    return git_user or http_user


def _smells_like_a_git_commit(revision):
    """Returns true iff revision seems to be a git commit (as opposed to
    a version number tag name).  This might produce false positives for
    very long version numbers, but we've never seen that in practice.
    """
    return len(revision) == 40


def _is_prerelease(commit, workspace):
    """Returns true iff commit seems to be a pre-release
    """
    development_stages = ["alpha", "beta", "rc", "pre"]
    prerelease = any(stage in commit for stage in development_stages)
    if prerelease:
        print("Skipping prerelease {} for {}".format(commit, workspace))
    return prerelease


def _latest_tag(gh_repo, workspace):
    for tag in gh_repo.tags():
        if _is_prerelease(tag.name, workspace):
            continue
        return tag.name
    print("Could not find any matching tags for {}".format(workspace))
    return None


def _handle_github(workspace_name, gh, data):
    time.sleep(0.2)  # Don't make github angry.
    old_commit = data["commit"]
    new_commit = None
    owner, repo_name = data["repository"].split("/")
    gh_repo = gh.repository(owner, repo_name)

    # If we're tracking via git commit, then upgrade to the newest commit.
    if _smells_like_a_git_commit(old_commit):
        new_commit = gh_repo.commit("HEAD").sha
        return old_commit, new_commit

    # Sometimes prefer checking only tags, not releases.
    tags_pattern = _OVERLOOK_RELEASE_REPOSITORIES.get(workspace_name)
    if tags_pattern == "":
        new_commit = _latest_tag(gh_repo, workspace_name)
        return old_commit, new_commit

    # Sometimes limit candidate tags to those matching a regex.
    if tags_pattern is not None:
        match = re.search(tags_pattern, old_commit)
        assert match, f"No {tags_pattern} in {old_commit}"
        (old_hit,) = match.groups()
        for tag in gh_repo.tags():
            match = re.search(tags_pattern, tag.name)
            if match:
                (new_hit,) = match.groups()
                if old_hit == new_hit:
                    if _is_prerelease(tag.name, workspace_name):
                        continue
                    new_commit = tag.name
                    break
        return old_commit, new_commit

    # By default, use the latest release if there is one.  Otherwise, use the
    # latest tag.
    try:
        new_commit = gh_repo.latest_release().tag_name
        if _is_prerelease(new_commit, workspace_name):
            new_commit = _latest_tag(gh_repo, workspace_name)
    except github3.exceptions.NotFoundError:
        new_commit = _latest_tag(gh_repo, workspace_name)
    return old_commit, new_commit


def _handle_buildifier(gh, data):
    assert data["name"] == "buildifier"
    old_version = data["version"]
    time.sleep(0.2)  # Don't make github angry.
    gh_repo = gh.repository("bazelbuild", "buildtools")
    new_version = next(gh_repo.tags()).name
    return old_version, new_version


def _check_for_upgrades(gh, args, metadata):
    for workspace_name, data in sorted(metadata.items()):
        if workspace_name in _IGNORED_REPOSITORIES:
            continue
        if data.get("version_pin"):
            continue
        key = data["repository_rule_type"]
        if key == "github":
            old_commit, new_commit = _handle_github(workspace_name, gh, data)
        elif key in ["pypi", "pypi_wheel"]:
            # TODO(jwnimmer-tri) Implement for real.
            print("{} version {} needs manual inspection".format(
                workspace_name, data["version"]))
            continue
        elif workspace_name == "buildifier":
            assert key == "manual"
            old_commit, new_commit = _handle_buildifier(gh, data)
        elif key == "manual":
            print("{} version {} needs manual inspection".format(
                workspace_name, data.get("version", "???")))
            continue
        else:
            raise RuntimeError("Bad key " + key)
        if old_commit == new_commit:
            continue
        elif new_commit is not None:
            print("{} needs upgrade from {} to {}".format(
                workspace_name, old_commit, new_commit))
        else:
            print("{} version {} needs manual inspection".format(
                workspace_name, old_commit))


def _is_modified(repo, path):
    """Returns true iff the given `path` is modified in the working tree of the
    given `git.Repo`, `repo`.
    """
    for other in [None, 'HEAD']:
        if path in [item.b_path for item in repo.index.diff(other)]:
            return True
    return False


def _do_upgrade(temp_dir, gh, local_drake_checkout,
                workspace_names, metadata, commit_changes):

    # Make sure there are workspaces to update.
    if len(workspace_names) == 0:
        return

    for workspace_name in workspace_names:
        if workspace_name not in metadata:
            raise RuntimeError(f"Unknown repository {workspace_name}")
        if metadata[workspace_name]["repository_rule_type"] != "github":
            raise RuntimeError(f"Cannot auto-upgrade {workspace_name}")

    messages = []
    bzl_filenames = []
    for workspace_name in workspace_names:
        data = metadata[workspace_name]
        repository = data["repository"]

        # Slurp the file we're supposed to modify.
        bzl_filename = f"tools/workspace/{workspace_name}/repository.bzl"
        with open(bzl_filename, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # Figure out what to upgrade.
        old_commit, new_commit = _handle_github(workspace_name, gh, data)
        if old_commit == new_commit:
            raise RuntimeError(f"No upgrade needed for {workspace_name}")
        elif new_commit is None:
            raise RuntimeError(f"Cannot auto-upgrade {workspace_name}")
        print("Upgrading {} from {} to {}".format(
            workspace_name, old_commit, new_commit))

        # Locate the two hexadecimal lines we need to edit.
        commit_line_re = re.compile(
            r'(?<=    commit = ")(' + re.escape(old_commit) + r')(?=",)')
        checksum_line_re = re.compile(
            r'(?<=    sha256 = ")([0-9a-f]{64})(?=",)')
        commit_line_num = None
        checksum_line_num = None
        for i, line in enumerate(lines):
            match = commit_line_re.search(line)
            if match:
                assert commit_line_num is None
                commit_line_num = i
                commit_line_match = match
            match = checksum_line_re.search(line)
            if match:
                assert checksum_line_num is None
                checksum_line_num = i
                checksum_line_match = match
        assert commit_line_num is not None
        assert checksum_line_num is not None

        # Download the new source archive.
        print("Downloading new archive...")
        new_url = f"https://github.com/{repository}/archive/{new_commit}.tar.gz"  # noqa: E501
        hasher = hashlib.sha256()
        with open(f"{temp_dir}/{new_commit}.tar.gz", "wb") as temp:
            with urllib.request.urlopen(new_url) as response:
                while True:
                    data = response.read(4096)
                    if not data:
                        break
                    hasher.update(data)
                    temp.write(data)
                new_checksum = hasher.hexdigest()

        # Update the repository.bzl contents and then write it out.
        lines[commit_line_num] = commit_line_re.sub(
            new_commit, lines[commit_line_num])
        lines[checksum_line_num] = checksum_line_re.sub(
            new_checksum, lines[checksum_line_num])
        with open(bzl_filename + ".new", "w", encoding="utf-8") as f:
            for line in lines:
                f.write(line)
        os.rename(bzl_filename + ".new", bzl_filename)
        bzl_filenames.append(bzl_filename)

        # Copy the downloaded tarball into the repository cache.
        print("Populating repository cache ...")
        subprocess.check_call(["bazel", "fetch",
                               "//...", f"--distdir={temp_dir}"])

        message = f"[workspace] Upgrade {workspace_name}"
        if _smells_like_a_git_commit(new_commit):
            message += " to latest commit"
        else:
            message += f" to latest release {new_commit}"
        messages.append(message)

    if commit_changes:
        msg_string = "\n".join(messages)
        local_drake_checkout.git.commit('-o', bzl_filenames, '-m', msg_string)
        print("\n" + ("*" * 72))
        workspace_names = list(workspace_names)
        if len(workspace_names) == 1:
            wn_string = workspace_names[0]
        elif len(workspace_names) == 2:
            wn_string = workspace_names[0] + " and " + workspace_names[1]
        else:
            wn_string = ", ".join(workspace_names[:-1])\
                        + "and " + workspace_names[-1]
        print(f"Done.  Changes for {wn_string} were committed.")
        print("Be sure to review the changes and amend the commit if needed.")
        print(("*" * 72) + "\n")
    else:
        print("\n" + ("*" * 72))
        print("Done.  Be sure to review and commit the changes:")
        print(f"  git add {' '.join(bzl_filenames)}")
        msg_string = '\\\n'.join(messages)
        print(f"  git commit -m '{msg_string}'")
        print(("*" * 72) + "\n")


def main():
    parser = argparse.ArgumentParser(
        prog="new_release", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--commit", action="store_true", default=False,
        help="When upgrading repositories, automatically commit the changes.")
    parser.add_argument(
        "--lint", action="store_true", default=False,
        help="Also run some sanity tests on the repository, after all other"
             " operations have completed successfully.")
    parser.add_argument(
        "--use_password", action="store_true", default=False,
        help="Prompt for the GitHub password, instead of using an API token.")
    parser.add_argument(
        "--token_file", default="~/.config/readonly_github_api_token.txt",
        help="Uses an API token read from this filename, unless "
        "--use_password was given (default: %(default)s)")
    parser.add_argument(
        "--user", metavar="USER", type=str, default=_get_default_username(),
        help="GitHub username (default: %(default)s)")
    parser.add_argument(
        "--verbose", action="store_true", default=False)
    parser.add_argument(
        "workspace", nargs="*", metavar="WORKSPACES_NAME", type=str,
        help="(Optional) Instead of reporting on possible upgrades,"
             " download new archives for the given externals"
             " and edit their bzl rules to match.")
    args = parser.parse_args()

    if not os.path.exists('WORKSPACE'):
        parser.error("Couldn't find WORKSPACE; this script must be run"
                     " from the root of your Drake checkout.")

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    if args.use_password and not args.user:
        parser.error("Couldn't guess github username; you must supply --user.")

    # Log in to github.
    if args.use_password:
        prompt = "Password for https://{}@github.com: ".format(args.user)
        gh = github3.login(
            username=args.user,
            password=getpass.getpass(prompt))
    else:
        with open(os.path.expanduser(args.token_file), "r") as f:
            token = f.read().strip()
        gh = github3.login(token=token)

    # Are we operating on all repositories, or just one?
    if len(args.workspace):
        workspaces = args.workspace
    else:
        if args.commit:
            parser.error("--commit requires one or more workspaces.")
        # (None denotes "all".)
        workspaces = None

    if args.commit:
        local_drake_checkout = git.Repo(os.path.realpath("."))
    else:
        local_drake_checkout = None

    # Grab the workspace metadata.
    print("Collecting bazel repository details...")
    metadata = read_repository_metadata(repositories=workspaces)
    if args.verbose:
        print(json.dumps(metadata, sort_keys=True, indent=2))

    if workspaces is not None:
        # Generate a set of workspaces which are members of a cohort and
        # need to be updated.
        available_updates = set()
        for cohort in _COHORTS:
            for workspace in cohort:
                data = read_repository_metadata(repositories=[workspace])
                old_commit, new_commit = \
                    _handle_github(workspace, gh, data[workspace])
                if old_commit != new_commit and new_commit is not None:
                    available_updates.add(workspace)

        for workspace in workspaces:
            for cohort in _COHORTS:
                if workspace in cohort:
                    # Make sure all members of a cohort are being updated
                    # together if possible.
                    for cohort_item in cohort:
                        if cohort_item not in workspaces \
                           and cohort_item in available_updates:
                            parser.error(f"Members of cohort {cohort} must be"
                                         " upgraded together if possible")

        workspaces_to_commit = set()
        workspaces_dont_commit = set()
        for workspace in workspaces:
            # Don't try to update things which have no available updates
            if workspace not in available_updates:
                raise RuntimeError(f"No upgrade needed for {workspace}")
                continue

            # Determine if we should and can commit the changes made.
            if local_drake_checkout is not None:
                bzl_filename = f"tools/workspace/{workspace}/repository.bzl"
                if _is_modified(local_drake_checkout, bzl_filename):
                    print(f"{bzl_filename} has local changes.")
                    print(f"Changes made for {workspace}"
                          " will NOT be committed.")
                    workspaces_dont_commit.add(workspace)
                else:
                    workspaces_to_commit.add(workspace)
            else:
                workspaces_dont_commit.add(workspace)

        with TemporaryDirectory(prefix='drake_new_release_') as temp_dir:
            _do_upgrade(temp_dir, gh, local_drake_checkout,
                        workspaces_to_commit, metadata, commit_changes=True)
            _do_upgrade(temp_dir, gh, local_drake_checkout,
                        workspaces_dont_commit, metadata, commit_changes=False)
    else:
        # Run our report of what's available.
        print("Checking for new releases...")
        _check_for_upgrades(gh, args, metadata)

    if args.lint:
        subprocess.check_call(["bazel", "test", "--config=lint", "//..."])


if __name__ == '__main__':
    main()
