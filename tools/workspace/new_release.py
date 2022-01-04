"""Reports on which of Drake's external dependencies can be updated to a more
recent version.  This is intended for use by Drake maintainers (only).

This program is only supported on Ubuntu Bionic 20.04.

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

This program can also automatically prepare an upgrade for one of our GitHub
externals via the --upgrade option:

  bazel build //tools/workspace:new_release
  bazel-bin/tools/workspace/new_release --upgrade=rules_python

Note that this program runs `bazel` as a subprocess, without any special
command line flags.  If you do need to use any flags when you run bazel,
then those must be added to an rcfile; they cannot be provided on the
command line.
"""

import argparse
import getpass
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
    "clang_cindex_python3",
    "pybind11",
]

# For these repositories, we only look at tags, not releases.  For the dict
# value, use a blank value to match the latest tag or a regex to only select
# tags that share the match with the tag currently in use.  (This can be used
# to pin to a given major or major.minor release series.)
_OVERLOOK_RELEASE_REPOSITORIES = {
    "github3_py": r"^(\d+.)",
    "intel_realsense_ros": r"^(\d+\.\d+\.)",
    "pycodestyle": "",
    "ros_xacro": r"^(\d+\.\d+\.)",
}


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


def _handle_github(workspace_name, gh, data):
    time.sleep(0.2)  # Don't make github angry.
    old_commit = data["commit"]
    owner, repo_name = data["repository"].split("/")
    gh_repo = gh.repository(owner, repo_name)

    # If we're tracking via git commit, then upgrade to the newest commit.
    if _smells_like_a_git_commit(old_commit):
        new_commit = gh_repo.commit("HEAD").sha
        return old_commit, new_commit

    # Sometimes prefer checking only tags, not releases.
    tags_pattern = _OVERLOOK_RELEASE_REPOSITORIES.get(workspace_name)
    if tags_pattern == "":
        new_commit = next(gh_repo.tags()).name
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
                    new_commit = tag.name
                    break
        return old_commit, new_commit

    # By default, use the latest release if there is one.  Otherwise, use the
    # latest tag.
    try:
        new_commit = gh_repo.latest_release().tag_name
    except github3.exceptions.NotFoundError:
        new_commit = next(gh_repo.tags()).name
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


def _do_upgrade(temp_dir, gh, workspace_name, metadata):
    if workspace_name not in metadata:
        raise RuntimeError(f"Unknown repository {workspace_name}")
    data = metadata[workspace_name]
    if data["repository_rule_type"] != "github":
        raise RuntimeError(f"Cannot auto-upgrade {workspace_name}")
    repository = data["repository"]

    # Figure out what to upgrade.
    old_commit, new_commit = _handle_github(workspace_name, gh, data)
    if old_commit == new_commit:
        raise RuntimeError(f"No upgrade needed for {workspace_name}")
    print("Upgrading {} from {} to {}".format(
        workspace_name, old_commit, new_commit))

    # Slurp the file we're supposed to modify.
    bzl_filename = f"tools/workspace/{workspace_name}/repository.bzl"
    with open(bzl_filename, "r", encoding="utf-8") as f:
        lines = f.readlines()

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
    new_url = f"https://github.com/{repository}/archive/{new_commit}.tar.gz"
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

    # Copy the downloaded tarball into the repository cache.
    print("Populating repository cache ...")
    subprocess.check_call(["bazel", "fetch", "//...", f"--distdir={temp_dir}"])

    print("Done.  Be sure to review and commit the changes:")
    message = f"[workspace] Upgrade {workspace_name}"
    if _smells_like_a_git_commit(new_commit):
        message += " to latest commit"
    else:
        message += f" to latest release {new_commit}"
    print(f"  git add {bzl_filename}")
    print(f"  git commit -m'{message}'")


def main():
    parser = argparse.ArgumentParser(
        prog="new_release", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
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
        "--upgrade", metavar="REPOSITORY_NAME", type=str,
        help="(Optional) Instead of reporting on possible upgrades, download"
             " a new archive for the given repository and edit its bzl rules"
             " to match.")
    args = parser.parse_args()
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
    if args.upgrade:
        repositories = [args.upgrade]
    else:
        # (None denotes "all".)
        repositories = None

    # Grab the workspace metadata.
    print("Collecting bazel repository details...")
    metadata = read_repository_metadata(repositories=repositories)
    if args.verbose:
        print(json.dumps(metadata, sort_keys=True, indent=2))

    if args.upgrade:
        with TemporaryDirectory(prefix='drake_new_release_') as temp_dir:
            _do_upgrade(temp_dir, gh, args.upgrade, metadata)
    else:
        # Run our report of what's available.
        print("Checking for new releases...")
        _check_for_upgrades(gh, args, metadata)


if __name__ == '__main__':
    main()
