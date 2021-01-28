"""Reports on which of Drake's external dependencies can be updated to a more
recent version.  This is intended for use by Drake maintainers (only).

This program is only supported on Ubuntu Bionic 18.04.

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
"""

import argparse
import getpass
import glob
import json
import logging
import os
import re
import subprocess
import sys
import time

import github3

from drake.tools.workspace.metadata import read_repository_metadata

# We'll skip these repositories when making suggestions.
_IGNORED_REPOSITORIES = [
    # We don't know how to check non-default branches yet.
    "clang_cindex_python3",
    "pybind11",
    # We've purposely pinned these to older revisions, as noted by comments in
    # their repository.bzl files.
    "fmt",
    "spdlog",
]

# For these repositories, we only look at tags, not releases.  For the dict
# value, use a blank value to match the latest tag or a regex to only select
# tags that share the match with the tag currently in use.  (This can be used
# to pin to a given major or major.minor release series.)
_OVERLOOK_RELEASE_REPOSITORIES = {
    "github3_py": r"^(\d+.)",
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


def _handle_github(workspace_name, gh, data):
    time.sleep(0.2)  # Don't make github angry.
    old_commit = data["commit"]
    owner, repo_name = data["repository"].split("/")
    gh_repo = gh.repository(owner, repo_name)

    # If we're tracking via git commit, then upgrade to the newest commit.
    if len(old_commit) == 40:
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


def run(gh, args, metadata):
    for workspace_name, data in sorted(metadata.items()):
        if workspace_name in _IGNORED_REPOSITORIES:
            continue
        key = data["repository_rule_type"]
        if key == "github":
            old_commit, new_commit = _handle_github(workspace_name, gh, data)
        elif key in ["pypi", "pypi_wheel"]:
            # TODO(jwnimmer-tri) Implement for real.
            print("{} version {} needs manual inspection".format(
                workspace_name, data["version"]))
            continue
        elif key == "manual":
            print("{} needs manual inspection".format(workspace_name))
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


def main():
    parser = argparse.ArgumentParser(prog="new_release", description=__doc__)
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

    # Grab the workspace metadata.
    print("Collecting bazel repository details...")
    metadata = read_repository_metadata()
    if args.verbose:
        print(json.dumps(metadata, sort_keys=True, indent=2))

    # Run our report.
    print("Checking for new releases...")
    run(gh, args, metadata)


if __name__ == '__main__':
    main()
