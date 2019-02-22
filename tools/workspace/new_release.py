"""Reports on which of Drake's external dependencies can be updated to a more
recent version.  This is intended for use by Drake maintainers (only).

This program is only supported on Ubuntu Bionic 18.04.

To query GitHub APIs, you'll need to authenticate yourself first.  There are
two ways to do this:

(1) Type in your password each time you run this program:

  bazel build --config python3 //tools/workspace:new_release
  bazel-bin/tools/workspace/new_release

(2) Use a GitHub API token:

  bazel build --config python3 //tools/workspace:new_release
  env GITHUB_API_TOKEN=$(cat ~/.config/readonly_github_api_token.txt) \
    bazel-bin/tools/workspace/new_release

To create the ~/.config/readonly_github_api_token.txt file, open a browser to
https://github.com/settings/tokens and create a new token (it does not need any
extra permissions; the default "no checkboxes are set" is good), and save the
plaintext hexidecimal token to that file.
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
    # N.B. See #5785; do your best not to bump this to a newer commit.
    "eigen",
    # Our "find something new" heuristics get confused by these repositories.
    "github3_py",
    "pycodestyle",
    # Until Drake has real uses of godotengine, we don't want to mess with it.
    "godotengine",
]


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


def _handle_github(gh, data):
    time.sleep(0.2)  # Don't make github angry.
    old_commit = data["commit"]
    owner, repo_name = data["repository"].split("/")
    gh_repo = gh.repository(owner, repo_name)
    if len(old_commit) == 40:
        new_commit = gh_repo.commit("HEAD").sha
    else:
        try:
            new_commit = gh_repo.latest_release().tag_name
        except github3.exceptions.NotFoundError:
            new_commit = next(gh_repo.tags()).name
    return old_commit, new_commit


def run(gh, args, metadata):
    for name, data in sorted(metadata.items()):
        if name in _IGNORED_REPOSITORIES:
            continue
        key = data["repository_rule_type"]
        if key == "github":
            old_commit, new_commit = _handle_github(gh, data)
        elif key == "bitbucket":
            # TODO(jwnimmer-tri) Implement for real.
            old_commit = data["commit"]
            new_commit = None
        else:
            raise RuntimeError("Bad key " + key)
        if old_commit == new_commit:
            continue
        elif new_commit is not None:
            print("{} needs upgrade from {} to {}".format(
                data["name"], old_commit, new_commit))
        else:
            print("{} version {} needs manual inspection".format(
                data["name"], old_commit))


def main():
    token = os.getenv("GITHUB_API_TOKEN", None)
    parser = argparse.ArgumentParser(prog="new_release", description=__doc__)
    if sys.version_info[0] != 3:
        parser.error("\n".join([
            "Wrong python version.",
            "This script only supports python3.",
            "To compile it in that mode, use this command:",
            " bazel build --config python3 //tools/workspace:new_release"]))
    parser.add_argument(
        "--use_token", action="store_true", default=(token is not None),
        help="When set, uses an API token instead of username + password")
    parser.add_argument(
        "--user", metavar="USER", type=str, default=_get_default_username(),
        help="GitHub username (default: %(default)s)")
    parser.add_argument(
        "--verbose", action="store_true", default=False)
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    if args.use_token and not token:
        parser.error("Missing environment variable GITHUB_API_TOKEN")
    elif not args.user:
        parser.error("Couldn't guess github username; you must supply --user.")

    # Log in to github.
    if args.use_token:
        gh = github3.login(token=token)
    else:
        prompt = "Password for https://{}@github.com: ".format(args.user)
        gh = github3.login(
            username=args.user,
            password=getpass.getpass(prompt))

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
