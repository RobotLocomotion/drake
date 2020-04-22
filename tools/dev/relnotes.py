"""Tool to help draft doc/release_notes/*.rst entries.
This is intended for use by Drake maintainers (only).
This program is only supported on Ubuntu Bionic 18.04.

The usage of this tools is outlined in the "201912 Drake release playbook"
document on Google Drive.

The tool reads the commit history of github.com/RobotLocomotion/drake using
the GitHub APIs -- the status of your current git clone is ignored.

To query GitHub APIs, you'll need to authenticate yourself first,
via a GitHub API token:

  bazel build //tools/dev:relnotes
  bazel-bin/tools/dev/relnotes

To create the required ~/.config/readonly_github_api_token.txt file, open a
browser to https://github.com/settings/tokens and create a new token (it does
not need any extra permissions; the default "no checkboxes are set" is good),
and save the plaintext hexadecimal token to that file.
"""

import argparse
import logging
import os
import re
import sys
import time

import github3


def _format_commit(gh, commit):
    # Grab the commit message subject and body.
    message = commit.message
    lines = message.splitlines()
    subject = lines[0]

    # See if the subject contained the pull request number.
    match = re.match(r'^(.*) +\(#([\d]+)\)$', subject)
    if match:
        # Yes; just use it.
        summary, pr = match.groups()
        summary = summary.strip()
    else:
        # No number in subject, so we'll have to look for PRs that mention this
        # commit.  Usually there is just the one PR in the search result (the
        # one that merged it), but sometimes other PRs might mention it (e.g.,
        # for reverts).
        summary = subject
        results = list(gh.search_issues(
            f"{commit.sha} is:pr is:merged repo:RobotLocomotion/drake",
            number=2))
        if len(results) == 1:
            pr = results[0].number
        else:
            # We don't know the PR number, so we'll leave the sha there for
            # someone to clean up later.
            pr = commit.sha

    # Remove subject line trailing period(s).
    nice_summary = summary.rstrip(".")

    # If there is a commit message body, turn it into some end-of-line text
    # that we'll have to editorialize by hand.
    detail = " ".join([x.strip() for x in lines[1:] if x])
    # When squashing, reviewable repeats the subject in the body.
    redundant_detail = f"* {summary}"
    if detail.startswith(redundant_detail):
        detail = detail[len(redundant_detail):].strip()
    if detail:
        detail = f"  # {detail}"

    # Format as top-level rst bullet point.
    return f"* {nice_summary} (`#{pr}`_){detail}"


def main():
    parser = argparse.ArgumentParser(prog="relnotes", description=__doc__)
    parser.add_argument(
        "--newest_commit", type=str, required=True,
        help="The commit sha of the newest commit to annotate (inclusive).")
    parser.add_argument(
        "--oldest_commit", type=str, required=True,
        help="The commit sha of the oldest commit to annotate (inclusive).")
    parser.add_argument(
        "--max_num_commits", type=int, default=400,
        help="Stop after chasing this many commits")
    parser.add_argument(
        "--token_file", default="~/.config/readonly_github_api_token.txt",
        help="Uses an API token read from this filename (default: "
        "%(default)s)")
    parser.add_argument(
        "--verbose", action="store_true", default=False)
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    if len(args.newest_commit) != 40:
        parser.error("Missing required 40-character sha for newest_commit.")
    if len(args.oldest_commit) != 40:
        parser.error("Missing required 40-character sha for oldest_commit.")
    with open(os.path.expanduser(args.token_file), "r") as f:
        token = f.read().strip()

    gh = github3.login(token=token)
    drake = gh.repository("RobotLocomotion", "drake")

    # Find all the commits from newest to oldest.
    commits = []
    for commit in drake.commits(sha=args.newest_commit):
        sys.stdout.write(".")
        commits.append(commit)
        if commit.sha == args.oldest_commit:
            break
        if len(commits) == args.max_num_commits:
            print("error: Reached max_num_commits")
            sys.exit(1)
    sys.stdout.write("\n")

    # Iterate from oldest to newest.
    for commit in reversed(commits):
        if len(commit.parents) > 1:
            logging.debug(f"Skipping merge commit {commit.sha}")
            continue
        print(_format_commit(gh, commit))
        # Try not to hit GitHub API rate limits.
        time.sleep(0.2)


if __name__ == '__main__':
    main()
