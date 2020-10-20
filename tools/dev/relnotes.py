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
import os.path
import re
import sys
import time

import github3


def _filename_to_primary_package(filename):
    """Given a filename (e.g., "systems/framework/system.h"), return its
    primary package (e.g., "systems").
    """
    segments = filename.split("/")
    if len(segments) > 1:
        return segments[0]
    else:
        # For "." just make something up.
        return "tools"


def _format_commit(gh, drake, commit):
    """Returns (packages, bullet) for the given commit.

    The packages is a list of top-level directories whose files were edited in
    this commit.

    The bullet is a "* Detail (#123)" summary of the change for release notes.
    """
    # Grab the commit message subject and body.
    message = commit.message
    lines = message.splitlines()
    subject = lines[0]

    # Figure out which top-level package(s) were changed, to help the release
    # notes author sort things better.
    comparison = drake.compare_commits(
        base=commit.parents[0].get('sha'), head=commit.sha)
    committed_files = [x.get('filename') for x in comparison.files]
    packages = sorted(set([
        _filename_to_primary_package(x) for x in committed_files
    ])) or ["tools"]
    if len(packages) > 1 and "bindings" in packages:
        packages.remove("bindings")

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
        time.sleep(0.2)  # Try not to hit GitHub API rate limits.
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
    return packages, f"* TBD {nice_summary} (`#{pr}`_){detail}"


def _update(args, rst_filename, gh, drake):
    """The --update action."""

    # Read in the existing content.
    with open(rst_filename, "r") as f:
        lines = f.readlines()

    # Scrape the last commit from the document.  The line looks like this:
    #   Current newest_commit {sha} (inclusive).
    prior_newest_commit = None
    prior_newest_commit_line = None
    for i, one_line in enumerate(lines):
        pattern = r"Current newest_commit ([0-9a-f]{40}) .inclusive."
        match = re.search(pattern, one_line)
        if match:
            assert not prior_newest_commit
            (prior_newest_commit,) = match.groups()
            prior_newest_commit_line = i
    if not prior_newest_commit_line:
        raise RuntimeError("Coult not find newest_commit inclusive")
    logging.debug(f"Prior newest_commit {prior_newest_commit}")

    # Find new commits from newest to oldest.
    commits = []
    for commit in drake.commits(sha='master'):
        if commit.sha == prior_newest_commit:
            break
        commits.append(commit)
        if len(commits) == args.max_num_commits:
            raise RuntimeError("Reached max_num_commits")

    # Edit the newest_commit annotation.
    if commits:
        new_newest_commit = commits[0].sha
        old_line = lines[prior_newest_commit_line]
        new_line = re.sub(r"[0-9a-f]{40}", new_newest_commit, old_line)
        assert new_line != old_line
        lines[prior_newest_commit_line] = new_line

    # Add each commit to a section, based on its package.
    for commit in commits:
        if len(commit.parents) > 1:
            logging.debug(f"Skipping merge commit {commit.sha}")
            continue
        # Try not to hit GitHub API rate limits.
        time.sleep(0.2)
        packages, bullet = _format_commit(gh, drake, commit)
        primary_package = packages[0]
        preamble = ""
        if len(packages) > 1:
            preamble = "[" + ",".join(packages) + "] "
        # Find the section for this commit, matching a line that looks like:
        # <relnotes for foo,{package},bar go here>
        found = False
        for i, one_line in enumerate(lines):
            match = re.search(r"<relnotes for (\S+) go here>", one_line)
            if match:
                (anchors_csv,) = match.groups()
                anchors = anchors_csv.split(",")
                if primary_package in anchors:
                    lines.insert(i + 1, f"{preamble}{bullet}\n")
                    found = True
                    break
        if not found:
            raise RuntimeError(f"Could not find anchor for {primary_package}")

    # Update the issue links.  Replace the text between these markers:
    # .. <begin issue links>
    # .. <end issue links>
    begin = lines.index(".. <begin issue links>\n")
    end = lines.index(".. <end issue links>\n")
    assert begin < end
    pr_numbers = set()
    for i, one_line in enumerate(lines):
        if begin < i < end:
            continue
        while True:
            match = re.search("`#([0-9]*)`_", one_line)
            if not match:
                break
            (number,) = match.groups()
            pr_numbers.add(number)
            one_line = one_line[match.end(0):]
    pr_links = [
        f".. _#{n}: https://github.com/RobotLocomotion/drake/pull/{n}\n"
        for n in sorted(list(pr_numbers))
    ]
    lines[begin + 1:end] = pr_links

    # Rewrite the notes file.
    with open(rst_filename + "~", "w") as f:
        for one_line in lines:
            f.write(one_line)
    os.replace(rst_filename + "~", rst_filename)


def _create(args, notes_dir, rst_filename, gh, drake):
    """The --create action."""

    if os.path.exists(rst_filename):
        raise RuntimeError(f"{rst_filename} already exists")

    # Find the commit sha for the prior_version release.
    prior_sha = next(drake.commits(sha=args.prior_version)).sha
    logging.debug(f"Prior release {args.prior_version} was {prior_sha}")

    # Fill in the notes template.
    with open(f"{notes_dir}/template.txt", "r") as f:
        template = f.read()
    content = template.format(
        version=args.version,
        prior_version=args.prior_version,
        oldest_commit_exclusive=prior_sha,
        newest_commit_inclusive=prior_sha,
    )
    # TODO(jwnimmer-tri) Strip out "This document is the template ..."
    # boilerplate before writing out the file.

    # Write the notes skeleton to disk.
    with open(rst_filename, "w") as f:
        f.write(content)


def main():
    # Parse the args.
    parser = argparse.ArgumentParser(
        prog="relnotes", description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument(
        "--action", type=str, choices=["create", "update"], required=True,
        help="Whether to create a new notes file or update an existing one")
    parser.add_argument(
        "--version", type=str, required=True,
        help="Notes file to create or edit, e.g., v0.22.0")
    parser.add_argument(
        "--prior_version", type=str,
        help="Prior revision (required iff creating new notes)")
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
    if args.version[0] != "v":
        parser.error("Versions are formatted like vM.m.p with the 'v'")
    if args.prior_version and args.prior_version[0] != "v":
        parser.error("Versions are formatted like vM.m.p with the 'v'")

    # Find the file to operate on.
    me = os.path.realpath(sys.argv[0])
    workspace = os.path.dirname(os.path.dirname(os.path.dirname(me)))
    notes_dir = f"{workspace}/doc/release_notes"
    if not os.path.isdir(notes_dir):
        parser.error("Could not find release_notes directory")
    rst_filename = f"{notes_dir}/{args.version}.rst"

    # Authenticate to GitHub.
    with open(os.path.expanduser(args.token_file), "r") as f:
        token = f.read().strip()
    gh = github3.login(token=token)
    drake = gh.repository("RobotLocomotion", "drake")

    # Perform the requested action.
    if args.action == "create":
        if not args.prior_version:
            parser.error("--prior_version is required to --create")
        _create(args, notes_dir, rst_filename, gh, drake)
    else:
        assert args.action == "update"
        _update(args, rst_filename, gh, drake)


if __name__ == '__main__':
    main()
