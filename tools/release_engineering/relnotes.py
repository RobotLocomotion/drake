r"""Tool to help populate doc/release_notes/*.rst entries by automatically
adding commit messages' content into a structured document template.

This program is intended only for use by Drake maintainers who are preparing
Drake's release notes documentation.

This program is supported only on Ubuntu (not macOS).

The usage of this tool is outlined in the Drake release playbook
document:

  https://drake.mit.edu/release_playbook.html

The tool reads the commit history of github.com/RobotLocomotion/drake using
the GitHub APIs -- the status of your current git clone is ignored.

Use bazel to build the executable relnotes tool:

  bazel build //tools/release_engineering:relnotes   # build
  bazel-bin/tools/release_engineering/relnotes       # run

To query GitHub APIs, you'll need to authenticate yourself first,
via a GitHub API token.

To create the required ~/.config/readonly_github_api_token.txt file, open a
browser to https://github.com/settings/tokens and create a new token (it does
not need any extra permissions; the default "no checkboxes are set" is good),
and save the plaintext hexadecimal token to that file.

Here's an example of how to create (and then update) a new release notes
document:

  bazel build //tools/release_engineering:relnotes
  bazel-bin/tools/release_engineering/relnotes --action=create \
    --version=v1.2.0 --prior_version=v1.1.0
  bazel-bin/tools/release_engineering/relnotes --action=update \
    --version=v1.2.0
"""

import argparse
from collections import Counter
import logging
import os.path
from pathlib import Path
import re
import sys
import time

import github3


def _format_inline_pr_link(pr_num):
    """Return an inline link to the PR `pr_num`.  The corresponding
    `_format_ref_pr_link` text must appear later in the file."""
    return f"[#{pr_num}][_#{pr_num}]"


def _format_ref_pr_link(pr_num):
    """Return a reference link to the PR `pr_num`.  This goes at the bottom of
    the file and allows earlier `_format_inline_pr_link` to work."""
    url = f"https://github.com/RobotLocomotion/drake/pull/{pr_num}"
    return f"[_#{pr_num}]: {url}"


def _filename_to_primary_package(filename):
    """Given a filename (e.g., "systems/framework/system.h"), return its
    primary package (e.g., "systems").
    """
    if filename.startswith("tools/workspace/"):
        return "workspace"
    if filename.startswith("geometry/optimization/"):
        return "solvers"
    segments = filename.split("/")
    if len(segments) > 1:
        return segments[0]
    else:
        # For "." just make something up.
        return "tools"


def _format_commit(gh, drake, commit):
    """Returns (packages, severities, bullet) for the given commit.

    The packages is a list of top-level directories whose files were edited in
    this commit. If the packages list is empty, then the commit is ineligible
    for release notes and should be dropped.

    The bullet is a "* Detail (#123)" summary of the change for release notes.

    Ineligible commits are those where:
    - all changes were to files in dev directories; or
    - the "release notes: none" label has been applied to the PR.
    """
    # Grab the commit message subject and body.
    message = commit.message
    lines = message.splitlines()
    subject = lines[0]

    # Now, we'll try to find the pull request number and GH API object.  If
    # we can't find the PR number, we'll leave the sha there for someone to
    # clean up later.
    pr_num = commit.sha
    pr_summary = subject
    pr_object = None

    # See if the subject contained the pull request number.
    match = re.match(r"^(.*) +\(#([\d]+)\)$", subject)
    if match:
        # Yes; just use it.
        pr_summary, pr_num = match.groups()
        pr_object = drake.pull_request(int(pr_num))
    else:
        # No number in subject, so we'll have to look for PRs that mention this
        # commit.  Usually there is just the one PR in the search result (the
        # one that merged it), but sometimes other PRs might mention it (e.g.,
        # for reverts).
        time.sleep(0.2)  # Try not to hit GitHub API rate limits.
        results = list(
            gh.search_issues(
                f"{commit.sha} is:pr is:merged repo:RobotLocomotion/drake",
                number=2,
            )
        )
        if len(results) == 1:
            pr_object = results[0]
            pr_num = str(pr_object.number)

    # Check if this commit is ineligible due to a label, or otherwise which
    # kind of category it should appear in.
    label_needle = "release notes: "
    severities = []
    if pr_object is not None:
        for label in pr_object.labels:
            if not label["name"].startswith(label_needle):
                continue
            severity = label["name"][len(label_needle) :]
            severities.append(severity)
    else:
        # If we failed to associate the commit with a PR, ask the user to
        # manually check the label.
        severities = ["check label"]
    if severities == ["none"]:
        return [], ["none"], ""

    # Figure out which top-level package(s) were changed, to help the release
    # notes author sort things better.
    comparison = drake.compare_commits(
        base=commit.parents[0].get("sha"), head=commit.sha
    )
    committed_files_weighted = {
        x.get("filename"): x.get("changes") for x in comparison.files
    }
    # If all files in the commit are in dev directories, return empty data to
    # indicate the commit is ineligible.
    committed_nondev_files = [
        x for x in committed_files_weighted.keys() if "/dev/" not in x
    ]
    if not committed_nondev_files:
        return [], ["none"], ""

    # Report packages in order of most lines changed.
    packages_weighted = sum(
        [
            Counter({_filename_to_primary_package(k): v})
            for k, v in committed_files_weighted.items()
        ],
        Counter(),
    )
    packages = [k for k, v in packages_weighted.most_common()] or ["tools"]
    if len(packages) > 1 and "bindings" in packages:
        packages.remove("bindings")

    # Remove subject line trailing period(s).
    nice_summary = pr_summary.strip().rstrip(".").strip()

    # If there is a commit message body, turn it into some end-of-line text
    # that we'll have to editorialize by hand. Also ignore "Co-Authored-By:
    # ..." (inserted by GitHub) as it's irrelevant.
    detail = " ".join(
        [
            x.strip()
            for x in lines[1:]
            if x and "co-authored-by:" not in x.lower()
        ]
    )
    # When squashing, reviewable repeats the subject in the body.
    redundant_detail = f"* {pr_summary}"
    if detail.startswith(redundant_detail):
        detail = detail[len(redundant_detail) :].strip()
    if detail:
        detail = f"  # {detail}"

    # Add a multi-packages hint, if we have too many.
    preamble = ""
    if len(packages) > 1:
        preamble = "[" + ",".join(packages) + "] "

    # Format as top-level bullet point.
    inline_link = _format_inline_pr_link(pr_num)
    bullet = f"* TBD {preamble}{nice_summary} ({inline_link}){detail}"
    return packages, severities, bullet


def _insert_into_section(lines, section_name, bullet):
    """Inserts the given bullet into a section indicated by `section_name`.

    The section is identified by a line that looks like:
      <relnotes for name1(,name2,name3) go here>.

    The bullet is inserted two lines after that line (i.e., after the section
    header assuming `section_name` is found in the name list. `section_name`
    should only appear once in lines.
    """
    found = False
    for i, one_line in enumerate(lines):
        match = re.search(r"<relnotes for (\S+) go here>", one_line)
        if match:
            (anchors_csv,) = match.groups()
            anchors = anchors_csv.split(",")
            if section_name in anchors:
                lines.insert(i + 2, f"{bullet}\n")
                found = True
                break
    if not found:
        raise RuntimeError(f"Could not find anchor for {section_name}")


def _update(args, notes_filename, gh, drake, target_commit):
    """The --update action."""

    # Read in the existing content.
    with notes_filename.open(encoding="utf-8") as f:
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
        raise RuntimeError("Could not find newest_commit inclusive")
    logging.debug(f"Prior newest_commit {prior_newest_commit}")

    # Find new commits from newest to oldest.
    commits = []
    for commit in drake.commits(sha="master"):
        if commit.sha == prior_newest_commit:
            break
        commits.append(commit)
        if len(commits) == args.max_num_commits:
            raise RuntimeError("Reached max_num_commits")

    if target_commit is not None:
        if target_commit == prior_newest_commit:
            commits = []
        else:
            commit_shas = [commit.sha for commit in commits]
            # Assert that we see target_commit along the mainline branch.
            if target_commit not in commit_shas:
                raise RuntimeError(
                    f"--target_commit={target_commit} is not part of "
                    f"commits from prior commit ({prior_newest_commit}) to "
                    f"latest commit on master. It is either too old (before "
                    f"prior commit) or not on the master branch."
                )
            # Trim commits down to target commit.
            target_index = commit_shas.index(target_commit)
            commits = commits[target_index:]

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
        packages, severities, bullet = _format_commit(gh, drake, commit)

        # Skip commits deemed ineligible.
        if not packages:
            assert severities == ["none"], severities
            continue

        # TODO(jwnimmer-tri) Eventually we should auto-sort these as well.
        # For the moment, we'll just paste in the text so the human doesn't
        # overlook any important labels.
        bullet = f"* [{' '.join(severities)}] {bullet[2:]}"

        primary_package = packages[0]

        # For each known severity tag, we insert the bullet into the document
        # at a particular section. The bullet can be filed in multiple
        # sections. Unknown tags are an error.
        insertion_points = {
            "fix": primary_package,
            "feature": primary_package,
            "breaking change": "breaking-changes",
            "newly deprecated": "newly-deprecated",
            "removal of deprecated": "deprecated-removed",
            "announce": "announcements",
        }
        primary_used = False
        for severity in severities:
            package_name = insertion_points.get(severity, "")
            if package_name == "":
                raise RuntimeError(
                    f"Unknown severity ('{severity}') prevents disposition:\n"
                    f"   '{bullet[2:]}'"
                )
            if package_name == primary_package:
                if primary_used:
                    # We only file once per primary package.
                    continue
                primary_used = True
            _insert_into_section(lines, package_name, bullet)

    # Update the issue links.  Replace the text between these markers:
    # .. <begin issue links>
    # .. <end issue links>
    begin = lines.index("<!-- <begin issue links> -->\n")
    end = lines.index("<!-- <end issue links> -->\n")
    assert begin < end
    pr_numbers = set()
    for i, one_line in enumerate(lines):
        if begin < i < end:
            continue
        while True:
            match = re.search(r"\[_#([0-9]*)\]", one_line)
            if not match:
                break
            (number,) = match.groups()
            pr_numbers.add(number)
            one_line = one_line[match.end(0) :]
    pr_links = [_format_ref_pr_link(n) + "\n" for n in sorted(list(pr_numbers))]
    lines[begin + 1 : end] = pr_links

    # Rewrite the notes file.
    temp_notes = notes_filename.with_suffix(".md~")
    with open(temp_notes, "w") as f:
        for one_line in lines:
            f.write(one_line)
    os.replace(temp_notes, notes_filename)


def _create(args, notes_dir, notes_filename, gh, drake):
    """The --create action."""

    if notes_filename.exists():
        raise RuntimeError(f"{notes_filename} already exists")

    # Update the version numbers in from_binary.md.
    from_binary_path = notes_dir / "../_pages/from_binary.md"
    old_text = from_binary_path.read_text(encoding="utf-8")
    new_text = old_text.replace(args.prior_version[1:], args.version[1:])
    from_binary_path.write_text(new_text, encoding="utf-8")

    # Find the commit sha for the prior_version release.
    prior_sha = next(drake.commits(sha=args.prior_version)).sha
    logging.debug(f"Prior release {args.prior_version} was {prior_sha}")

    # Fill in the notes template.
    template = (notes_dir / "template.txt").read_text(encoding="utf-8")
    content = template.format(
        version=args.version[1:],
        prior_version=args.prior_version[1:],
        oldest_commit_exclusive=prior_sha,
        newest_commit_inclusive=prior_sha,
    )

    # Strip out "This document is the template ..." boilerplate, which is the
    # first line of the file.
    content = content[content.index("\n") + 1 :]

    # Write the notes skeleton to disk.
    notes_filename.write_text(content, encoding="utf-8")


def main():
    # Parse the args.
    parser = argparse.ArgumentParser(
        prog="relnotes",
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--action",
        type=str,
        choices=["create", "update"],
        required=True,
        help="Whether to create a new notes file or update an existing one",
    )
    parser.add_argument(
        "--version",
        type=str,
        required=True,
        help="Notes file to create or edit, e.g., v1.2.0",
    )
    parser.add_argument(
        "--prior_version",
        type=str,
        help="Prior revision (required iff creating new notes)",
    )
    parser.add_argument(
        "--max_num_commits",
        type=int,
        default=400,
        help="Stop after chasing this many commits",
    )
    parser.add_argument(
        "--target_commit",
        type=str,
        help="Use this as the target commit for --action=update. This *must* "
        "be newer than the prior commit, and must be fully a resolved "
        "40-character SHA1.",
    )
    parser.add_argument(
        "--token_file",
        default="~/.config/readonly_github_api_token.txt",
        help="Uses an API token read from this filename (default: %(default)s)",
    )
    parser.add_argument("--verbose", action="store_true", default=False)
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
    notes_dir = Path(f"{workspace}/doc/_release-notes")
    if not notes_dir.is_dir():
        parser.error("Could not find release_notes directory")
    notes_filename = notes_dir / f"{args.version}.md"

    # Authenticate to GitHub.
    with open(os.path.expanduser(args.token_file), "r") as f:
        token = f.read().strip()
    gh = github3.login(token=token)
    drake = gh.repository("RobotLocomotion", "drake")

    # Perform the requested action.
    if args.action == "create":
        if not args.prior_version:
            parser.error("--prior_version is required to --action=create")
        if args.target_commit is not None:
            parser.error(
                "--target_commit cannot be specified with --action=create"
            )
        _create(args, notes_dir, notes_filename, gh, drake)
    else:
        assert args.action == "update"
        if args.target_commit is not None:
            if not re.match(r"^[0-9a-f]{40}$", args.target_commit):
                parser.error(
                    f"--target_commit={args.target_commit} is not a "
                    f"40-character SHA1"
                )

        _update(args, notes_filename, gh, drake, args.target_commit)


if __name__ == "__main__":
    main()
