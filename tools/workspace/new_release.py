"""Reports on which of Drake's external dependencies can be updated to a more
recent version.  This is intended for use by Drake maintainers (only).

This program is only supported on Ubuntu Noble 24.04.

To query GitHub APIs, you'll need to authenticate yourself first.  There are
two ways to do this:

(1) Type in your password each time you run this program:

  bazel run //tools/workspace:new_release -- --use_password

(2) Use a GitHub API token:

  bazel run //tools/workspace:new_release

To create the ~/.config/readonly_github_api_token.txt file used by (2), open a
browser to https://github.com/settings/tokens and create a new token (it does
not need any extra permissions; the default "no checkboxes are set" is good),
and save the plaintext hexadecimal token to that file.

This program can also automatically prepare upgrades for our GitHub externals
by passing the name(s) of package(s) to upgrade as additional arguments:

  bazel run //tools/workspace:new_release -- --lint --commit rules_python

Note that this program runs `bazel` as a subprocess, without any special
command line flags.  If you do need to use any flags when you run bazel,
then those must be added to an rcfile; they cannot be provided on the
command line.
"""

import argparse
from dataclasses import dataclass
import getpass
import hashlib
import json
import logging
import os
import re
import shlex
import subprocess
from tempfile import TemporaryDirectory
import time
from typing import Optional, Set
import urllib

import git
import github3

from tools.workspace.metadata import read_repository_metadata

logger = logging.getLogger("new_release")
logger.setLevel(logging.INFO)

warn = logger.warning
info = logger.info

# Repository rules that fetch from GitHub.
_GITHUB_RULE_TYPES = ["github", "github_release_attachments"]

# Repository rule that uses an external upgrade script.
_SCRIPTED_RULE_TYPE = "scripted"

# We'll skip these repositories when making suggestions.
_IGNORED_REPOSITORIES = [
    "clang_cindex_python3_internal",  # Uses a non-default branch.
    "mosek",  # Requires special, non-automated care during upgrades.
    "pybind11",  # Uses a non-default branch.
]

# These repositories cannot be automatically upgraded, but can be automatically
# checked for possible upgrades. When checking for new releases, print a
# reminder to check for upgrades, but don't do anything.
_CHECK_ONLY_REPOSITORIES = [
    "doxygen_internal",
]

# These repositories cannot be automatically upgraded nor automatically checked
# for possible upgrades. When checking for new releases, always print a reminder
# to manually check for upgrades.
_OTHER_REPOSITORIES = [
    "python",
]

# For these repositories, ignore any tags that match the specified regex.
_IGNORED_TAGS = {
    "gymnasium_py_internal": r"v[0-9.]+a[0-9]+",
    "libpng_internal": r"v[0-9.]+(alpha|beta)[0-9]+",
    "msgpack_internal": r"c-[0-9.]+",
    "ros_xacro_internal": r"xacro-[0-9.]+",
    "sdformat_internal": r"sdformat-prerelease_[0-9.]+",
}

# For these repositories, we only look at tags, not releases.  For the dict
# value, use a blank value to match the latest tag or a regex to only select
# tags that share the match with the tag currently in use; the parentheses
# group in the regex denotes the portion of the tag to lock as invariant.
# (This can be used to pin to a given major or major.minor release series.)
_OVERLOOK_RELEASE_REPOSITORIES = {
    "doxygen_internal": "",
    "github3_py_internal": r"^(\d+.)",
    "gz_math_internal": r"^(gz)",
    "gz_utils_internal": r"^(gz)",
    "qhull_internal": r"^(2)",
    "sdformat_internal": "",
    "xmlrunner_py_internal": "",
}

# Packages in these cohorts should be upgraded together (in a single commit).
_COHORTS = (
    # clarabel_cpp uses crate_universe; be sure to keep them aligned.
    {"clarabel_cpp_internal", "crate_universe"},
    # mypy uses mypy_extensions; be sure to keep them aligned.
    {"mypy_internal", "mypy_extensions_internal"},
    # sdformat depends on both gz libraries; be sure to keep them aligned.
    {"sdformat_internal", "gz_math_internal", "gz_utils_internal"},
    # uwebsockets depends on usockets; be sure to keep them aligned.
    {"uwebsockets_internal", "usockets_internal"},
)


@dataclass
class UpgradeResult:
    was_upgraded: bool
    can_be_committed: bool = False
    modified_paths: Optional[Set[str]] = None
    commit_message: Optional[str] = None


def _str_replace_forced(original, old, new):
    if old == new:
        return original
    result = original.replace(old, new)
    if result == original:
        raise RuntimeError(f"Could not find '{old}' to substitute")
    return result


def _rewrite_file_contents(path, new_content):
    """Atomically replace the contents of path with new_content."""
    with open(f"{path}.new", "w", encoding="utf-8") as f:
        f.write(new_content)
    os.rename(f"{path}.new", path)


def _check_output(args):
    return subprocess.check_output(args).decode("utf8")


def _get_default_username():
    origin_url = _check_output(
        ["git", "config", "--get", "remote.origin.url"]
    ).strip()
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


def _is_ignored_tag(commit, workspace):
    """Returns true iff commit matches an ignore rule or seems to be a
    pre-release.
    """
    ignore_re = _IGNORED_TAGS.get(workspace)
    if ignore_re and re.match(ignore_re, commit):
        # Matches the regex of tag names to definitely ignore; do so quietly so
        # we don't spam the user.
        return True

    development_stages = ["alpha", "beta", "pre", "rc", "unstable"]
    prerelease = any(stage in commit for stage in development_stages)
    if prerelease:
        # Heuristically looks like a pre-release; ignore it, but log it for the
        # user to check.
        warn(f"Skipping prerelease {commit} for {workspace}")
    return prerelease


def _latest_tag(gh_repo, workspace):
    for tag in gh_repo.tags():
        if _is_ignored_tag(tag.name, workspace):
            continue
        return tag.name
    warn(f"Could not find any matching tags for {workspace}")
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
                    if _is_ignored_tag(tag.name, workspace_name):
                        continue
                    new_commit = tag.name
                    break
        return old_commit, new_commit

    # By default, use the latest release if there is one.  Otherwise, use the
    # latest tag.
    try:
        new_commit = gh_repo.latest_release().tag_name
        if _is_ignored_tag(new_commit, workspace_name):
            new_commit = _latest_tag(gh_repo, workspace_name)
    except github3.exceptions.NotFoundError:
        new_commit = _latest_tag(gh_repo, workspace_name)
    return old_commit, new_commit


def _check_for_upgrades(gh, args, metadata):
    for workspace_name, data in sorted(metadata.items()):
        if workspace_name in _IGNORED_REPOSITORIES:
            continue
        if data.get("version_pin"):
            continue
        rule_type = data["repository_rule_type"]
        if rule_type in _GITHUB_RULE_TYPES:
            old_commit, new_commit = _handle_github(workspace_name, gh, data)
        elif rule_type == _SCRIPTED_RULE_TYPE:
            info(f"{workspace_name} may need upgrade")
            continue
        else:
            raise RuntimeError(f"Bad rule type {rule_type} in {workspace_name}")
        if old_commit == new_commit:
            continue
        elif new_commit is not None:
            info_message = (
                f"{workspace_name} needs upgrade"
                f" from {old_commit} to {new_commit}"
            )
            if workspace_name in _CHECK_ONLY_REPOSITORIES:
                info_message += ", but cannot be auto-upgraded"
            info(info_message)
        else:
            warn(
                f"{workspace_name} version {old_commit} needs manual inspection"
            )


def _modified_paths(repo, root):
    """Returns the set of paths under `root` which are added, removed or
    altered.
    """
    assert os.path.isdir(os.path.join(repo.working_tree_dir, root))
    if not root.endswith("/"):
        root += "/"

    result = set()
    for item in repo.untracked_files:
        if item.startswith(root):
            result.add(item)

    for other in [None, "HEAD"]:
        for item in repo.index.diff(other):
            if item.a_path.startswith(root):
                result.add(item.a_path)
            if item.b_path.startswith(root):
                result.add(item.b_path)

    return result


def _is_unmodified(repo, path):
    """Returns true iff the given `path` is unmodified in the working tree of
    the given `git.Repo`, `repo`. If repo is None, returns False.
    """
    if repo is None:
        return False

    if os.path.isdir(os.path.join(repo.working_tree_dir, path)):
        return len(_modified_paths(repo, path)) == 0

    else:
        for other in [None, "HEAD"]:
            if path in [item.b_path for item in repo.index.diff(other)]:
                return False

    return True


def _do_commit(
    local_drake_checkout, actually_commit, workspace_names, paths, message
):
    if actually_commit:
        names = ", ".join(workspace_names)
        local_drake_checkout.git.add("-A", *paths)
        local_drake_checkout.git.commit(
            "-o", *paths, "-m", "[workspace] " + message
        )
        info("")
        info("*" * 72)
        info(f"Done.  Changes for {names} were committed.")
        info("Be sure to review the changes and amend the commit if needed.")
        info("*" * 72)
        info("")
    else:
        info("")
        info("*" * 72)
        info("Done.  Be sure to review and commit the changes:")
        info(f"  git add {' '.join([shlex.quote(p) for p in paths])}")
        info(f"  git commit -m{shlex.quote('[workspace] ' + message)}")
        info("*" * 72)
        info("")


def _download(url, local_filename):
    """Given a url, downloads it to the local_filename (overwriting anything
    that was there previously). Returns the sha256 checksum.
    """
    hasher = hashlib.sha256()
    with open(local_filename, "wb") as f:
        with urllib.request.urlopen(url) as response:
            while True:
                data = response.read(4096)
                if not data:
                    break
                hasher.update(data)
                f.write(data)
    return hasher.hexdigest()


def _do_upgrade_github_archive(
    *, temp_dir, old_commit, new_commit, bzl_filename, repository
):
    # Slurp the file we're supposed to modify.
    with open(bzl_filename, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # Locate the two hexadecimal lines we need to edit.
    commit_line_re = re.compile(
        r'(?<=    commit = ")(' + re.escape(old_commit) + r')(?=",)'
    )
    checksum_line_re = re.compile(r'(?<=    sha256 = ")([0-9a-f]{64})(?=",)')
    commit_line_num = None
    checksum_line_num = None
    for i, line in enumerate(lines):
        match = commit_line_re.search(line)
        if match:
            assert commit_line_num is None
            commit_line_num = i
        match = checksum_line_re.search(line)
        if match:
            assert checksum_line_num is None
            checksum_line_num = i
    assert commit_line_num is not None
    assert checksum_line_num is not None

    # Download the new source archive.
    info("Downloading new archive...")
    if _smells_like_a_git_commit(new_commit):
        new_url = f"https://github.com/{repository}/archive/{new_commit}.tar.gz"
    else:
        new_url = f"https://github.com/{repository}/archive/refs/tags/{new_commit}.tar.gz"  # noqa
    new_filename = new_commit.replace("/", "_")
    new_checksum = _download(new_url, f"{temp_dir}/{new_filename}.tar.gz")

    # Update the repository.bzl contents and then write it out.
    lines[commit_line_num] = commit_line_re.sub(
        new_commit, lines[commit_line_num]
    )
    lines[checksum_line_num] = checksum_line_re.sub(
        new_checksum, lines[checksum_line_num]
    )
    _rewrite_file_contents(bzl_filename, "".join(lines))


def _do_upgrade_github_release_attachments(
    *,
    temp_dir,
    old_commit,
    new_commit,
    bzl_filename,
    repository,
    old_attachments,
):
    # Slurp the file we're supposed to modify.
    with open(bzl_filename, "r", encoding="utf-8") as f:
        bzl_content = f.read()

    # Download the new attachments.
    info("Downloading new attachments...")
    new_attachments = {}
    for filename in old_attachments.keys():
        new_url = (
            f"https://github.com/{repository}/"
            f"releases/download/{new_commit}/{filename}"
        )
        new_checksum = _download(new_url, f"{temp_dir}/{filename}")
        new_attachments[filename] = new_checksum

    # Update the repository.bzl contents and then write it out.
    bzl_content = _str_replace_forced(
        bzl_content, f'commit = "{old_commit}"', f'commit = "{new_commit}"'
    )
    for filename, old_checksum in old_attachments.items():
        new_checksum = new_attachments[filename]
        bzl_content = _str_replace_forced(
            bzl_content, f'"{old_checksum}"', f'"{new_checksum}"'
        )
    _rewrite_file_contents(bzl_filename, bzl_content)


def _do_upgrade_scripted(
    *, temp_dir, local_drake_checkout, workspace_root, script
):
    # Run the upgrade script.
    repo_root = local_drake_checkout.working_tree_dir
    subprocess.check_call([os.path.join(repo_root, workspace_root, script)])

    # Look for modified paths.
    return _modified_paths(local_drake_checkout, workspace_root)


def _do_upgrade(temp_dir, gh, local_drake_checkout, workspace_name, metadata):
    """Returns an `UpgradeResult` describing what (if anything) was done."""
    if workspace_name not in metadata:
        raise RuntimeError(f"Unknown repository {workspace_name}")

    data = metadata[workspace_name]
    rule_type = data["repository_rule_type"]
    bzl_filename = f"tools/workspace/{workspace_name}/repository.bzl"

    if workspace_name in _OTHER_REPOSITORIES + _CHECK_ONLY_REPOSITORIES:
        upgrade_advice = data.get("upgrade_advice", "")
        error_message = f"Cannot auto-upgrade {workspace_name}"
        if len(upgrade_advice):
            error_message += "\n".join(
                [
                    "",
                    "*" * 72,
                    upgrade_advice,
                    "*" * 72,
                ]
            )
        raise RuntimeError(error_message)

    if rule_type == _SCRIPTED_RULE_TYPE:
        # Determine if we should and can commit the changes made.
        workspace_root = f"tools/workspace/{workspace_name}/"
        can_commit = _is_unmodified(local_drake_checkout, workspace_root)
        if local_drake_checkout and not can_commit:
            warn(f"{workspace_root} has local changes.")
            warn(f"Changes made for {workspace_name} will NOT be committed.")

        # Do the upgrade.
        new_commit = None
        modified_paths = _do_upgrade_scripted(
            temp_dir=temp_dir,
            local_drake_checkout=local_drake_checkout,
            workspace_root=workspace_root,
            script=data["upgrade_script"],
        )
        if not len(modified_paths):
            return UpgradeResult(False)

    else:
        if rule_type not in _GITHUB_RULE_TYPES:
            raise RuntimeError(f"Cannot auto-upgrade {workspace_name}")

        # Sanity check that an upgrade is possible.
        old_commit, new_commit = _handle_github(workspace_name, gh, data)
        if old_commit == new_commit:
            return UpgradeResult(False)
        elif new_commit is None:
            raise RuntimeError(f"Cannot auto-upgrade {workspace_name}")
        info(f"Upgrading {workspace_name} from {old_commit} to {new_commit}")

        # Determine if we should and can commit the changes made.
        can_commit = _is_unmodified(local_drake_checkout, bzl_filename)
        if local_drake_checkout and not can_commit:
            warn(f"{bzl_filename} has local changes.")
            warn(f"Changes made for {workspace_name} will NOT be committed.")

        # Do the upgrade.
        if rule_type == "github":
            _do_upgrade_github_archive(
                temp_dir=temp_dir,
                old_commit=old_commit,
                new_commit=new_commit,
                bzl_filename=bzl_filename,
                repository=data["repository"],
            )
        else:
            assert rule_type == "github_release_attachments"
            _do_upgrade_github_release_attachments(
                temp_dir=temp_dir,
                old_commit=old_commit,
                new_commit=new_commit,
                bzl_filename=bzl_filename,
                repository=data["repository"],
                old_attachments=data["attachments"],
            )

        modified_paths = {bzl_filename}

    # Copy the downloaded tarball into the repository cache.
    info("Populating repository cache ...")
    subprocess.check_call(["bazel", "fetch", "//...", f"--distdir={temp_dir}"])

    # Check for additional instructions.
    upgrade_advice = data.get("upgrade_advice", "")
    if len(upgrade_advice):
        warn("")
        warn("*" * 72)
        warn(upgrade_advice)
        warn("*" * 72)
        warn("")

    # Finalize the result.
    if new_commit:
        if _smells_like_a_git_commit(new_commit):
            message = f"Update dependency {workspace_name} to latest commit"
        else:
            release = new_commit.lstrip("v")
            message = f"Update dependency {workspace_name} to {release}"
    else:
        # This is a scripted upgrade of multiple packages, so we'll omit
        # the word "dependency" from the commit message.
        message = f"Update {workspace_name} to latest"

    return UpgradeResult(True, can_commit, modified_paths, message)


def _do_upgrades(temp_dir, gh, local_drake_checkout, workspace_names, metadata):
    # Make sure there are workspaces to update.
    if len(workspace_names) == 0:
        return

    can_commit = True
    modified_paths = []
    commit_messages = []
    modified_workspace_names = []
    for workspace_name in workspace_names:
        result = _do_upgrade(
            temp_dir, gh, local_drake_checkout, workspace_name, metadata
        )
        if result.was_upgraded:
            can_commit = can_commit and result.can_be_committed
            modified_paths += result.modified_paths
            commit_messages.append(result.commit_message)
            modified_workspace_names.append(workspace_name)
        else:
            info(f"No updates for {workspace_name}")

    if not modified_workspace_names:
        # Nothing was updated
        names = ", ".join(workspace_names)
        info("")
        info("*" * 72)
        info(f"Done. No updates for {names}")
        info("*" * 72)
        info("")
        return

    # Determine if we should and can commit the changes made.
    if len(modified_workspace_names) == 1:
        _do_commit(
            local_drake_checkout,
            actually_commit=can_commit,
            workspace_names=modified_workspace_names,
            paths=modified_paths,
            message=commit_messages[0],
        )
    else:
        cohort = ", ".join(modified_workspace_names)

        if not can_commit:
            warn(f"Changes made for {cohort} will NOT be committed.")

        message = f"Upgrade {cohort} to latest\n\n"
        message += "- " + "\n- ".join(commit_messages)
        _do_commit(
            local_drake_checkout,
            actually_commit=can_commit,
            workspace_names=modified_workspace_names,
            paths=modified_paths,
            message=message,
        )


def main():
    parser = argparse.ArgumentParser(
        prog="new_release",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--commit",
        action="store_true",
        default=False,
        help="When upgrading repositories, automatically commit the changes.",
    )
    parser.add_argument(
        "--lint",
        action="store_true",
        default=False,
        help="Also run some sanity tests on the repository, after all other"
        " operations have completed successfully.",
    )
    parser.add_argument(
        "--use_password",
        action="store_true",
        default=False,
        help="Prompt for the GitHub password, instead of using an API token.",
    )
    parser.add_argument(
        "--token_file",
        default="~/.config/readonly_github_api_token.txt",
        help="Uses an API token read from this filename, unless "
        "--use_password was given (default: %(default)s)",
    )
    parser.add_argument(
        "--user",
        metavar="USER",
        type=str,
        default=_get_default_username(),
        help="GitHub username (default: %(default)s)",
    )
    parser.add_argument("--verbose", action="store_true", default=False)
    parser.add_argument(
        "workspace",
        nargs="*",
        metavar="WORKSPACES_NAME",
        type=str,
        help="(Optional) Instead of reporting on possible upgrades,"
        " download new archives for the given externals"
        " and edit their bzl rules to match.",
    )
    args = parser.parse_args()

    if "BUILD_WORKSPACE_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])

    if not os.path.exists("WORKSPACE"):
        parser.error(
            "Couldn't find WORKSPACE; this script must be run"
            " from the root of your Drake checkout."
        )

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(format="%(message)s")

    if args.use_password and not args.user:
        parser.error("Couldn't guess github username; you must supply --user.")

    # Log in to github.
    if args.use_password:
        prompt = f"Password for https://{args.user}@github.com: "
        gh = github3.login(username=args.user, password=getpass.getpass(prompt))
    else:
        with open(os.path.expanduser(args.token_file), "r") as f:
            token = f.read().strip()
        gh = github3.login(token=token)

    # Are we operating on all repositories, or just one?
    if len(args.workspace):
        workspaces = set(args.workspace)

        # Grow the set of specified repositories to cover cohorts.
        for workspace in args.workspace:
            for cohort in _COHORTS:
                if workspace in cohort:
                    workspaces.update(cohort)

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
    info("Collecting bazel repository details...")
    metadata = read_repository_metadata()
    if args.verbose:
        metadata_json = os.path.join(
            os.path.realpath("."),
            "debug_repository_metadata.json",
        )
        with open(metadata_json, "w", encoding="utf-8") as f:
            logging.debug(f"Writing repository metadata to '{metadata_json}'.")
            json.dump(metadata, f, sort_keys=True, indent=2)

    if workspaces is not None:
        visited_workspaces = set()
        for workspace in workspaces:
            # If we already did this as part of a tandem upgrade, skip it.
            if workspace in visited_workspaces:
                continue

            # Determine if this workspace is part of a cohort.
            cohort_workspaces = {workspace}
            for cohort in _COHORTS:
                if workspace in cohort:
                    cohort_workspaces = cohort

            # Actually do the upgrade(s).
            with TemporaryDirectory(prefix="drake_new_release_") as temp_dir:
                _do_upgrades(
                    temp_dir,
                    gh,
                    local_drake_checkout,
                    cohort_workspaces,
                    metadata,
                )
                visited_workspaces.update(cohort_workspaces)
    else:
        # Run our report of what's available.
        info("Checking for new releases...")
        _check_for_upgrades(gh, args, metadata)

        for repo in _OTHER_REPOSITORIES:
            info(f"{repo} may need upgrade but cannot be auto-upgraded.")

    if args.lint:
        subprocess.check_call(["bazel", "test", "--config=lint", "//..."])


if __name__ == "__main__":
    main()
