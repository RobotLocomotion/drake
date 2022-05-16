"""Reports summary data related to Drake's repository rules.  This
implementation uses Bazel command-line actions so is suitable only
for manual use, not any build rules or test automation.
"""

import glob
import json
import os
import subprocess


def _check_output(args):
    return subprocess.check_output(args).decode("utf8")


def read_repository_metadata(repositories=None):
    """If no repositories are given, returns data for all repositories.
    Returns a dict of {repository_name => details_dict}.
    """
    result = {}

    # Ask where the repository rules write their output.
    output_base = _check_output(
        ["bazel", "info", "output_base"]).strip()
    assert os.path.isdir(output_base), output_base

    if not repositories:
        # Obtain a list of known repositories.
        package_lines = _check_output(
            ["bazel", "query", "deps(//...)", "--output", "package"])
        repositories = set()
        for line in package_lines.split("\n"):
            if not line.startswith("@"):
                continue
            name = line[1:].split("/")[0]
            repositories.add(name)

        # These are starlark deps, so don't show up in the query.
        repositories.add("bazel_skylib")
        repositories.add("rules_pkg")

    # Make sure all of the repository_rule results are up-to-date.
    subprocess.check_call(["bazel", "fetch", "//..."])

    # Read the metadata.
    for name in sorted(repositories):
        json_path = os.path.join(
            output_base, "external", name, "drake_repository_metadata.json")
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
                result[data["name"]] = data
        except IOError:
            pass

    return result
