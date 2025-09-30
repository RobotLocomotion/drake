"""Reports summary data related to Drake's repository rules.  This
implementation uses Bazel command-line actions so is suitable only
for manual use, not any build rules or test automation.
"""

import glob
import json
import logging
import os
import subprocess


_REPOSITORIES_WITH_NO_METADATA = [
    # Bazel modules (or module extensions thereof).
    "apple_support",
    "bazel_skylib",
    "bazel_tools",
    "buildifier_prebuilt",
    "gflags",
    "google_benchmark",
    "googletest",
    "gurobi",
    "lcm_maven",
    "llvm",
    "nasm",
    "platforms",
    "rules_cc",
    "rules_java",
    "rules_jvm_external",
    "rules_license",
    "rules_python",
    "rules_rust",
    "rules_shell",
    # Host libraries / binaries.
    "gfortran_internal",
    # Only ever upgraded by hand.
    "doxygen_internal",
    "snopt",
    "spgrid_internal",
]


def _expect_to_have_metadata(apparent_name):
    if apparent_name.startswith("crate_"):
        # Handled by the over-arching "crate_universe" upgrade.
        return False
    if apparent_name.startswith("module_"):
        # Bazel modules are handled by Renovate, not Drake tools.
        return False
    if apparent_name in _REPOSITORIES_WITH_NO_METADATA:
        return False
    return True


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
                # Not a repository.
                continue
            if line.startswith("@@"):
                # A repository that only has a canonical name, not an apparent
                # name. That means we don't load it in MODULE.bzl, so we don't
                # care about its metadata.
                continue
            apparent_name = line[1:].split("/")[0]
            repositories.add(apparent_name)

    # Make sure all of the repository_rule results are up-to-date.
    subprocess.check_call(["bazel", "fetch", "//..."])

    # Read the metadata.
    for apparent_name in sorted(repositories):
        found = False
        for canonical_name in [
                f"+drake_dep_repositories+{apparent_name}",
                f"+internal_repositories+{apparent_name}"]:
            json_path = os.path.join(
                output_base, "external", canonical_name,
                "drake_repository_metadata.json")
            try:
                with open(json_path, "r") as f:
                    data = json.load(f)
                    result[data["name"]] = data
                found = True
            except IOError:
                pass
            if found:
                break

        # Yell when something went wrong.
        expect_to_find = _expect_to_have_metadata(apparent_name)
        if expect_to_find and not found:
            logging.warn(f"Missing metadata for {apparent_name}")
        elif not expect_to_find and found:
            logging.warn(f"Unexpectedly found metadata for {apparent_name}")

    # Add 'magic' metadata for repositories that don't/can't generate it the
    # usual way.
    result["crate_universe"] = {
        "repository_rule_type": "scripted",
        "upgrade_script": "upgrade.sh",
        # Downloads are associated with individual "crate__..." repositories.
        "downloads": {},
    }

    return result
