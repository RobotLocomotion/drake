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
    # Bazel modules.
    "eigen",
    "fmt",
    "spdlog",
    "module_eigen",
    "module_fmt",
    "module_spdlog",
    "pkgconfig_eigen",
    "pkgconfig_fmt",
    "pkgconfig_spdlog",
    "platforms",
    "rules_cc",
    "rules_java",
    "rules_license",
    "rules_python",
    "rules_rust",
    "rules_shell",
    # Host libraries / binaries.
    "gfortran",
    "glib",
    "glx",
    "gurobi",
    "lapack",
    "libblas",
    "liblapack",
    "mumps_internal",
    "nasm",
    "net_sf_jchart2d",
    "opencl",
    "opengl",
    "org_apache_xmlgraphics_commons",
    "x11",
    "zlib",
    # Vendored.
    "doxygen",
    "snopt",
    "spgrid_internal",
]


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

        # The bazel query only finds build-time dependencies.  Drake also
        # requires some load-time dependencies such as starlark libraries,
        # compilers, etc.  Here, we add by hand those we want to be archived
        # and upgraded.
        #
        # NOTE: At this time, we are skipping the rust_toolchain repositories;
        # see TODO in tools/workspace/rust_toolchain/repository.bzl.
        repositories.add("bazel_skylib")
        repositories.add("com_github_nelhage_rules_boost_internal")

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
            except IOError:
                pass
            if found:
                break

    # Yell when something went wrong.
    if not found and apparent_name not in _REPOSITORIES_WITH_NO_METADATA:
        logging.warn(f"Missing metadata for {apparent_name}")
    elif found and apparent_name in _REPOSITORIES_WITH_NO_METADATA:
        logging.warn(f"Unexpectedly found metadata for {name}")

    # Add 'magic' metadata for repositories that don't/can't generate it the
    # usual way.
    result["crate_universe"] = {
        "repository_rule_type": "scripted",
        "upgrade_script": "upgrade.sh",
        # Downloads are associated with individual "crate__..." repositories.
        "downloads": {},
    }
    result["rust_toolchain"] = {
        "repository_rule_type": "scripted",
        "upgrade_script": "upgrade.py",
        # Downloads are associated with individual "rust_..." repositories.
        "downloads": {},
    }

    return result
