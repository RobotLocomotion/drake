"""
Push Docker release tags.

This is intended for use by Drake maintainers (only).
This program is only supported on Ubuntu Noble 24.04.
"""

import argparse
import json
import re
import shutil
import subprocess
import sys
import textwrap
from typing import Any, Dict, List
import urllib.request

import docker

_DOCKER_REGISTRY_API_URI = "https://registry.hub.docker.com/v2/repositories"
_DOCKER_REPOSITORY_NAME = "robotlocomotion/drake"


class _State:
    def __init__(self, options: Dict[str, Any]):
        self._options = options
        self._docker = docker.APIClient()

        self.source_version = options.source_version

    def _begin(self, action: str, src: str, dst: str = None) -> None:
        """
        Report the start of an action.
        """
        if dst is not None:
            print(f"{action} {src!r} to {dst!r} ...", flush=True, end="")
        else:
            print(f"{action} {src!r} ...", flush=True, end="")

    def _done(self) -> None:
        """
        Report the completion of an action.
        """
        print(" done")

    def push_docker_tag(
        self,
        old_tag_name: str,
        new_tag_name: str,
        repository: str = _DOCKER_REPOSITORY_NAME,
    ) -> None:
        image = f"{repository}:{old_tag_name}"
        if self._options.dry_run:
            print(f"push {image!r} to {repository!r} as {new_tag_name!r}")
        else:
            self._begin("pulling", f"{repository}:{old_tag_name}")
            self._docker.pull(repository, old_tag_name)
            self._done()

            self._docker.tag(image, repository, new_tag_name)

            self._begin("pushing", f"{repository}:{new_tag_name}")
            self._docker.push(repository, new_tag_name)
            self._done()

            self._docker.remove_image(image)


def _fatal(msg: str, result: int = 1) -> None:
    width = shutil.get_terminal_size().columns
    for line in msg.split("\n"):
        print(textwrap.fill(line, width), file=sys.stderr)
    sys.exit(result)


def _assert_tty() -> None:
    try:
        subprocess.check_call(["tty", "-s"])
    except subprocess.CalledProcessError:
        _fatal(
            "ERROR: tty was NOT detected. This script may need"
            " various login credentials to be entered interactively."
        )
        sys.exit(1)


def _check_version(version) -> bool:
    """
    Returns True iff the given version string matches PEP 440.
    """
    return (
        re.match(
            r"^([1-9][0-9]*!)?(0|[1-9][0-9]*)"
            r"(\.(0|[1-9][0-9]*))*((a|b|rc)(0|[1-9][0-9]*))?"
            r"(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?"
            r"([+][a-z0-9]+([-_\.][a-z0-9]+)*)?$",
            version,
        )
        is not None
    )


def _list_docker_tags(repository=_DOCKER_REPOSITORY_NAME) -> List[str]:
    tags = []
    uri = f"{_DOCKER_REGISTRY_API_URI}/{repository}/tags?page_size=1000"

    while uri is not None:
        with urllib.request.urlopen(uri) as response:
            reply = json.load(response)

        for t in reply["results"]:
            tags.append(t["name"])

        uri = reply.get("next")

    return tags


def _push_docker(state: _State) -> None:
    """
    Re-tags Docker staging images as release images.
    """
    tail = f"{state.source_version}-staging"
    for tag_name in _list_docker_tags():
        if tag_name.endswith(tail):
            release_tag_name = tag_name.rsplit("-", 1)[0]
            state.push_docker_tag(tag_name, release_tag_name)


def main(args: List[str]) -> None:
    parser = argparse.ArgumentParser(prog="push_docker", description=__doc__)
    parser.add_argument(
        "-n",
        "--dry-run",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="Print what would be done without actually pushing anything.",
    )
    parser.add_argument(
        "source_version",
        help="Version tag (x.y.z) of the release to be pushed.",
    )
    options = parser.parse_args(args)

    # Validate version arguments
    if not _check_version(options.source_version):
        _fatal(
            f"ERROR: source_version {options.source_version!r}"
            " is not a valid version number."
        )

    # Ensure execution environment is suitable.
    _assert_tty()

    # Set up shared state
    state = _State(options)

    # Push the requested release artifacts.
    _push_docker(state)


if __name__ == "__main__":
    main(sys.argv[1:])
