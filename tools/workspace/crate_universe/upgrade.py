#!/usr/bin/env python3

"""Upgrade Drake's Rust dependencies and regenerate their repository names."""

# noqa: shebang
# We suppress shebang lint checking because we have a magic trampoline atop our
# main function that allows us to re-execute ourselves using Bazel.

import json
import os
from pathlib import Path
import subprocess
import tomllib


def main() -> None:
    cargo_runfile = os.environ.get("DRAKE_CARGO_RLOCATIONPATH")
    if cargo_runfile is None:
        os.chdir(Path(__file__).resolve().parents[3])
        os.execvp(
            "bazel",
            ["bazel", "run", "//tools/workspace/crate_universe:upgrade"],
        )

    from python import runfiles

    manifest = runfiles.Create()
    cargo = manifest.Rlocation(cargo_runfile)
    script_dir = (
        Path(os.environ["BUILD_WORKSPACE_DIRECTORY"])
        / "tools/workspace/crate_universe"
    )
    subprocess.run(
        [
            cargo,
            "update",
            "--manifest-path",
            "lock/Cargo.toml",
        ],
        cwd=script_dir,
        check=True,
        env=os.environ
        | manifest.EnvVars()
        | {"BUILD_WORKING_DIRECTORY": str(script_dir)},
    )

    lockfile = script_dir / "lock/Cargo.lock"
    packages = tomllib.loads(lockfile.read_text(encoding="utf-8"))["package"]
    names = sorted(
        f"crate__{package['name']}-{package['version']}".replace("+", "-")
        for package in packages
        if package.get("source")
    )
    (script_dir / "lock/repo_names.bzl").write_text(
        "REPO_NAMES = [\n"
        + "".join(f"    {json.dumps(name)},\n" for name in names)
        + "]\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
