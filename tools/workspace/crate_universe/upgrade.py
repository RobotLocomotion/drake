"""Upgrade Drake's Rust dependencies and regenerate their repository names."""

import json
import os
from pathlib import Path
import subprocess
import tomllib

from python import runfiles


def main() -> None:
    # Operate relative to the root of the Drake source tree.
    drake_dir = Path(os.environ["BUILD_WORKSPACE_DIRECTORY"])
    os.chdir(drake_dir)
    my_dir = drake_dir / "tools/workspace/crate_universe"

    manifest = runfiles.Create()
    cargo_runfile = os.environ["DRAKE_CARGO_RLOCATIONPATH"]
    cargo = manifest.Rlocation(cargo_runfile)
    cargo_toml = manifest.Rlocation(
        os.environ["DRAKE_CARGO_MANIFEST_RLOCATIONPATH"]
    )
    workspace = Path(cargo_toml).parent

    subprocess.run(
        [
            cargo,
            "update",
            "--manifest-path",
            cargo_toml,
        ],
        cwd=workspace,
        check=True,
        env=(
            os.environ
            | manifest.EnvVars()
            | {"BUILD_WORKING_DIRECTORY": str(workspace)}
        ),
    )

    lockfile = my_dir / "lock/Cargo.lock"
    packages = tomllib.loads(lockfile.read_text(encoding="utf-8"))["package"]
    names = sorted(
        f"{package['name']}-{package['version']}".replace("+", "-")
        for package in packages
        if package.get("source")
    )
    (my_dir / "lock/repo_names.bzl").write_text(
        "REPO_NAMES = [\n"
        + "".join(f"    {json.dumps(name)},\n" for name in names)
        + "]\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
