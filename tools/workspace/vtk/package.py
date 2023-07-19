"""Build and package the .tar.gz archives for VTK."""
from __future__ import annotations

import argparse
import atexit
import os
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

# NOTE: on macOS we need these path modifications.  For mypy checks, we add the
# image directory rather than do a `from image.<module>` import for no other
# reason than mypy gets confused linting the whole directory if there is an
# `from vtk_common import ...` and `from image.vtk_common import ...`.
sys.path.insert(0, str(Path(__file__).parent.resolve() / "image"))
from clone_vtk import clone_vtk  # noqa: E402

from vtk_cmake_configure_args import vtk_cmake_configure_args  # noqa: E402

from vtk_common import (  # noqa: E402
    _rlocation,
    architecture,
    build_vtk,
    package_vtk,
    system_is_linux,
    system_is_macos,
    vtk_git_ref,
    vtk_package_tree,
)


@dataclass
class DockerPlatform:
    """Map human friendly names to docker base image tags to build with."""

    codename: str
    """The common name, e.g., ``jammy``."""

    docker_tag: str
    """The ubuntu docker tag to use, e.g., ``ubuntu:22.04``."""


# Artifacts that need to be cleaned up. DO NOT MODIFY outside of this file.
_images_to_remove: list[str] = []
_paths_to_remove: list[Path] = []


@atexit.register
def _cleanup():
    """Remove any artifacts (docker images, macOS build trees) on exit."""
    if len(_images_to_remove):
        subprocess.check_call(
            [
                "docker",
                "image",
                "rm",
                *_images_to_remove,
            ],
        )
    if len(_paths_to_remove):
        could_not_remove = []
        for p in _paths_to_remove:
            if p.is_dir():
                shutil.rmtree(p)
            elif p.is_file() or p.is_symlink():
                p.unlink()
            if p.exists():
                could_not_remove.append(p)

        if could_not_remove:
            raise RuntimeError(
                f"Unable to remove the path(s): {could_not_remove}"
            )


def build_linux(output_dir: Path, platforms: list[DockerPlatform], keep: bool):
    dockerfile_path = _rlocation("Dockerfile")
    tools_workspace_vtk_dir = dockerfile_path.parent.resolve()
    env = os.environ.copy()
    env["DOCKER_BUILDKIT"] = "1"
    for docker_platform in platforms:
        # Generate a unique identifier for this build.
        salt = os.urandom(8).hex()
        time = datetime.now(timezone.utc).strftime("%Y%m%d%H%M%S")
        tag = f"vtk:{time}-{salt}-{docker_platform.codename}"

        # Build and tag the vtk image.
        print(f"=> Building: {tag}")
        subprocess.check_call(
            [
                "docker",
                "build",
                "--force-rm",
                "--tag",
                tag,
                "--build-arg",
                f"PLATFORM={docker_platform.docker_tag}",
                str(tools_workspace_vtk_dir),
            ],
            env=env,
        )

        # Only add after it has successfully built.
        if not keep:
            _images_to_remove.append(tag)

        # Extract the .tar.gz archive.
        extract_container_name = f"{tag}.extract".replace(":", ".")
        package_tree = vtk_package_tree()
        ls_proc = subprocess.run(
            [
                "docker",
                "run",
                f"--name={extract_container_name}",
                tag,
                "bash",
                "-c",
                f"ls {package_tree.root}/vtk-*.tar.gz{{,.sha256}}",
            ],
            env=env,
            stdout=subprocess.PIPE,
        )

        try:
            vtk_paths = ls_proc.stdout.decode("utf-8").splitlines()
            # There should be 1 vtk-*.tar.gz, and 1 vtk-*.tar.gz.sha256.
            assert len(vtk_paths) == 2, f"Expected two files in {vtk_paths}."
            for container_path in vtk_paths:
                basename = Path(container_path).name
                subprocess.check_call(
                    [
                        "docker",
                        "cp",
                        f"{extract_container_name}:{container_path}",
                        str(output_dir / basename),
                    ]
                )
        finally:
            subprocess.check_call(["docker", "rm", extract_container_name])


def build_macos(output_dir: Path, keep: bool):
    # Gather the explicit information needed for directly configuring the CMake
    # C and C++ compilers, and configuring the Xcode toolchain.
    xcrun_proc = subprocess.run(
        ["xcrun", "-sdk", "macosx", "-show-sdk-path"],
        check=True,
        stdout=subprocess.PIPE,
    )
    sysroot = xcrun_proc.stdout.decode("utf-8").strip()

    xcrun_proc = subprocess.run(
        [
            "xcrun",
            "-sdk",
            "macosx",
            "-find",
            "gcc",
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    xcode_c_compiler = xcrun_proc.stdout.decode("utf-8").strip()

    xcrun_proc = subprocess.run(
        [
            "xcrun",
            "-sdk",
            "macosx",
            "-find",
            "g++",
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    xcode_cxx_compiler = xcrun_proc.stdout.decode("utf-8").strip()

    print("=> Using Xcode toolchain:")
    print(f"   Sysroot:      {sysroot}")
    print(f"   C Compiler:   {xcode_c_compiler}")
    print(f"   C++ Compiler: {xcode_cxx_compiler}")

    package_tree = vtk_package_tree()
    if not keep:
        _paths_to_remove.append(package_tree.root)

    print("=> Creating the following directories:")
    print(f"   Containment folder: {package_tree.root}")
    print(f"   VTK source code:    {package_tree.source_dir}")
    print(f"   VTK build tree:     {package_tree.build_dir}")
    print(f"   VTK install tree:   {package_tree.install_dir}")

    # Make sure we have the root available.
    if not package_tree.root.exists():
        package_tree.root.mkdir(parents=True, exist_ok=False)
    elif not package_tree.root.is_dir():
        raise RuntimeError(
            f"{package_tree.root} exists, but is not a directory. Refusing to "
            "overwrite, please delete manually and rerun."
        )

    # Only clone if we need to.  If it is being run after a --keep, then the
    # clone is already there.  Just run `checkout` to ensure we have the right
    # worktree available.
    vtk_ref = vtk_git_ref()
    if not package_tree.source_dir.exists():
        clone_vtk(vtk_ref, package_tree.source_dir)
    elif package_tree.source_dir.is_dir():
        subprocess.check_call(
            [
                "git",
                "checkout",
                vtk_ref,
            ],
            cwd=package_tree.source_dir,
        )
    else:
        raise RuntimeError(
            f"{package_tree.source_dir} exists, but is not a directory. "
            "Refusing to overwrite, please delete manually and rerun."
        )

    configure_args = [
        f"-DCMAKE_OSX_ARCHITECTURES:STRING={architecture()}",
        "-DCMAKE_INSTALL_LIBDIR=lib",
        # NOTE: OpenGL is a framework (we cannot use NEVER).
        "-DCMAKE_FIND_FRAMEWORK=LAST",
        f"-DCMAKE_OSX_SYSROOT={sysroot}",
        f"-DCMAKE_C_COMPILER={xcode_c_compiler}",
        f"-DCMAKE_CXX_COMPILER={xcode_cxx_compiler}",
        *vtk_cmake_configure_args(),
    ]
    build_vtk(package_tree, configure_args)
    vtk_archive = package_vtk(package_tree)

    # package_vtk created the archive in the package_tree.root directory, move
    # the final product back to where the user requested with --output-dir.
    tar_gz_dest = output_dir / vtk_archive.tar_gz_path.name
    vtk_archive.tar_gz_path.rename(tar_gz_dest)

    sha256_sum_dest = output_dir / vtk_archive.sha256_sum_path.name
    vtk_archive.sha256_sum_path.rename(sha256_sum_dest)

    print("SUCCESS:")
    print(f"  Archive:  {tar_gz_dest}")
    print(f"  Checksum: {sha256_sum_dest}")


def main() -> None:
    # Work around `bazel run` changing the working directory; this is to allow
    # the user to pass in relative paths in a sane manner.
    real_cwd = os.environ.get("BUILD_WORKING_DIRECTORY")
    if real_cwd is not None:
        os.chdir(real_cwd)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o",
        "--output-dir",
        metavar="DIR",
        type=Path,
        default=Path(".").resolve(),
        help="directory to put the final .tar.gz in.  Default: %(default)s",
    )

    # On Linux we have docker, on macOS we just build in a dedicated location.
    # Verify Linux or macOS once, everything else assumes !Linux => macOS.
    if system_is_linux():
        keep_help = "permanently tag docker container"
    elif system_is_macos():
        package_tree = vtk_package_tree()
        keep_help = (
            "retain the VTK checkout, build, and installation trees "
            f"contained in {package_tree.root}"
        )
    else:
        parser.error(f"Unsupported operating system: {platform.system()}")
    parser.add_argument("-k", "--keep", action="store_true", help=keep_help)

    # On Linux we build using docker and can build both, or one / the other.
    # On macOS we cannot build a "fat-binary" with both x86_64 and arm64
    # because of the VTK dependencies.
    if system_is_linux():
        focal = DockerPlatform("focal", "ubuntu:20.04")
        jammy = DockerPlatform("jammy", "ubuntu:22.04")
        all_platforms = [focal, jammy]  # used after parsing args too
        parser.add_argument(
            "-p",
            "--platform",
            type=str,
            choices=["all"] + [p.codename for p in all_platforms],
            default="all",
            help="which distribution(s) to build.  Default: %(default)s",
        )

    # NOTE: this mode is useful for staging a VTK bisect.
    parser.add_argument(
        "--dump-cmake-configure-args",
        action="store_true",
        help=(
            "Do not package, just print the VTK CMake configuration arguments "
            "drake is using for the platform running this command."
        ),
    )

    args = parser.parse_args()

    if args.dump_cmake_configure_args:
        print(" ".join(vtk_cmake_configure_args()))
    else:
        if system_is_linux():
            if args.platform == "all":
                platforms = all_platforms
            else:
                platforms = [
                    p for p in all_platforms if args.platform == p.codename
                ]
            build_linux(args.output_dir, platforms, args.keep)
        else:
            build_macos(args.output_dir, args.keep)


if __name__ == "__main__":
    main()
