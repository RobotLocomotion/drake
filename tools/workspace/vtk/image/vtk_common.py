from __future__ import annotations

import hashlib
import platform
import shutil
import subprocess
import tarfile
from dataclasses import dataclass
from pathlib import Path

# NOTE: on the linux build, we need `runfiles` for `tools/workspace/package.py`
# to know where to run docker.  However, inside of docker, we cannot import
# `runfiles` and do not need to.  On macOS, though, we do need it.
try:
    from bazel_tools.tools.python.runfiles import runfiles

    BAZEL_TOOLS_RUNFILES_IMPORTED = True
except ImportError:
    BAZEL_TOOLS_RUNFILES_IMPORTED = False


@dataclass
class PackageTree:
    root: Path
    source_dir: Path
    build_dir: Path
    install_dir: Path


@dataclass
class VtkArchive:
    tar_gz_path: Path
    sha256_sum_path: Path


def system_is_linux():
    return platform.system() == "Linux"


def system_is_macos():
    return platform.system() == "Darwin"


def architecture() -> str:
    machine = platform.machine()
    assert machine in {
        "x86_64",
        "arm64",
    }, f"Unsupported build architecture {machine}"
    return machine


def build_number() -> int:
    """The build number for the S3 artifact.

    Any time a VTK .tar.gz archive needs to be repackaged, the build number
    should be increased by 1.  Never overwrite VTK .tar.gz archives on S3,
    historical builds of drake will stop working.

    NOTE: rebuilds are infrequent, and usually only need to happen when there
    are dependency related updates / changes / issues.
    """
    arch = architecture()
    code = codename()
    if code == "focal":
        if arch == "x86_64":
            return 1  # focal x86_64
        elif arch == "arm64":
            return 1  # focal arm64
    elif code == "jammy":
        if arch == "x86_64":
            return 1  # jammy x86_64
        elif arch == "arm64":
            return 1  # jammy arm64
    elif code == "mac":
        if arch == "x86_64":
            return 1  # macOS x86_64
        elif arch == "arm64":
            return 1  # macOS arm64

    raise NotImplementedError("Unrecognized platform.")


def codename() -> str:
    if platform.system() == "Linux":
        import lsb_release

        return lsb_release.get_os_release()["CODENAME"]

    return "mac"


def _rlocation(relative_path: str) -> Path:
    """Return the real path to ``tools/workspace/vtk/{relative_path}``."""
    if BAZEL_TOOLS_RUNFILES_IMPORTED:
        manifest = runfiles.Create()
        resource_path = f"drake/tools/workspace/vtk/{relative_path}"
        resolved_path = manifest.Rlocation(resource_path)
        assert resolved_path, f"Missing {resource_path}"
        return Path(resolved_path).resolve()

    # Fail docker builds that reach this statement.
    raise RuntimeError(
        "_rlocation cannot be used, execution of "
        "`from bazel_tools.tools.python.runfiles import runfiles` failed."
    )


def vtk_git_ref() -> str:
    """The commit / branch / ref to build for VTK.

    NOTE: when updating this, reset all build_number()'s to 1.
    """
    return "d706250a1422ae1e7ece0fa09a510186769a5fec"


def vtk_package_tree() -> PackageTree:
    if system_is_linux():
        root = Path("/vtk")
    else:
        # We are in tools/workspace/vtk/image/, build in vtk/mac_binary_build.
        this_file_real_path = _rlocation("image/vtk_common.py")
        tools_workspace_vtk_path = this_file_real_path.parent.parent.resolve()
        root = tools_workspace_vtk_path / "mac_binary_build"
    source_dir = root / "source"
    build_dir = root / "build"
    install_dir = root / "install"
    return PackageTree(
        root=root,
        source_dir=source_dir,
        build_dir=build_dir,
        install_dir=install_dir,
    )


def vtk_version(source_dir: Path) -> str:
    proc = subprocess.run(
        ["git", "describe"], check=True, cwd=source_dir, stdout=subprocess.PIPE
    )
    return proc.stdout.decode("utf-8").strip()


def vtk_archive_name(source_dir: Path) -> str:
    version = vtk_version(source_dir)
    code = codename()
    arch = architecture()
    build = build_number()
    return f"vtk-{version}-{code}-{arch}-{build}.tar.gz"


def build_vtk(package_tree: PackageTree, configure_args: list[str]):
    """Build VTK with the specified CMake configure arguments using Ninja."""
    # Always start with a clean CMake build tree on Linux.  On macOS, reuse the
    # existing build tree and re-run CMake configure.
    if system_is_linux():
        if package_tree.build_dir.is_dir():
            shutil.rmtree(package_tree.build_dir)
        elif package_tree.build_dir.is_file():
            package_tree.build_dir.unlink()
        exist_ok = False
    elif system_is_macos():
        exist_ok = True
    package_tree.build_dir.mkdir(parents=True, exist_ok=exist_ok)

    # Always start with a clean install tree.  Do this before building to save
    # time when there are errors.
    if package_tree.install_dir.is_dir():
        shutil.rmtree(package_tree.install_dir)
    elif package_tree.install_dir.is_file():
        package_tree.install_dir.unlink()
    if package_tree.install_dir.exists():
        raise RuntimeError(
            f"{package_tree.install_dir} exists but could not be removed, "
            "please delete manually and rerun."
        )

    # (Re)run CMake configure.
    subprocess.check_call(
        [
            "cmake",
            "-Werror=dev",
            "-G",
            "Ninja",
            f"-DCMAKE_INSTALL_PREFIX:PATH={package_tree.install_dir}",
            "-DCMAKE_BUILD_TYPE:STRING=Release",
            *configure_args,
            "-B",
            str(package_tree.build_dir),
            "-S",
            str(package_tree.source_dir),
        ]
    )
    # Run CMake build / install.
    subprocess.check_call(
        [
            "cmake",
            "--build",
            str(package_tree.build_dir),
            "--target",
            "install",
        ]  # this comment stops black from making the line too long.
    )


def package_vtk(package_tree: PackageTree) -> VtkArchive:
    """Create a .tar.gz of the package_tree.install_dir and return the path."""
    archive_name = vtk_archive_name(package_tree.source_dir)
    tar_gz_path = package_tree.root / archive_name
    # Creating the .tgz can take a little while, display something to the user
    # so it's clear the script isn't stuck.
    print(f"=> Compressing {package_tree.install_dir} to {tar_gz_path}.")
    with tarfile.open(tar_gz_path, "w:gz") as tgz:
        # NOTE: the second argument to `add` is what to rename things as, by
        # using the empty string it means that all components are stripped.
        # This way, when extracting, the bin, lib, etc directories are created.
        tgz.add(package_tree.install_dir, "")

    # Create the accompanying sha256sum verification file.
    sha256_sum_path = tar_gz_path.parent / f"{tar_gz_path.name}.sha256"
    print(f"=> Computing sha256sum of {tar_gz_path} into {sha256_sum_path}.")
    sha256 = hashlib.sha256()
    with open(tar_gz_path, "rb", buffering=0) as tgz:
        while True:
            data = tgz.read(sha256.block_size)
            if not data:
                break
            sha256.update(data)
    with open(sha256_sum_path, "w") as f:
        f.write(f"{sha256.hexdigest()} {tar_gz_path.name}\n")

    return VtkArchive(tar_gz_path, sha256_sum_path)
