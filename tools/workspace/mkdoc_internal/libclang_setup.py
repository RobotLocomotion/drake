import glob
import os
import platform
import subprocess

from clang import cindex

# Alternative: Make this a function in `mkdoc.py`, and import it from mkdoc as
# a module? (if this was really authored in `mkdoc.py`...)


def add_library_paths(parameters=None):
    """Set library paths for finding libclang on supported platforms.

    Args:
        parameters(list): If not None, it's used for adding parameters which
            are used in `mkdoc.py`.

    Returns:
    """
    library_file = None
    if platform.system() == "Darwin":
        completed_process = subprocess.run(
            ["xcrun", "--find", "clang"],
            stdout=subprocess.PIPE,
            encoding="utf-8",
        )
        if completed_process.returncode == 0:
            toolchain_dir = os.path.dirname(
                os.path.dirname(completed_process.stdout.strip())
            )
            library_file = os.path.join(toolchain_dir, "lib", "libclang.dylib")
        completed_process = subprocess.run(
            ["xcrun", "--show-sdk-path"],
            stdout=subprocess.PIPE,
            encoding="utf-8",
        )
        if parameters is not None and completed_process.returncode == 0:
            sdkroot = completed_process.stdout.strip()
            if os.path.exists(sdkroot):
                parameters += ["-isysroot", sdkroot]
    elif platform.system() == "Linux":
        # By default we expect Clang 15 to be installed, but on Ubuntu >= 24.04
        # we'll use Clang 19 for compatibility with GCC >= 14.
        version = 15
        try:
            completed_process = subprocess.run(
                ["lsb_release", "-sr"], stdout=subprocess.PIPE, encoding="utf-8"
            )
            if completed_process.returncode == 0:
                output = completed_process.stdout.strip()
                ubuntu_major = int(output.split(".")[0])
                if ubuntu_major >= 24:
                    version = 19
        except Exception:
            pass
        arch = platform.machine()
        llvm_root = f"/usr/lib64/llvm{version}"
        if os.path.exists(llvm_root):
            # Looks like AlmaLinux (or Red Hat, Fedora, etc.).
            library_file = f"{llvm_root}/lib/libclang.so"
            cppinclude = glob.glob(f"{llvm_root}/lib/clang/*/include")
            if len(cppinclude) == 1:
                parameters += ["-isystem", cppinclude[0]]
        else:
            # Hope it's Ubuntu...
            library_file = f"/usr/lib/{arch}-linux-gnu/libclang-{version}.so"
    if not os.path.exists(library_file):
        raise RuntimeError(f"Library file {library_file} does NOT exist")
    cindex.Config.set_library_file(library_file)
