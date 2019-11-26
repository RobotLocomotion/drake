# -*- mode: python -*-
# vi: set ft=python :

def _drake_impl(repo_ctx):
    # Find the root of the @drake_loader repository.
    loader_workspace = repo_ctx.path(Label("//:WORKSPACE")).dirname

    # If the loader came from an http_archive of a Drake binary release, it
    # will have paths like drake/lib/..., drake/share/..., etc.  If the loader
    # came from a new_local_repository on disk, the `path = ...` might already
    # incorporate the "drake" prefix so have paths like lib/..., share/...,
    # etc.  We'll automatically detect which case is in effect.
    # TODO(jwnimmer-tri) Detect.
    prefix = loader_workspace

    # Sanity check ${prefix}.
    required_files = [
        "share/drake/.drake-find_resource-sentinel",
        "share/drake/setup/install_prereqs",
    ]
    for required_file in required_files:
        abspath = str(prefix) + "/" + required_file
        if not repo_ctx.path(abspath).exists:
            fail("Missing file " + abspath)

    # Symlink the data resources into the repository.
    share = prefix.get_child("share")
    share.exists or fail("Missing " + share)
    share_drake = share.get_child("drake")
    share_drake.exists or fail("Missing " + share_drake)
    resources_data_files = []
    resources_data_subdirs = []
    for resource_path in share_drake.readdir():
        basename = resource_path.basename
        is_file = basename.startswith(".")
        if is_file:
            resources_data_files.append(basename)
        else:
            resources_data_subdirs.append(basename)
        repo_ctx.symlink(resource_path, basename)

    # Symlink the binaries into the repository.
    lib = prefix.get_child("lib")
    lib.exists or fail("Missing " + lib)
    repo_ctx.symlink(lib, ".lib")

    # Write some variable definitions for later loading.
    repo_ctx.file(
        ".vars.bzl",
        content = "\n".join([
            "resources_data_files = " + str(resources_data_files),
            "resources_data_subdirs = " + str(resources_data_subdirs),
        ]),
        executable = False,
    )

    # Create the stub BUILD files.
    # TODO(jwnimmer-tri) If the resources_data_subdirs and these packages
    # overlap, will we accidentally place a BUILD file into the install tree
    # via the symlink?
    for path, body in _build_file_contents.items():
        repo_ctx.file(path, content = body, executable = False)

drake_repository = repository_rule(
    implementation = _drake_impl,
    local = True,
)
"""XXX
"""

# Below this point, we'll have a `_build_file_contents = dict()` that contains
# the contents of the stub BUILD files.  (When creating the install tree, the
# repo_gen.py tool appends that content.)
