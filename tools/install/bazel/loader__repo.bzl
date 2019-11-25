# -*- mode: python -*-
# vi: set ft=python :

def _drake_impl(repo_ctx):
    # Find ${INSTALL_ROOT}/share/drake_bazel_workspace_loader/repo.bzl.
    loader = "@drake_bazel_workspace_loader"
    me = repo_ctx.path(Label(loader + "//:repo.bzl")).realpath
    me.exists or fail("Cannot resolve self-path")

    # Find ${INSTALL_ROOT}.
    root = me.dirname.dirname.dirname
    me.exists or fail("Cannot resolve root")

    # Sanity check ${INSTALL_ROOT}.
    required_files = [
        "share/drake/.drake-find_resource-sentinel",
        "share/drake/setup/install_prereqs",
    ]
    for required_file in required_files:
        abspath = str(root) + "/" + required_file
        if not repo_ctx.path(abspath).exists:
            fail("Missing file " + abspath)

    # Symlink the data resources into the repository.
    share = root.get_child("share")
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
    lib = root.get_child("lib")
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

    # Symlink the stub BUILD files.
    # TODO(jwnimmer-tri) If the resources_data_subdirs and these packages
    # overlap, will we accidentally place a BUILD file into the install tree
    # via the symlink?
    build_files = [
        "drake__BUILD.bazel",
        "drake__bindings__pydrake__BUILD.bazel",
    ]
    for mangled_relative_path in build_files:
        relative_path = mangled_relative_path.replace("__", "/")
        repo_ctx.symlink(
            Label(loader + "//:" + mangled_relative_path),
            relative_path[len("drake/"):],
        )

drake_repository = repository_rule(
    implementation = _drake_impl,
    local = True,
)
"""XXX
"""
