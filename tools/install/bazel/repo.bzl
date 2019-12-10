# -*- mode: python -*-
# vi: set ft=python :

def _call_drake_impl(*args):  # Akin to a forward declaration.
    _drake_impl(*args)

drake_repository = repository_rule(
    implementation = _call_drake_impl,
    local = True,
    doc = """
Declares the @drake repository based on an installed Drake binary image.

This enables downstream BUILD files to refer to Drake targets such as
@drake//bindings/pydrake even when using precompiled Drake releases.

Only a limited number of targets are supported, currently only:
- @drake//bindings/pydrake

To use this rule, insert the following stanza into your WORKSPACE:
```
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
http_archive(
    name = "drake_binary",
    url = "https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-bionic.tar.gz",  # noqa
    build_file_content = "#",
)
load("@drake_binary//:share/drake/repo.bzl", "drake_repository")
drake_repository(name = "drake")
```

For more detailed examples of proper use, including how to use a numbered
release instead of an arbitrary "latest" release, see
https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_bazel_installed
""",
)

def _drake_impl(repo_ctx):
    # Obtain the root of the @drake_loader repository (i.e., wherever this
    # repo.bzl file came from).
    loader_workspace = repo_ctx.path(Label("//:WORKSPACE")).dirname

    # If the loader came from an http_archive of a Drake binary release, its
    # workspace will have paths like drake/lib/..., drake/share/..., etc.
    # If the loader came from a new_local_repository on disk, the `path = ...`
    # provided to new_local_repository might already incorporate the "drake"
    # prefix so have paths like lib/..., share/..., etc.  We'll automatically
    # detect which case is in effect.
    for prefix in [
        loader_workspace,
        loader_workspace.get_child("drake"),
        None,
    ]:
        if prefix == None:
            fail("Could not find drake sentinel under {}".format(
                loader_workspace,
            ))
        share = prefix.get_child("share")
        share_drake = share.get_child("drake")
        sentinel = share_drake.get_child(".drake-find_resource-sentinel")
        if sentinel.exists:
            break

    # Sanity check ${prefix}.
    required_files = [
        "include/drake/common/drake_assert.h",
        "lib/libdrake.so",
        "share/drake/setup/install_prereqs",
    ]
    for required_file in required_files:
        if not repo_ctx.path(str(prefix) + "/" + required_file).exists:
            fail("The drake install prefix {} is missing file {}".format(
                prefix,
                required_file,
            ))

    # Symlink our libraries into the repository.  We can use any path for
    # these, since our BUILD rules will declare new names for everything,
    # unrelated to their physical structure here.
    repo_ctx.symlink(prefix.get_child("lib"), ".lib")

    # Create the stub BUILD files.  (During development, these live at
    # drake/tools/install/bazel/drake**.BUILD.bazel in the source tree.)
    for path, body in _build_file_contents.items():
        repo_ctx.file(path, content = body, executable = False)

    # Symlink the data resources into the repository.  These must exactly match
    # a Drake source tree's physical structure, since we cannot easily alter
    # the path for runfiles via our BUILD files.
    for relpath in MANIFEST["runfiles"]["drake"]:
        repo_ctx.symlink(str(share_drake) + "/" + relpath, relpath)

    # Emit the manifest for later loading.
    manifest_bzl = "MANIFEST = " + struct(**MANIFEST).to_json()
    repo_ctx.file("_manifest.bzl", content = manifest_bzl, executable = False)

# Below this point, we'll have a `_build_file_contents = dict()` that contains
# the contents of the stub BUILD files.  (When compiling Drake, the repo_gen.py
# tool automatically appends that content before we install this file.)
