# -*- mode: python -*-
# vi: set ft=python :

# * # Comment lines beginning with a "# * #" are stripped out as part of the
# * # conversion from repo_template.bzl to repo.bzl via the repo_gen tool.

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

For an example of proper use, see
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
    repo_ctx.symlink(prefix.get_child("include"), ".include")
    repo_ctx.symlink(prefix.get_child("lib"), ".lib")

    # Create the stub BUILD files.  (During development, these live at
    # drake/tools/install/bazel/drake**.BUILD.bazel in the source tree.)
    for path, body in _BUILD_FILE_CONTENTS.items():
        repo_ctx.file(path, content = body, executable = False)

    # Symlink the data resources into the repository.  These must exactly match
    # a Drake source tree's physical structure, since we cannot easily alter
    # the path for runfiles via our BUILD files.
    for relpath in _MANIFEST["runfiles"]["drake"]:
        repo_ctx.symlink(str(share_drake) + "/" + relpath, relpath)

    # Symlink all drake LCM types to this repository's root package, since it
    # should be named `drake` (see bazelbuild/bazel#3998).
    if repo_ctx.name != "drake":
        print("WARNING: Drake LCM types will not be importable via `drake` " +
              "if this repository is not named `drake`.")
    python_site_packages_relpath = _MANIFEST["python_site_packages_relpath"]
    drake_lcmtypes_package = "." + python_site_packages_relpath + "/drake"
    for relpath in _MANIFEST["lcmtypes_drake_py"]:
        repo_ctx.symlink(drake_lcmtypes_package + "/" + relpath, relpath)

    # Emit the manifest for later loading.
    manifest_bzl = "MANIFEST = " + struct(**_MANIFEST).to_json()
    repo_ctx.file(".manifest.bzl", content = manifest_bzl, executable = False)

    # Annotate the OS for use by our BUILD files.
    os_bzl = "NAME = \"{}\"\n".format(repo_ctx.os.name)
    repo_ctx.file(".os.bzl", content = os_bzl, executable = False)

# * # This placeholder definition in repo_template.bzl is rewritten by repo_gen
# * # during the Drake source build.  Its new contents will be the bodies of
# * # the drake*.BUILD.bazel stub files.  See README.md for details.
_BUILD_FILE_CONTENTS = {"common/BUILD.bazel": "# This is @drake//common."}

# * # This placeholder definition in repo_template.bzl is rewritten by repo_gen
# * # during the Drake source build.  Its new the contents will be the manifest
# * # of Drake's installed runfiles.  See README.md for details.
_MANIFEST = {"runfiles": {"drake": ["common/bar", "examples/baz"]}}
