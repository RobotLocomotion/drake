load(
    "@drake//tools/workspace:github.bzl",
    "setup_github_repository",
)
load(
    "@drake//tools/workspace:execute.bzl",
    "execute_or_fail",
)

def parse_module(repo_ctx, subdir):
    """Parses and returns a vtk.module file as a dict.

    An upstream `Foo/Bar/vtk.module` file is formatted like this:
    NAME
      VTK::FooBar
    THIRD_PARTY
    DEPENDS
      VTK::CommonCore
      VTK::CommonDataModel

    Our returned dict will look like this:
    {
      "subdir": "Foo/Bar",
      "NAME": "VTK::FooBar",
      "THIRD_PARTY": 1,
      "DEPENDS: [
        "VTK::CommonCore",
        "VTK::CommonDataModel"
      ],
    }

    Internally, to make it easier to parse each dict item as a single line,
    we'll munge the file contents to have one line per dict key:

    NAME=VTK::FooBar
    THIRD_PARTY
    DEPENDS=VTK::CommonCore=VTK::CommonDataModel
    """

    result = dict(subdir = subdir)
    content = repo_ctx.read(subdir + "/vtk.module")
    lines = content.replace("\n  ", "=").splitlines()
    for line in lines:
        tokens = line.split("=")
        key, values = tokens[0], tokens[1:]
        if key.endswith("S"):
            # Plural key.
            result[key] = values
        else:
            # Singular key.
            if len(values) == 1:
                result[key] = values[0]
            elif len(values) == 0:
                # This a a boolean-like item (either present, or not).
                # We'll encode that as an int to avoid JSON encoding snafus.
                result[key] = 1
            else:
                fail("{}: Too may values for {}".format(subdir, key))
    return result

def create_modules_bzl(repo_ctx):
    """Finds all vtk.module files, parses them, and writes their content into
    an loadable `modules.bzl` file in the root of the repository.

    This is necessary because BUILD files can't parse external metdata as part
    of their rules; the only thing they can do is load `*.bzl` files, so we
    must convert the module metadata to bzl.
    """

    # Find all vtk.module files.
    subdirs = []
    for line in execute_or_fail(
        repo_ctx,
        ["/usr/bin/find", ".", "-name", "vtk.module"],
    ).stdout.splitlines():
        # Remove the leading "./" and tailing "/vtk.module".
        subdir = line[2:-11]
        subdirs.append(subdir)

    # Parse all vtk.module files.
    modules = dict()
    for subdir in subdirs:
        content = parse_module(repo_ctx, subdir)
        modules[content["NAME"]] = content

    # Encode the output. Because we lean on json encoding, this will not work
    # correctly if anything in the data structure is None, True, or False.
    # We rely on parse_module() to avoid that situation.
    bzl_content = "MODULES = " + json.encode(modules) + "\n"

    # Pass along the os.name and os.arch for convenience.
    platform = dict(name = repo_ctx.os.name, arch = repo_ctx.os.arch)
    bzl_content += "PLATFORM = " + json.encode(platform) + "\n"

    # Write the output.
    repo_ctx.file("modules.bzl", content = bzl_content)

def _impl(repo_ctx):
    error = setup_github_repository(repo_ctx).error
    if error != None:
        fail(error)
    create_modules_bzl(repo_ctx)
    repo_ctx.symlink(repo_ctx.attr.settings_bzl, "settings.bzl")

vtk_internal_repository = repository_rule(
    attrs = {
        # These are the attributes for setup_github_repository.
        "repository": attr.string(
            default = "Kitware/VTK",
        ),
        "commit": attr.string(
            # TODO(jwnimmer-tri) Once there's a tagged release with support
            # for VTK_ABI_NAMESPACE, we should switch to an official version
            # number here. That probably means waiting for the VTK 10 release.
            default = "d706250a1422ae1e7ece0fa09a510186769a5fec",
        ),
        "commit_pin": attr.int(
            # See above. There's not any satisfactory tagged version yet.
            default = 1,
        ),
        "sha256": attr.string(
            default = "6106493c8a6e9bd36250e80e4441a1644cd9bff706e6171607f996f0233f515c",  # noqa
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/vtk_internal:package.BUILD.bazel",  # noqa
        ),
        "patches": attr.label_list(
            default = [
                "@drake//tools/workspace/vtk_internal:patches/common_core_version.patch",  # noqa
                "@drake//tools/workspace/vtk_internal:patches/common_core_vs_data_model_cycle.patch",  # noqa
                "@drake//tools/workspace/vtk_internal:patches/common_core_warnings.patch",  # noqa
                "@drake//tools/workspace/vtk_internal:patches/common_data_model_warnings.patch",  # noqa
                "@drake//tools/workspace/vtk_internal:patches/io_image_formats.patch",  # noqa
            ],
        ),
        "extra_strip_prefix": attr.string(),
        "mirrors": attr.string_list_dict(),
        # This attribute is specific to our rule, not setup_github_repository.
        "settings_bzl": attr.label(
            allow_single_file = True,
            default = Label("@drake//tools/workspace/vtk_internal:settings.bzl"),  # noqa
        ),
    },
    implementation = _impl,
)
