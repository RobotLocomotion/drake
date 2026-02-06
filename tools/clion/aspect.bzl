load("@bazel_skylib//lib:structs.bzl", "structs")

# Load the magic.
load(
    "@drake_clion_environment//:path.bzl",
    "additional_transitive_quote_include_directory",
)

# Load the relevant upstream methods.
load(
    "//.clwb_aspects:intellij_info_bundled.bzl",
    "semantics",
)
load(
    "//.clwb_aspects:intellij_info_impl_bundled.bzl",
    "intellij_info_aspect_impl",
    "make_intellij_info_aspect",
)

# This is part of the `semantics` hooks provided by intellij_aspect's aspect.
# We use it to rewrite the transitive_quote_include_directory to contain an
# entry for Drake that avoids the virtual_headers symlink farm.
#
# For reference, ~/.CLion2017.2/config/plugins/clwb/intellij_info_impl.bzl is
# the code that ends up calling this function -- read its source to understand
# the data types that we are manipulating here.
def _extra_ide_info(target, ctx, ide_info, ide_info_file, output_groups):
    # N.B. All of our return statements here should return False, which means
    # "I was not the primary semantics for this target".  The intellij_aspect's
    # code is always the primary semantics, we're just (potentially) frobbing
    # its results.
    if not additional_transitive_quote_include_directory:
        # No Drake path needs to be inserted.
        return False
    c_ide_info = ide_info.get("c_ide_info", None)
    if c_ide_info == None:
        # Not a C / C++ target.
        return False

    # Place Drake's parent directory on CLion's include path.
    includes = c_ide_info.compilation_context.quote_includes
    includes.append(additional_transitive_quote_include_directory)
    return False

# Curry our semantics argument into the aspect rule's implementation function.
def _aspect_impl(target, ctx):
    new_semantics = struct(
        extra_ide_info = _extra_ide_info,
        **structs.to_dict(semantics)
    )
    return intellij_info_aspect_impl(target, ctx, new_semantics)

# This is the command-line entry point.
intellij_info_aspect = make_intellij_info_aspect(_aspect_impl, semantics)
