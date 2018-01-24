# -*- python -*-

# Load the relevant upstream methods.
load(
    "@intellij_aspect//:intellij_info_impl.bzl",
    "make_intellij_info_aspect",
    "intellij_info_aspect_impl",
)
load(
    "@intellij_aspect//:intellij_info_bundled.bzl",
    "tool_label",
)

# Load the magic.
load(
    "@drake_clion_environment//:path.bzl",
    "additional_transitive_quote_include_directory",
)

# This is part of the `semantics` hooks provided by intellij_aspect's aspect.
# We use it to rewrite the transitive_quote_include_directory to contain an
# entry for Drake that avoids the virtual_headers symlink farm.
def _extra_ide_info(target, ctx, ide_info, ide_info_file, output_groups):
    # N.B. All of our return statements here should return False, which means
    # "I was not the primary semantics for this target".  The intellij_aspect's
    # code is always the primary semantics, we're just (potentially) frobbing
    # its results.
    if "c_ide_info" not in ide_info:
        return False
    # Convert c_ide_info from a struct to a dict.
    old_c_info = ide_info["c_ide_info"]
    new_c_info = dict()
    for key in dir(old_c_info):
        maybe_value = getattr(old_c_info, key, None)
        if maybe_value != None:  # Ignore methods; only want attributes.
            new_c_info[key] = maybe_value
    # Prepend our new include directories to the list of paths.
    frob_key = "transitive_quote_include_directory"
    maybe_prepend = additional_transitive_quote_include_directory
    if maybe_prepend and frob_key in new_c_info:
        new_c_info[frob_key].insert(0, maybe_prepend)
        # Write back c_ide_info as a struct.
        ide_info["c_ide_info"] = struct(**new_c_info)
    return False

# Define the relevant semantics (see intellij_info_bundled.bzl).
semantics = struct(
    extra_ide_info = _extra_ide_info,
    tool_label = tool_label,
)

# Curry our semantics argument into the aspect rule's implementation function.
def _aspect_impl(target, ctx):
    return intellij_info_aspect_impl(target, ctx, semantics)

# This is the command-line entry point.
intellij_info_aspect = make_intellij_info_aspect(_aspect_impl, semantics)
