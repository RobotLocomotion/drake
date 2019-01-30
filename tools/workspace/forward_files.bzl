# -*- python -*-

def forward_files(
        srcs = [],
        strip_prefix = None,
        dest_prefix = None,
        visibility = None,
        tags = []):
    """Forwards files in `srcs` to be physically present in the current
    packages.

    Present implementation simply copies the files.

    @param srcs
        List of string, pointing *directly* to files as absolute Bazel target
        paths. This does NOT resolve relative targets, nor does it unpack
        filegroup targets `$(locations ...)`.
    @param strip_prefix
        String to be stripped from source files. Should include trailing slash.
    @param dest_prefix
        String to be prepended to target.
    """
    if strip_prefix == None or dest_prefix == None:
        fail("Must supply `strip_prefix` and `dest_prefix`.")
    outs = []
    for src in srcs:
        if not src.startswith(strip_prefix):
            fail("'{}' not under '{}'".format(src, strip_prefix))
        out = dest_prefix + src[len(strip_prefix):]
        native.genrule(
            name = out + ".forward",
            srcs = [src],
            outs = [out],
            cmd = "cp $< $@",
            tags = tags,
            visibility = visibility,
        )
        outs.append(out)
    return outs
