# -*- python -*-

def forward_files(
        srcs,
        strip_prefix,
        dest_prefix,
        visibility = None,
        tags = []):
    """Forwards files in `srcs` to be physically present in the current
    packages.

    Present implementation simply copies the files.

    @param srcs
        List of string, pointing *directly* to files. Does not handle filegroup
        targets.
    @param strip_prefix
        String to be stripped from source files. Should include trailing slash.
    @param dest_prefix
        String to be prepended to target.
    """
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
