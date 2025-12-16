def _path_stem(src):
    """Returns e.g. "quux" when given "foo/bar/quux.ext".
    """
    return src.split("/")[-1].split(".")[0]

def generate_rendering_opengl2_sources():
    name = "generated_rendering_opengl2_sources"
    hdrs = []
    for src in native.glob([
        "Rendering/OpenGL2/glsl/*.glsl",
        "Rendering/OpenGL2/textures/*.jpg",
    ]):
        stem = _path_stem(src)
        hdr = "Rendering/OpenGL2/" + stem + ".h"
        native.genrule(
            name = "_genrule_" + hdr,
            srcs = [src],
            outs = [hdr],
            cmd = "$(execpath :data_to_header) $< > $@",
            tools = [":data_to_header"],
        )
        hdrs.append(hdr)
    native.filegroup(name = name, srcs = hdrs)
