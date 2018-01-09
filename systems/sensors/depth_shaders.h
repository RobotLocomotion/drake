#pragma once

namespace drake {
namespace systems {
namespace sensors {
namespace shaders {

/// A vertex shader program for rendering depth images, which computes vertices
/// and normals for the fragment shader program coming after.
constexpr char kDepthVS[] =
    "//VTK::System::Dec\n"  // Always start with this line.
    "attribute vec4 vertexMC;\n"
    "attribute vec3 normalMC;\n"
    "uniform mat3 normalMatrix;\n"
    "uniform mat4 MCDCMatrix;\n"
    "uniform mat4 MCVCMatrix;\n"
    "varying vec3 normalVCVSOutput;\n"
    "varying vec4 vertexVCVSOutput;\n"
    "attribute vec2 tcoordMC;\n"
    "varying vec2 tcoordVCVSOutput;\n"
    "void main () {\n"
    "  normalVCVSOutput = normalMatrix * normalMC;\n"
    "  tcoordVCVSOutput = tcoordMC;\n"
    "  vertexVCVSOutput = MCVCMatrix * vertexMC;\n"
    "  gl_Position = MCDCMatrix * vertexMC;\n"
    "}\n";

/// A fragment shader program for rendering depth images, which computes depth
/// values for each pixel in depth images, converts them to be in range [0, 1]
/// and packs those values to three color channels. In other words, we encode
/// a depth image into a color image and output the color image. For the detail
/// of packing algorithm, please refer to:
/// https://aras-p.info/blog/2009/07/30/encoding-floats-to-rgba-the-final/
/// Note that we are only using three channels instead of four channels to
/// express a float value. This differs from the example code in the link above.
/// The reason is that we need to set one to alpha channel so that the rendered
/// "image" will be opaque. Otherwise, we will have different colors from what
/// we output here, thus expect, in the end.

constexpr char kDepthFS[] =
    "//VTK::System::Dec\n"  // Always start with this line.
    "//VTK::Output::Dec\n"  // Always have this line in your FS.
    "varying vec3 normalVCVSOutput;\n"
    "varying vec4 vertexVCVSOutput;\n"
    "varying vec2 tcoordVCVSOutput;\n"
    "out vec4 color_out;\n"
    "uniform float z_near;\n"
    "uniform float z_far;\n"
    "\n"
    "// This function splits a float value, whose range is [0, 1], to three\n"
    "// float values, whose ranges are also [0, 1] but will eventually be\n"
    "// converted to be [0, 255] of unsigned char. Each of the split float\n"
    "// values holds specific decimal portion of the original float value\n"
    "// using a bit shift operation, an integer part truncation and a bit\n"
    "// mask operation. The maximum amount of information that each of the\n"
    "// float value can hold is up to 8 bits which is the size of unsigned\n"
    "// char and that is why we use a magic number 255 a lot in this\n"
    "// function.\n"
    "vec3 PackFloatToVec3i(float value) {\n"
    "  const vec3 bit_shift = vec3(255. * 255., 255., 1.);\n"
    "  const float tmp = 1. / 255.;"
    "  const vec3 bit_mask = vec3(0., tmp, tmp);\n"
    "  vec3 res = fract(value * bit_shift);\n"
    "  return res - (res.xxy * bit_mask);\n"
    "}\n"
    "\n"
    "void main () {\n"
    "  float z = -vertexVCVSOutput.z;  // In meters.\n"
    "  // Converting meters to [0, 1].\n"
    "  float z_norm = (z - z_near) / (z_far - z_near);\n"
    "  vec3 res;\n"
    "  if (z >= z_far) {\n"
    "    res = vec3(1, 1, 1);\n"
    "  } else if (z <= z_near) {\n"
    "    res = vec3(0, 0, 0);\n"
    "  } else {\n"
    "    res = PackFloatToVec3i(z_norm);\n"
    "  }\n"
    "  color_out = vec4(res,  1.);"
    "}\n";

}  // namespace shaders
}  // namespace sensors
}  // namespace systems
}  // namespace drake
