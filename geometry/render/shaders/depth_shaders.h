#pragma once

namespace drake {
namespace geometry {
namespace render {
namespace shaders {

// NOTE: The _original_ vertex shader computed the camera-space normal and
// propagated the texture coordinate to the fragment shader. These quantities
// are *not* used and represented wasted computation. However, in the future,
// these *could* be used to handle more complex depth computations based on how
// much of a structured IR pattern is fed back to the camera. For example,
// a surface with a normal perpendicular to the view direction wouldn't reflect
// anything back and a surface with varying reflectance properties would be
// captured as a texture. To simplify the inclusion of this data when we're
// ready, the requisite shader code has been _commented out_ below for future
// reference.
// TODO(SeanCurtis-TRI): Re-enable providing normals and texture coordinates
//  to the fragment shader when these quantities are used.

// NOTE: For the VTK infrastructure, the shader should always start with the
// line:
//   //VTK::System::Dec
/** A vertex shader program for rendering depth images, which computes vertices
 and normals for the fragment shader program coming after. */
constexpr char kDepthVS[] = R"__(
    //VTK::System::Dec
    attribute vec4 vertexMC;
    // attribute vec3 normalMC;
    // uniform mat3 normalMatrix;
    uniform mat4 MCDCMatrix;
    uniform mat4 MCVCMatrix;
    // varying vec3 normalVCVSOutput;
    varying vec4 vertexVCVSOutput;
    // attribute vec2 tcoordMC;
    // varying vec2 tcoordVCVSOutput;
    void main () {
      // normalVCVSOutput = normalMatrix * normalMC;
      // tcoordVCVSOutput = tcoordMC;
      vertexVCVSOutput = MCVCMatrix * vertexMC;
      gl_Position = MCDCMatrix * vertexMC;
    }
)__";

// TODO(SeanCurtis-TRI): Investigate rendering directly to a one-channel,
//  32-bit float image so that the encoding isn't necessary.

// TODO(SeanCurtis-TRI): This is a 24-bit depth value. We're losing precision.
//  Determine if that loss of precision is significant.

// NOTE: For the VTK infrastructure, the shader should always start with the
// lines:
//   //VTK::System::Dec
//   //VTK::Output::Dec
/** A fragment shader program for rendering depth images, which computes depth
 values for each pixel in depth images, converts them to be in range [0, 1]
 and packs those values to three color channels. In other words, we encode
 a depth image into a color image and output the color image. For the detail
 of packing algorithm, please refer to:
 https://aras-p.info/blog/2009/07/30/encoding-floats-to-rgba-the-final/
 Note that we are only using three channels instead of four channels to
 express a float value. This differs from the example code in the link above.
 The reason is that we need to set one to alpha channel so that the rendered
 "image" will be opaque. Otherwise, we will have different colors from what
 we output here, thus expect, in the end.  */
constexpr char kDepthFS[] = R"__(
    //VTK::System::Dec
    //VTK::Output::Dec
    varying vec3 normalVCVSOutput;
    varying vec4 vertexVCVSOutput;
    varying vec2 tcoordVCVSOutput;
    out vec4 color_out;
    uniform float z_near;
    uniform float z_far;

    // This function splits a float value, whose range is [0, 1], to three
    // float values, whose ranges are also [0, 1] but will eventually be
    // converted to be [0, 255] of unsigned char. Each of the split float
    // values holds specific decimal portion of the original float value
    // using a bit shift operation, an integer part truncation and a bit
    // mask operation. The maximum amount of information that each of the
    // float value can hold is up to 8 bits which is the size of unsigned
    // char and that is why we use a magic number 255 a lot in this
    // function.
    // Here we give you an example with concrete numbers using the base
    // number 100 instead of 255 just to help you understand better:
    //
    // `value` = 0.123456
    // `bit_shift` = `[1., 100., 10000.]`
    // `bit_mask` = `[0.01, 0.01, 0]`
    //
    // `res` = `fract(value * bit_shift)`
    //       = `fract(0.123456 * [1., 100., 10000.])`
    //       = `fract([0.123456, 12.3456, 1234.56]`
    //       = `[0.123456, 0.3456, 0.56]`
    //
    // `res.yzz` * `bit_mask` = `[0.3456, 0.56, 0.56]` * `[0.01, 0.01, 0]`
    //                        = `[0.003456, 0.0056, 0]`
    //
    // `return` = `res` - `res.yzz` * `bit_mask`
    //          = `[0.123456, 0.3456, 0.56]` - `[0.003456, 0.0056, 0]`
    //          = `[0.12, 0.34, 0.56]`.
    //
    // To decode this value, you will simply need to calculate the reverse:
    //
    // i.e. `decoded = 0.12 + 0.34 * 0.01 + 0.56 * 0.0001`
    //      `        = 0.12 + 0.0034 + 0.000056`
    //      `        = 0.123456`.
    vec3 PackFloatToVec3i(float value) {
      const vec3 bit_shift = vec3(1., 255., 255. * 255.);
      const float tmp = 1. / 255.;
      const vec3 bit_mask = vec3(tmp, tmp, 0.);
      vec3 res = fract(value * bit_shift);
      return res - (res.yzz * bit_mask);
    }

    void main () {
      // NOTE: This isn't the distance to the camera, but the distance to the
      // plane that is parallel with the camera's image plane on which the
      // corresponding point lies.
      float z = -vertexVCVSOutput.z;  // In meters.
      // Converting meters to [0, 1].
      float z_norm = (z - z_near) / (z_far - z_near);
      vec3 res;
      if (z >= z_far) {
        res = vec3(1, 1, 1);
      } else if (z <= z_near) {
        res = vec3(0, 0, 0);
      } else {
        res = PackFloatToVec3i(z_norm);
      }
      color_out = vec4(res, 1.);
    }
)__";

}  // namespace shaders
}  // namespace render
}  // namespace geometry
}  // namespace drake
