#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>

#include <Eigen/Dense>

/**
 * A map from the name of a material to its color. The color is specified in
 * RGBA (Red, Green, Blue, Alpha) format.
 *
 * TODO(liang.fok) Add support for texture-based materials, see #2588.
 */
typedef std::map<std::string, Eigen::Vector4d, std::less<std::string>,
                 Eigen::aligned_allocator<
                     std::pair<const std::string,
                               Eigen::Vector4d>>> MaterialMap;
