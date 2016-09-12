#pragma once

/// @file
/// This file contains definitions for using Eigen with the STL.
/// @see eigen_types.h

#include <unordered_map>

#include <Eigen/Core>

namespace drake {

/// A std::unordered_map that uses Eigen::aligned_allocator, so that the
/// contained types may be fixed-size Eigen values.
template <typename Key, typename T>
using eigen_aligned_std_unordered_map =
    std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>,
                       Eigen::aligned_allocator<std::pair<Key const, T>>>;

}  // namespace drake
