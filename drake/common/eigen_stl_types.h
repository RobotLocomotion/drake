#pragma once

/// @file
/// This file contains definitions for using Eigen with the STL.
/// @see eigen_types.h

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace drake {

/// A std::unordered_map that uses Eigen::aligned_allocator so that the
/// contained types may be fixed-size Eigen values.
template <typename Key, typename T>
using eigen_aligned_std_unordered_map =
    std::unordered_map<Key, T, std::hash<Key>, std::equal_to<Key>,
                       Eigen::aligned_allocator<std::pair<Key const, T>>>;

/// A std::vector that uses Eigen::aligned_allocator so that the contained
/// types may be fixed-size Eigen values.
template <typename T>
using eigen_aligned_std_vector = std::vector<T, Eigen::aligned_allocator<T>>;

}  // namespace drake
