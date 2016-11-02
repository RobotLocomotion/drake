#pragma once

/// @file
/// This file contains definitions for using Eigen with the STL.
/// See http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html.
/// @see eigen_types.h

#include <functional>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "drake/common/drake_gcc48.h"

namespace drake {

/// A std::map that uses Eigen::aligned_allocator so that the
/// contained types may be fixed-size Eigen values.
template <typename Key, typename T>
using eigen_aligned_std_map =
    std::map<Key, T, std::less<Key>,
             Eigen::aligned_allocator<std::pair<Key const, T>>>;

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
