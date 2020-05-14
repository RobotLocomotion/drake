#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/attic_warning.h"

typedef std::map<
    std::string, Eigen::Isometry3d, std::less<std::string>,
    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > >
    PoseMap;
