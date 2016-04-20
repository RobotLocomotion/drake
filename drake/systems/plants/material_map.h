#pragma once

#include <string>
#include <map>
#include <Eigen/Dense>

typedef std::map<std::string, Eigen::Vector4d, std::less<std::string>,
                 Eigen::aligned_allocator<
                     std::pair<std::string, Eigen::Vector4d> > > MaterialMap;
