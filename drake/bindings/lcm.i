%module(package="drake.wrappers") lcm

%include <std_shared_ptr.i>

%{
#include <memory>
#include <lcm/lcm-cpp.hpp>
%}

%shared_ptr(lcm::LCM)

%include <lcm/lcm-cpp.hpp>