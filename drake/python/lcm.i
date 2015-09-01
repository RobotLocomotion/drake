%module lcm_wrapper

%include <std_shared_ptr.i>

%{
#include <lcm/lcm-cpp.hpp>
%}

%shared_ptr(lcm::LCM)

%include <lcm/lcm-cpp.hpp>