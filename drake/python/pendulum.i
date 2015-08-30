%module pendulum_wrapper
%import "systems.i"

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "Pendulum.h"
#include <numpy/arrayobject.h>
%}

%shared_ptr(lcm::LCM)
%shared_ptr(PendulumWithBotVis)
%shared_ptr(PendulumEnergyShaping)
%shared_ptr(BotVisualizer)
%shared_ptr(Pendulum)

%include <lcm/lcm-cpp.hpp>
%include "Pendulum.h"
