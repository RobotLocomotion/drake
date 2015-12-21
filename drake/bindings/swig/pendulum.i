%module(package="pydrake.examples") pendulum

%include <eigen.i>
%import "systems.i"
%import "botvis.i"

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "Pendulum.h"
%}

%shared_ptr(PendulumWithBotVis)
%shared_ptr(PendulumEnergyShaping)
%shared_ptr(Pendulum)

%include "Pendulum.h"
