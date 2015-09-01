%module bot_visualizer_wrapper

%include <eigen.i>
%import "systems.i"
%import "lcm.i"

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "BotVisualizer.h"
%}

%shared_ptr(BotVisualizer)