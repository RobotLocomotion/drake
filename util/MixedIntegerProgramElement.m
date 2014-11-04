classdef MixedIntegerProgramElement
  properties
  end

  methods(Abstract=true)
    function [constraints, objective] = getYalmipForm(obj)
    end

    function helper = addToHelper(obj, helper)
    end
  end
end
