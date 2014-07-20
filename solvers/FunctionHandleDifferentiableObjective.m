classdef FunctionHandleDifferentiableObjective < FunctionHandleDifferentiableConstraint
% The Drake optimization classes treat objectives as constraints.  
% This class simply provides an alternative name for the 
% FunctionHandleDifferentiableConstraint class, to aid in readability of the code.
  
  methods
    function obj = FunctionHandleDifferentiableObjective(xdim,eval_handle)
      obj = obj@FunctionHandleDifferentiableConstraint(-inf,inf,xdim,eval_handle);
    end
  end
end
