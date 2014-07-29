classdef FunctionHandleObjective < FunctionHandleConstraint
% The Drake optimization classes treat objectives as constraints.
% This class simply provides an alternative name for the
% FunctionHandleConstraint class, to aid in readability of the code.

  methods
    function obj = FunctionHandleObjective(xdim,eval_handle,grad_level)
      if nargin<3, grad_level = -1; end
      obj = obj@FunctionHandleConstraint(-inf,inf,xdim,eval_handle,grad_level);
    end
  end
end
