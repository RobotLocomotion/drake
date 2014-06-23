classdef NonlinearObjective < NonlinearConstraint
% The Drake optimization classes treat objectives as constraints.  
% This class simply provides an alternative name for the 
% NonlinearConstraint class, to aid in readability of the code.
  
  methods
    function obj = NonlinearObjective(xdim,eval_handle)
      obj = obj@NonlinearConstraint(-inf,inf,xdim,eval_handle);
    end
  end
end
