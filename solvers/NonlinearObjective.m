classdef NonlinearObjective < NonlinearConstraint
% The Drake optimization classes treat objectives as constraints.  
% This class simply provides an alternative name for the 
% NonlinearConstraint class, to aid in readability of the code.
  
  methods
    function obj = NonlinearObjective(varargin)
      % note: would it make sense to enforce lb=-inf and ub=inf?
      obj = obj@NonlinearConstraint(varargin{:});
    end
  end
end
