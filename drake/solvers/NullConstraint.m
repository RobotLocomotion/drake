classdef NullConstraint < Constraint
% this constraint does nothing, but having it is easier that handling the
% possibility of empty matrices in lists of constraints

  methods
    function obj = NullConstraint(xdim)
      obj = obj@Constraint([],[],xdim,inf);
    end
  end
  
  methods (Access = protected)
    function varargout = constraintEval(obj,varargin)
      varargout = cell(1,nargout);
      % outputs are intentionally left empty
    end
  end
end