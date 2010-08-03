classdef FunctionHandleTrajectory < Trajectory
  
  properties
    handle
    dhandle
    breaks
  end
  
  methods
    function obj = FunctionHandleTrajectory(handle,dim,breaks,dhandle)
      if (nargin>0)
        if (~isa(handle,'function_handle')) error('handle should be a function handle'); end
        if nargin < 4, dhandle = [];
        elseif (~isa(dhandle,'function_handle')) error('dhandle should be a function handle'); end
        obj.handle=handle;
        obj.dhandle=dhandle;
        obj.breaks = breaks;
        obj.dim = dim;
        obj.tspan = [min(breaks), max(breaks)];
      end
    end
    
    function y = eval(obj,t)
      y = feval(obj.handle,t);
    end
    function ydot = deriv(obj,t)
      ydot = feval(obj.dhandle,t);
    end
    function t = getBreaks(obj)
      t = obj.breaks;
    end
  end
end