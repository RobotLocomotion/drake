classdef FunctionHandleTrajectory < Trajectory
% Constructs a proper trajectory object from a function handle.
  
  properties
    handle  % the function handle
    dhandle % a handle to the derivative of the function
    breaks  % a list of breakpoints to indicate where the function handle should be considered accurate/reliable
  end
  
  methods
    function obj = FunctionHandleTrajectory(handle,dim,breaks,dhandle)
    % Constructs a trajectory from the function handle, and optionally its derivative.
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
    % trajectory eval = function handle feval
      y = feval(obj.handle,t);
    end
    function ydot = deriv(obj,t)
    % evaluate the derivative handle
      if (isempty(obj.dhandle)) error('the derivative was not defined for this handle'); end
      ydot = feval(obj.dhandle,t);
    end
    function t = getBreaks(obj)
    % return the list of accurate/reliable points
      t = obj.breaks;
    end
  end
end