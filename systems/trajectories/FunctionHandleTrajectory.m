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
      warning('Drake:FunctionHandleTrajectory','creating function handle.  this is potentially inefficient.  consider implementing things a different way');
      obj = obj@Trajectory(dim);
      if (nargin>0)
        if (~isa(handle,'function_handle')) error('handle should be a function handle'); end
        if nargin < 4, dhandle = [];
        elseif (~isa(dhandle,'function_handle')) error('dhandle should be a function handle'); end
        obj.handle=handle;
        obj.dhandle=dhandle;
        obj.breaks = breaks;
        dim = dim(1:find(dim~=1,1,'last')); % strip trailing 1's
        if (isempty(dim)) dim=1; end  
        obj.dim = dim;
        obj.tspan = [min(breaks), max(breaks)];
      end
    end
    
    function y = eval(obj,t)
    % trajectory eval = function handle feval
      if (length(t)>1)  
        n=prod(obj.dim);
        for i=1:length(t)
          y(:,i)=reshape(feval(obj.handle,t(i)),n,1);
        end
        y=reshape(y,[obj.dim,length(t)]);
      else
        y = feval(obj.handle,t);
      end
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
    
    function dtraj = fnder(obj)
      if (isempty(obj.dhandle)) error('the derivative was not defined for this handle'); end
      dtraj = FunctionHandleTrajectory(obj.dhandle,obj.dim,obj.breaks);
    end
    
    function pp = flipToPP(obj,ppbuilder)
      if (nargin<2) 
        if isempty(obj.dhandle) ppbuilder=@foh;
        else
          y=eval(obj,obj.breaks);
          ydot=deriv(obj,obj.breaks);
          pp=PPTrajectory(pchipDeriv(obj.breaks,y,ydot));
          return;
        end
      end
      y=eval(obj,obj.breaks);
      pp=PPTrajectory(ppbuilder(obj.breaks,y));
    end
  end
end
