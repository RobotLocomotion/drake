classdef ODESolTrajectory < Trajectory
  
  properties
    sol=[];
    shape;
  end
  
  methods 
    function obj = ODESolTrajectory(sol,shape)
      obj = obj@Trajectory(size(sol.y,1));
      if (nargin>0)
        obj.sol = sol;
        if (nargin>1)
          for i=1:length(shape)
            obj.shape{i}=shape(i);
          end
        end
        obj.tspan = [min(obj.sol.x), max(obj.sol.x)];
      end
    end
    
    function y = eval(obj,t)
      if (any(t<obj.tspan(1)) || any(t>obj.tspan(end))) 
        error('outside interval'); 
      end
      y = deval(obj.sol,t);
      if (~isempty(obj.shape)) y = reshape(y,obj.shape{:},[]); end
    end
    function ydot = deriv(obj,t)
        ydot = obj.sol.extdata.odefun(t,deval(obj.sol,t));
        if (~isempty(obj.shape)) ydot = reshape(ydot,obj.shape{:},[]); end
    end
    
    function t = getBreaks(obj)
      if (obj.sol.x(end)<obj.sol.x(1)) % then it was solved backwards in time.  ok to reverse
        t = obj.sol.x(end:-1:1);
      else
        t = obj.sol.x;
      end
    end
  end
  
end
