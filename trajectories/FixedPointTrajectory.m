classdef FixedPointTrajectory < Trajectory
% Trivial instance of a trajectory as a fixed point.
  
  properties
    pt
  end
  
  methods 
    function obj = FixedPointTrajectory(pt)
    % Construct FixedPointTrajectory from a pt
      if (nargin>0)
        obj.pt=pt;
        obj.tspan = [-inf,inf];
        obj.dim = length(pt);
      end
    end
    
    function dtraj = fnder(obj)
    % Implements the (trivial) derivative.
      dtraj = FixedPointTrajectory(0*obj.pt);
    end
    
    function y = eval(obj,t)
    % Return the fixed point for all t.
      y = obj.pt;
    end
    
    function t = getBreaks(obj)
    % Return a single break (at t=0).
      t = 0;
    end
  end
end
