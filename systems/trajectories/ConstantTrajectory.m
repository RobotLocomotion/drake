classdef ConstantTrajectory < Trajectory
% Trivial instance of a trajectory as a constant.
  
  properties
    pt
  end
  
  methods 
    function obj = ConstantTrajectory(pt)
    % Construct ConstantTrajectory from a pt
      obj = obj@Trajectory(size(pt));
      obj.pt=pt;
      obj.tspan = [-inf,inf];
    end
    
    function dtraj = fnder(obj)
    % Implements the (trivial) derivative.
      dtraj = ConstantTrajectory(0*obj.pt);
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
