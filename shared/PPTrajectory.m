classdef PPTrajectory < Trajectory
  
  properties
    pp
  end
  
  methods
    function obj = PPTrajectory(ppform)
      obj.pp = ppform;
      obj.dim=ppform.dim;
      obj.tspan = [min(obj.pp.breaks) max(obj.pp.breaks)];
    end
    function ydot = deriv(obj,t)
      ydot = ppvalSafe(fnder(obj.pp),t);
    end

    function y = eval(obj,t)
      y = ppvalSafe(obj.pp,t);
    end

    function dtraj = fnder(obj)
      dtraj = PPTrajectory(fnder(obj.pp));
    end
    
    function t = getBreaks(obj)
      t = obj.pp.breaks;
    end
  end
end
