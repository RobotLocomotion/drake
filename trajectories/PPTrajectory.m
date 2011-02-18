classdef PPTrajectory < Trajectory
  
  properties
    pp
  end
  
  methods
    function obj = PPTrajectory(ppform)
      obj = obj@Trajectory(ppform.dim);
      obj.pp = ppform;
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
    
    function obj = shiftTime(obj,offset)
      typecheck(offset,'double');
      sizecheck(offset,[1 1]);
      obj.tspan = obj.tspan + offset;
      obj.pp.breaks = obj.pp.breaks + offset;
    end
    
    function t = getBreaks(obj)
      t = obj.pp.breaks;
    end
    
    % should getParameters and setParameters include the breaks? or just
    % the actual coefficients?  
  end
end
