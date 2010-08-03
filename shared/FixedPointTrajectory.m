classdef FixedPointTrajectory < Trajectory

  properties
    pt
  end
  
  methods 
    function obj = FixedPointTrajectory(pt)
      if (nargin>0)
        obj.pt=pt;
        obj.tspan = [-inf,inf];
        obj.dim = length(pt);
      end
    end
    
    function dtraj = fnder(obj)
      dtraj = FixedPointTrajectory(0*obj.pt);
    end
    
    function y = eval(obj,t)
      y = obj.pt;
    end
    
    function t = getBreaks(obj)
      t = 0;
    end
  end
end
