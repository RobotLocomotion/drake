classdef ActionSequence
  % structure which lists a series of kinematic (and possibly dynamic)
  % objectives and constraints for a robot
  
  properties (Access=protected)
    %% Objectives
    
    %% Constraints
    kincon = {};  % a cell array of ActionKinematicConstraints
    tspan=[inf, -inf];
  end
  
  methods
    function obj=addKinematicConstraint(obj,kc)
      typecheck(kc,'ActionKinematicConstraint');
      obj.kincon = horzcat(obj.kincon,kc);
      obj.tspan(1) = min(obj.tspan(1),kc.tspan(1));
      obj.tspan(2) = min(obj.tspan(2),kc.tspan(2));
    end
    
    function ikargs = getIKArguments(obj,t)
      ikargs={};
      for i=1:length(kincon)
        ikargs=horzcat(ikargs,getIKArguments(kincon{i},t));
      end
    end
  end
  
end

% NORELEASE
  