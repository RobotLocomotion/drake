classdef QPControllerPlan
  properties
    % The support logic maps make it possible for the planner
    % to specify the support state used by the controller, based
    % on the controller's instantaneous force and kinematic input. 
    % See QPInput2D.m for a more complete description. 
    support_logic_maps = struct('require_support', true(4,1),...
                                'only_if_force_sensed', logical([0;0;1;1]),...
                                'only_if_kinematic', logical([0;1;0;1]),...
                                'kinematic_or_sensed', logical([0;1;1;1]),...
                                'prevent_support', false(4,1));
    end_time;
    start_time = 0;
    default_qp_input = atlasControllers.QPInput2D;
  end

  methods(Abstract)
    qp_input = getQPControllerInput(obj, t, x)
  end

  methods
    function next_plan = getSuccessor(obj, t, x)
      next_plan = obj;
      next_plan.end_time = inf;
    end
  end
end