classdef QPControllerPlan
  properties
    % The support logic maps make it possible for the planner
    % to specify the support state used by the controller, based
    % on the controller's instantaneous force and kinematic input. 
    % See QPInputConstantHeight.m for a more complete description. 
    support_logic_maps = struct('require_support', ones(4,1),...
                                'only_if_force_sensed', [0;0;1;1],...
                                'only_if_kinematic', [0;1;0;1],...
                                'kinematic_or_sensed', [0;1;1;1],...
                                'prevent_support', zeros(4,1));
    duration;
    start_time = 0;
    default_qp_input = atlasControllers.QPInputConstantHeight;
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