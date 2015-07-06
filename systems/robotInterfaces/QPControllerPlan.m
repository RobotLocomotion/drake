classdef QPControllerPlan < handle
  properties
    % The support logic maps make it possible for the planner
    % to specify the support state used by the controller, based
    % on the controller's instantaneous force and kinematic input. 
    % See QPInputConstantHeight.m for a more complete description. 
    duration = inf;
    start_time = 0;
    default_qp_input = atlasControllers.QPInputConstantHeight;
    gain_set = 'standing';
  end

  properties(Constant)
    support_logic_maps = struct('require_support', ones(4,1),...
                                'only_if_force_sensed', [0;0;1;1],...
                                'only_if_kinematic', [0;1;0;1],...
                                'kinematic_or_sensed', [0;1;1;1],...
                                'prevent_support', zeros(4,1));
  end

  methods(Abstract)
    qp_input = getQPControllerInput(obj, t, x)
  end

  methods
    function next_plan = getSuccessor(obj, t, x)
      next_plan = obj;
      next_plan.duration = inf;
    end

    function is_finished = isFinished(obj, t, x)
      if isempty(obj.start_time)
        is_finished = false;
      else
        is_finished = t - obj.start_time >= obj.duration;
      end
    end
  end
end