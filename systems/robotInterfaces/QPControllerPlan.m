classdef QPControllerPlan
  properties
    support_logic_maps = struct('require_support', ones(4,1),...
                                'only_if_force_sensed', [0;0;1;1],...
                                'only_if_kinematic', [0;1;0;1],...
                                'prevent_support', zeros(4,1));
    end_time;
    start_time = 0;
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