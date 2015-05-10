classdef QPControllerPlan < handle
  properties(Constant)
    support_logic_maps = struct('require_support', ones(4,1),...
                                'only_if_force_sensed', [0;0;1;1],...
                                'only_if_kinematic', [0;1;0;1],...
                                'kinematic_or_sensed', [0;1;1;1],...
                                'prevent_support', zeros(4,1));
  end

  methods(Abstract)
    qp_input = getQPControllerInput(obj, t, x)
    next_plan = getSuccessor(obj, t, x)
    is_finished = isFinished(obj, t, x)
    ret = duration(obj)
    ret = start_time(obj)
  end
end