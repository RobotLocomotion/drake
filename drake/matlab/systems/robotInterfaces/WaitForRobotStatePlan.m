classdef WaitForRobotStatePlan < QPControllerPlanMatlabImplementation
  methods
    function qp_input = getQPControllerInput(obj,varargin) 
      qp_input = [];
    end

    function is_finished = isFinished(obj, t, x)
      is_finished = ~isempty(x);
    end
  end
end
