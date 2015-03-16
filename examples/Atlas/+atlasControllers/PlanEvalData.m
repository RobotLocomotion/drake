classdef PlanEvalData < ControllerData
% Container class for the PlanEval's queue of plans. This class is necessary
% in order to make the plan queue a handle object, in order to make it
% persistent across multiple calls to PlanEval.getCurrentPlan
  properties
    plan_queue
  end

  methods
    function obj = PlanEvalData()
      obj = obj@ControllerData(struct('plan_queue', {{}}));
    end

    function verifyControllerData(obj, data)
      for j = 1:length(data.plan_queue)
        typecheck(data.plan_queue{j}, 'QPControllerPlan');
      end
    end
  end
end
