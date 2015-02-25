classdef PlanEvalData < ControllerData
  properties
    plan_queue
  end

  methods
    function obj = PlanEvalData()
      obj = obj@ControllerData(struct('plan_queue', {{}}));
    end

    function verifyControllerData(obj, data)
      warning('not implemented yet');
    end
  end
end
