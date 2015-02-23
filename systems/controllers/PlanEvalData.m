classdef PlanEvalData < ControllerData
  properties
    available_plans
    plan_id_queue
    default_plan_id
  end

  methods
    function obj = PlanEvalData(data)
      typecheck(data, 'struct');
      obj = obj@ControllerData(data);
    end

    function data = verifyControllerData(~,data)
      warning('not implemented yet');
    end

    function updateControllerData(obj, data)
      updateControllerData@ControllerData(obj,data);
    end
  end
end
