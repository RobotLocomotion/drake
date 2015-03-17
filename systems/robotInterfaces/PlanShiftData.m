classdef PlanShiftData < ControllerData
  properties
    plan_shift
  end

  methods
    function obj = PlanShiftData()
      obj = obj@ControllerData(struct('plan_shift', zeros(6,1)));
    end

    function verifyControllerData(obj, data)
      sizecheck(data.plan_shift, [6, 1]);
    end
  end
end
