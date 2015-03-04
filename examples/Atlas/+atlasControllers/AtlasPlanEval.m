classdef AtlasPlanEval < PlanEval
  properties  
    robot_property_cache
    robot
  end

  methods
    function obj = AtlasPlanEval(r, varargin)
      obj = obj@PlanEval(varargin{:});
      obj.robot = r;

      obj.robot_property_cache = atlasUtil.propertyCache(obj.robot);
    end

    function qp_input = getQPControllerInput(obj, t, x)
      plan = obj.getCurrentPlan(t, x);
      qp_input = plan.getQPControllerInput(t, x, obj.robot_property_cache);
    end
  end
end
