classdef AtlasPlanEval < PlanEval
% A PlanEval which includes some Atlas-specific tweaks. Specifically, it
% provides a cache of useful robot properties which may be expensive to look
% up at runtime.
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

    function qp_input = getQPControllerInput(obj, t, x, contact_force_detected)
      if nargin < 4
        contact_force_detected = zeros(obj.robot_property_cache.num_bodies, 1);
      end
      plan = obj.getCurrentPlan(t, x);
      qp_input = plan.getQPControllerInput(t, x, obj.robot_property_cache, contact_force_detected);
    end
  end
end
