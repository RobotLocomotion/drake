classdef AtlasPlanEval < PlanEval
% A PlanEval which includes some Atlas-specific tweaks. Specifically, it
% provides a cache of useful robot properties which may be expensive to look
% up at runtime.
  properties (Access = protected)
    robot
  end

  methods
    function obj = AtlasPlanEval(r, varargin)
      obj = obj@PlanEval(varargin{:});
      obj.robot = r;
    end

    function qp_input = getQPControllerInput(obj, t, x, contact_force_detected)
      if nargin < 4
        contact_force_detected = zeros(obj.robot.getNumBodies(), 1);
      end
      plan = obj.getCurrentPlan(t, x);
      qp_input = plan.getQPControllerInput(t, x, contact_force_detected);
    end
  end
end
