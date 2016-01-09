classdef BipedPlanEval < PlanEval
% A PlanEval which includes some biped-specific tweaks. Specifically, it
% calls its plans' getQPControllerInput method with an additional argument
% (contact_force_detected)

  properties (Access = protected)
    robot
  end

  methods
    function obj = BipedPlanEval(r, varargin)
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
