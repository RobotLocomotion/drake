classdef FrozenPlan < QPControllerPlanMatlabImplementation
  properties
    frozen_qp_input;
  end

  methods
    function obj = FrozenPlan(qp_input)
      obj.frozen_qp_input = qp_input;
      obj.duration_ = inf;
    end

    function qp_input = getQPControllerInput(obj, varargin)
      qp_input = obj.frozen_qp_input;
    end
  end
end

