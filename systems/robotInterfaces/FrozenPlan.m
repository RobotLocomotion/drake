classdef FrozenPlan < QPControllerPlan
  properties
    frozen_qp_input;
  end

  methods
    function obj = FrozenPlan(qp_input)
      obj.frozen_qp_input = qp_input;
      obj.duration = inf;
    end

    function qp_input = getQPControllerInput(obj, varargin)
      qp_input = obj.frozen_qp_input;
    end
  end
end

