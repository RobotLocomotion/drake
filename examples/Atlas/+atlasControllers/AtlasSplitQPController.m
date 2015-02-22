classdef AtlasSplitQPController < DrakeSystem
  properties
    control
    plan_eval;
  end

  methods
    function obj = AtlasSplitQPController(r, control, plan_eval)
      input_frame = r.getStateFrame();
      output_frame = r.getInputFrame();
      obj = obj@DrakeSystem(0,0,numel(input_frame),numel(output_frame),true,true);
      obj = obj.setInputFrame(input_frame);
      obj = obj.setOutputFrame(output_frame);
      obj.control = control;
      obj.plan_eval = plan_eval;
    end

    function y = output(obj, t, ~, x)
      qp_input = obj.plan_eval.tick(t, x);
      y = obj.control.tick(t, x, qp_input, [-1;-1]);
    end
  end
end
