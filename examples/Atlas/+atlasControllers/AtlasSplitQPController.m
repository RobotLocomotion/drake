classdef AtlasSplitQPController < DrakeSystem
  properties
    control
    plan_eval;
  end

  methods
    function obj = AtlasSplitQPController(r, control, plan_eval)
      checkDependency('lcmgl');
      input_frame = r.getStateFrame();
      output_frame = r.getInputFrame();
      obj = obj@DrakeSystem(0,0,numel(input_frame.coordinates),numel(output_frame.coordinates),true,true);
      obj = obj.setInputFrame(input_frame);
      obj = obj.setOutputFrame(output_frame);
      obj.control = control;
      obj.plan_eval = plan_eval;
    end

    function y = output(obj, t, ~, x)
      t0 = tic();
      qp_input = obj.plan_eval.tick(t, x);
      ptime = toc(t0);
      t0 = tic();
      y = obj.control.tick(t, x, qp_input, [-1;-1]);
      ctime = toc(t0);
%       fprintf(1, 'plan eval: %f, control: %f\n', ptime, ctime);
    end
  end
end
