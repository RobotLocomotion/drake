classdef AtlasPlanEvalAndController < DrakeSystem
% Neither PlanEval nor PlanlessQPController implements the DrakeSystem
% interface. Instead, we wrap a PlanEval and a PlanlessQPController inside
% this class, which behaves as a drake system by taking in state, calling the
% PlanEval and Controller in order, and outputting the atlasInput. In
% addition, you can also chose to omit the PlanEval or Controller, in which
% case the missing data will be sent or recieved through LCM in the
% background. That might look something like the following:
% 
% sys1 = AtlasPlanEvalAndController(r, [], planEval);
% sys2 = AtlasPlanEvalAndController(r, control, []);
% plancontroller = cascade(sys1, sys2);
% sys = feedback(r, plancontroller);
  properties
    control
    plan_eval;
    options;
    lc
    monitor
  end

  methods
    function obj = AtlasPlanEvalAndController(r, control, plan_eval, options)
      checkDependency('lcmgl');
      if ~isempty(control), typecheck(control, 'atlasControllers.PlanlessQPController'); end
      if ~isempty(control), typecheck(plan_eval, 'atlasControllers.AtlasPlanEval'); end
      if nargin < 4
        options = struct();
      end
      options = applyDefaults(options, struct('debug_lcm', false));
      
      input_frame = r.getStateFrame();
      if isempty(control)
        output_frame = input_frame;
      else
        output_frame = r.getInputFrame();
      end
      obj = obj@DrakeSystem(0,0,numel(input_frame.coordinates),numel(output_frame.coordinates),true,true);
      obj = obj.setInputFrame(input_frame);
      obj = obj.setOutputFrame(output_frame);
      obj.control = control;
      obj.plan_eval = plan_eval;
      obj.options = options;

      if isempty(obj.plan_eval) || isempty(obj.control)
        obj.lc = lcm.lcm.LCM.getSingleton();
        if isempty(obj.plan_eval)
          obj.monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input, 'timestamp');
          obj.lc.subscribe('QP_CONTROLLER_INPUT', obj.monitor);
        end
      end
    end

    function y = output(obj, t, ~, x)
      if ~isempty(obj.plan_eval)
        t0 = tic();
        qp_input = obj.plan_eval.getQPControllerInput(t, x);
        ptime = toc(t0);
        fprintf(1, 'plan eval: %f, ', ptime);
      else
        t0 = tic();
        qp_input = [];
        while isempty(qp_input)
          qp_input = obj.monitor.getMessage();
        end
        qp_input = drake.lcmt_qp_controller_input(qp_input);
        lcm_time = toc(t0);
        fprintf(1, 'lcm receive: %f, ', lcm_time);
      end

      if ~isempty(obj.control)
        t0 = tic();
        [y, v_ref] = obj.control.updateAndOutput(t, x, qp_input, [-1;-1]);
        ctime = toc(t0);
        fprintf(1, 'control: %f\n', ctime);
      else
        t0 = tic();
        encodeQPInputLCMMex(qp_input);
        lcm_time = toc(t0);
        y = x;
        fprintf(1, 'lcm_serialize: %f, ', lcm_time);
      end
    end
  end
end
