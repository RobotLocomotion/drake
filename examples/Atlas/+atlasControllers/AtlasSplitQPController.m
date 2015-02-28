classdef AtlasSplitQPController < DrakeSystem
  properties
    control
    plan_eval;
    lc
    monitor
    debug_monitor
  end

  methods
    function obj = AtlasSplitQPController(r, control, plan_eval)
      checkDependency('lcmgl');
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

      obj.lc = lcm.lcm.LCM.getSingleton();
      if isempty(obj.plan_eval) || isempty(obj.control)
        if isempty(obj.plan_eval)
          obj.monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input, 'timestamp');
          obj.debug_monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input, 'timestamp');
          obj.lc.subscribe('QP_CONTROLLER_INPUT', obj.monitor);
          obj.lc.subscribe('QP_CONTROLLER_INPUT_DEBUG', obj.debug_monitor);
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

        qp_input_debug = [];
        while isempty(qp_input_debug)
          qp_input_debug = obj.debug_monitor.getMessage();
        end
        qp_input_debug = drake.lcmt_qp_controller_input(qp_input_debug);

        try
          compareLCMMsgs(qp_input, qp_input_debug);
        catch e
          e.getReport()
          keyboard();
        end

        lcm_time = toc(t0);
        fprintf(1, 'lcm receive: %f, ', lcm_time);
      end

      if ~isempty(obj.control)
        t0 = tic();
        [y, v_ref] = obj.control.tick(t, x, qp_input, [-1;-1]);
        ctime = toc(t0);
        fprintf(1, 'control: %f\n', ctime);
      else
        t0 = tic();
        encodeQPInputLCMMex(qp_input);
        qp_input_msg = qp_input.to_lcm();
        obj.lc.publish('QP_CONTROLLER_INPUT_DEBUG', qp_input_msg);
        lcm_time = toc(t0);
        y = x;
        fprintf(1, 'lcm_serialize: %f, ', lcm_time);
      end
    end
  end
end
