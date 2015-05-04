classdef QPLocomotionPlanCPPWrapper < QPControllerPlan
  properties (SetAccess = protected)
    qp_locomotion_plan_ptr
  end

  methods
    function obj = QPLocomotionPlanCPPWrapper(settings)
      model_ptr = settings.robot.getManipulator().mex_model_ptr;
      channel = 'qp_controller_input';
      obj.qp_locomotion_plan_ptr = constructQPLocomotionPlanmex(model_ptr, settings, channel);
    end

    function next_plan = getSuccessor(obj, t, x)
      next_plan = obj;
      next_plan.setDuration(inf);
    end

    function is_finished = isFinished(obj, t, x)
      if isempty(obj.start_time)
        is_finished = false;
      else
        is_finished = t - obj.start_time >= obj.duration;
      end
    end
    
    qp_input = getQPControllerInput(obj, t_global, x, contact_force_detected, rpc_dummy)
    ret = duration(obj)
    ret = start_time(obj)
  end
end
