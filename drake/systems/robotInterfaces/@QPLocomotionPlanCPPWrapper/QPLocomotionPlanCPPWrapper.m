classdef QPLocomotionPlanCPPWrapper < QPControllerPlan
  properties (SetAccess = protected)
    qp_locomotion_plan_ptr
  end

  properties (SetAccess = private, GetAccess = public)
    settings % store the settings that were used to construct the c++ object for later inspection
  end

  methods
    function obj = QPLocomotionPlanCPPWrapper(settings)
      model_ptr = settings.robot.getManipulator().mex_model_ptr;
      channel = 'qp_controller_input';
      obj.qp_locomotion_plan_ptr = constructQPLocomotionPlanmex(model_ptr, settings, channel);
      obj.settings = settings;
    end

    function next_plan = getSuccessor(obj, t, x)
      next_plan = FrozenPlan(drake.lcmt_qp_controller_input(obj.getLastQPInput()));
      next_plan.setStartTime(t);
    end

    ret = isFinished(obj, t, x);
    qp_input = getQPControllerInput(obj, t_global, x, contact_force_detected)
    ret = duration(obj)
    ret = start_time(obj)
    setStartTime(obj, t)
    qp_input = getLastQPInput(obj)
  end
end
