classdef QPControllerPlanMatlabImplementation < QPControllerPlan
  properties (SetAccess = protected)
    duration_ = inf;
    start_time_ = 0;
    default_qp_input_ = atlasControllers.QPInputConstantHeight;
    gain_set_ = 'standing';
  end

  methods
    function next_plan = getSuccessor(obj, t, x)
      next_plan = obj;
      next_plan.duration = inf;
    end

    function is_finished = isFinished(obj, t, x)
      if isempty(obj.start_time)
        is_finished = false;
      else
        is_finished = t - obj.start_time >= obj.duration;
      end
    end
    
    function ret = duration(obj)
      ret = obj.duration_;
    end
    
    function ret = start_time(obj)
      ret = obj.start_time_;
    end
    
    function ret = default_qp_input(obj)
      ret = obj.default_qp_input_;
    end
    
    function ret = gain_set(obj)
      ret = obj.gain_set_;
    end

    function obj = setStartTime(obj, t)
      obj.start_time_ = t;
    end
  end
end