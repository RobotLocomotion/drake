classdef PlanEval
  properties
    data
  end

  methods
    function obj = PlanEval(plans)
      obj.data = atlasControllers.PlanEvalData();
      if nargin > 1
        if iscell(plans)
          for j = 1:length(plans)
            obj = obj.appendPlan(plans{j});
          end
        else
          obj = obj.appendPlan(plans);
        end
      end
    end

    function current_plan = getCurrentPlan(obj)
      current_plan = obj.data.plan_queue{1};
      while t > current_plan.end_time
        if length(obj.data.plan_queue) == 1
          obj.data.plan_queue{1} = current_plan.getSuccessor(t, x);
        else
          obj.data.plan_queue(1) = [];
          obj.data.plan_queue{1}.start_time = t;
          current_plan = obj.data.plan_queue{1};
        end
      end
    end

    function qp_input = getQPControllerInput(obj, t, x)
      current_plan = obj.getCurrentPlan();
      qp_input = current_plan.getQPControllerInput(t, x);
    end

    function obj = insertPlan(obj, new_plan, idx)
      if nargin < 3
        idx = 1;
      end
      if idx < 1 
        error('idx must be >= 1');
      end
      if idx > length(obj.data.plan_queue)
        error('Cannot insert a plan after the end of the queue. Ignoring this request.');
      end
      obj.data.plan_queue = [obj.data.plan_queue(1:(idx-1)), {new_plan}, obj.data.plan_queue(idx:end)];
    end

    function obj = appendPlan(obj, new_plan)
      obj.data.plan_queue{end+1} = new_plan;
    end

    function obj = switchToPlan(obj, new_plan)
      obj.data.plan_queue = {new_plan};
    end
  end
end

