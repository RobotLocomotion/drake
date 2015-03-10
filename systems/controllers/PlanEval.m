classdef PlanEval
  % A PlanEval represents one half of a complete control system. It contains
  % all the stateful information about the currently executed plan, and
  % produces instantaneous input to the QP controller specifying the desired
  % configuration, body positions, and parameters. Plan Eval maintains a queue
  % of plans as additional state. When the current plan ends, it will
  % automatically move to the next plan in the queue. If no additional plans
  % are available, it will instead call current_plan.getSuccessor() to try to
  % get a default successor plan. For example, QPWalkingPlan.getSuccessor()
  % returns a standing plan at the current posture.
  properties
    data
  end

  methods
    function obj = PlanEval(plans)
      obj.data = atlasControllers.PlanEvalData();
      if nargin >= 1
        if iscell(plans)
          for j = 1:length(plans)
            obj = obj.appendPlan(plans{j});
          end
        else
          obj = obj.appendPlan(plans);
        end
      end
    end

    function current_plan = getCurrentPlan(obj, t, x)
      while true
        current_plan = obj.data.plan_queue{1};
        if (t - current_plan.start_time) < current_plan.duration
          break
        end
        if length(obj.data.plan_queue) == 1
          obj.data.plan_queue{1} = current_plan.getSuccessor(t, x);
        else
          obj.data.plan_queue(1) = [];
        end
        obj.data.plan_queue{1}.start_time = t;
      end
    end

    function qp_input = getQPControllerInput(obj, t, x)
      % Get a QPInput structure describing the instantaneous input to the QP
      % controller
      % @param t the current time (s)
      % @param x the current robot state vector
      current_plan = obj.getCurrentPlan(t, x);
      qp_input = current_plan.getQPControllerInput(t, x);
    end

    function obj = insertPlan(obj, new_plan, idx)
      % Insert a new plan to the queue *before* the given index.
      % For example, to move a new plan to the front of the queue, set idx=1
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
      % Add a plan to the end of the queue
      obj.data.plan_queue{end+1} = new_plan;
    end

    function obj = switchToPlan(obj, new_plan)
      % Replace the queue with a single new plan, which will become the
      % current plan immediately.
      obj.data.plan_queue = {new_plan};
    end
  end
end

