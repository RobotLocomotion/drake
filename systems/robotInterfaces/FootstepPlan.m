classdef FootstepPlan
  properties
    footsteps
    params
    safe_regions
    region_order
  end

  methods
    function obj = FootstepPlan(footsteps, params, safe_regions, region_order)
      obj.footsteps = footsteps;
      obj.params = struct(params);
      obj.safe_regions = safe_regions;
      obj.region_order = region_order;
    end

    function msg = to_footstep_plan_t(obj, biped)
      msg = drc.footstep_plan_t();
      msg.num_steps = length(obj.footsteps);
      step_msgs = javaArray('drc.footstep_t', msg.num_steps);
      for j = 1:msg.num_steps
        step_msgs(j) = obj.footsteps(j).to_footstep_t(biped);
      end
      msg.footsteps = step_msgs;
      msg.params = obj.params;
    end

    function msg = toLCM(obj)
      msg = obj.to_footstep_plan_t();
    end

    function plan = slice(obj, idx)
      plan = obj;
      plan.footsteps = obj.footsteps(idx);
      plan.region_order = obj.region_order(idx);
    end

    function plan = extend(obj, final_length, n)
      % Extend a footstep plan by replicating its final n footsteps. Useful for
      % generating seeds for later optimization.
      % @param final_length desired number of footsteps in the extended plan
      % @option n how many final steps to consider (the last n steps will be
      %          repeatedly appended to the footstep plan until the final
      %          length is achieved). Optional. Default: 2
      % @retval plan the extended plan
      if nargin < 3
        n = 2;
      end
      if n > length(obj.footsteps)
        error('DRC:FootstepPlan:NotEnoughStepsToExtend', 'Not enough steps in the plan to extend in the requested manner');
      end
      if final_length <= length(obj.footsteps)
        plan = plan.slice(1:final_length);
      else
        plan = obj;
        j = 1;
        source_ndx = (length(obj.footsteps) - n) + (1:n);
        for k = (length(obj.footsteps) + 1):final_length
          plan.footsteps(k) = plan.footsteps(source_ndx(j));
          plan.region_order(k) = plan.region_order(source_ndx(j));
          plan.footsteps(k).id = plan.footsteps(k-1).id + 1;
          j = mod(j, length(source_ndx)) + 1;
        end
      end
    end

    function ts = compute_step_timing(obj, biped)
      % Compute the approximate step timing based on the distance each swing foot must travel.
      % @retval ts a vector of times (in seconds) corresponding to the completion
      %            (return to double support) of each step in the plan. The first
      %            two entries of ts will always be zero, since the first two steps
      %            in the plan correspond to the current locations of the feet.
      ts = zeros(1, length(obj.footsteps));
      for j = 3:length(obj.footsteps)
        [swing_ts, ~, ~, ~] = planSwing(biped, obj.footsteps(j-2), obj.footsteps(j));
        ts(j) = ts(j-1) + swing_ts(end);
      end
    end

    function varargout = sanity_check(obj)
      ok = true;
      frame_ids = [obj.footsteps.frame_id];
      if any(frame_ids(1:end-1) == frame_ids(2:end))
        ok = false;
        if nargout < 1
          error('Body indices should not repeat.');
        end
      end
      varargout = {ok};
    end

    function draw_lcmgl(obj, lcmgl)
      for j = 1:length(obj.footsteps)
        if mod(j, 2)
          lcmgl.glColor3f(1,0,0);
        else
          lcmgl.glColor3f(0,1,0);
        end
        lcmgl.sphere(obj.footsteps(j).pos(1:3), 0.02, 20, 20);
      end
      lcmgl.switchBuffers();
    end
  end

  methods(Static=true)
    function plan = from_footstep_plan_t(msg, biped)
      footsteps = Footstep.empty();
      for j = 1:msg.num_steps
        footsteps(j) = Footstep.from_footstep_t(msg.footsteps(j), biped);
      end
      plan = FootstepPlan(footsteps, msg.params, [], []);
    end

    function plan = blank_plan(nsteps, ordered_frame_id, params, safe_regions)
      footsteps = Footstep.empty();

      for j = 1:nsteps
        pos = nan(6,1);
        id = j;
        frame_id = ordered_frame_id(mod(j-1, length(ordered_frame_id)) + 1);
        is_in_contact = true;
        pos_fixed = zeros(6,1);
        terrain_pts = [];
        infeasibility = nan;
        walking_params = [];
        footsteps(j) = Footstep(pos, id, frame_id, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params);
      end
      region_order = nan(1, nsteps);
      plan = FootstepPlan(footsteps, params, safe_regions, region_order);
    end
  end
end
