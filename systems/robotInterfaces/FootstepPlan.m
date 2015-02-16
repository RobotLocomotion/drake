classdef FootstepPlan
% A container for footstep plans. A footstep plan contains the location of each
% footstep, the safe terrain regions that those steps occupy, and the assignments
% of each step to a safe region.
  properties
    footsteps % a list of Footstep objects
    params % footstep plan params, as in drc.footstep_plan_params_t
    safe_regions % a list of safe regions, as described in planFootsteps.m
    region_order % a list of the same length as footsteps. If region_order(i) == j, then footsteps(i).pos must be within safe_regions(j)
    biped
  end

  methods
    function obj = FootstepPlan(footsteps, biped, params, safe_regions, region_order)
      obj.footsteps = footsteps;
      obj.biped = biped;
      obj.params = struct(params);
      obj.safe_regions = safe_regions;
      obj.region_order = region_order;
    end

    function msg = to_footstep_plan_t(obj)
      msg = drc.footstep_plan_t();
      msg.num_steps = length(obj.footsteps);
      step_msgs = javaArray('drc.footstep_t', msg.num_steps);
      for j = 1:msg.num_steps
        step_msgs(j) = obj.footsteps(j).to_footstep_t(obj.biped);
      end
      msg.footsteps = step_msgs;
      msg.params = populateLCMFields(drc.footstep_plan_params_t(), obj.params);
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
      if nargin < 2
        biped = obj.biped;
      end
      ts = zeros(1, length(obj.footsteps));
      for j = 3:length(obj.footsteps)
        [swing_ts, ~, ~, ~] = planSwing(biped, obj.footsteps(j-2), obj.footsteps(j));
        ts(j) = ts(j-1) + swing_ts(end);
      end
    end

    function varargout = sanity_check(obj)
      % Self-test for footstep plans.
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
        lcmgl.glPushMatrix();
        lcmgl.glTranslated(obj.footsteps(j).pos(1),...
                           obj.footsteps(j).pos(2),...
                           obj.footsteps(j).pos(3));
        axis = rpy2axis(obj.footsteps(j).pos(4:6));
        axis = axis([4,1:3]); % LCMGL wants [angle; axis], not [axis; angle]
        axis(1) = 180 / pi * axis(1);
        lcmgl.glRotated(axis(1), axis(2), axis(3), axis(4));
        lcmgl.sphere([0;0;0], 0.015, 20, 20);
        lcmgl.glPushMatrix();
        len = 0.1;
        lcmgl.glTranslated(len / 2, 0, 0);
        lcmgl.drawArrow3d(len, 0.02, 0.02, 0.005);
        lcmgl.glPopMatrix();
        lcmgl.glPopMatrix();
      end
    end

    function draw_2d(obj)
      hold on
      X1 = [obj.footsteps(1:2:end).pos];
      X2 = [obj.footsteps(2:2:end).pos];
      plot(X1(1,:), X1(2,:), 'go',...
          X2(1,:), X2(2,:), 'ro')
      quiver(X1(1,:), X1(2,:), cos(X1(6,:)), sin(X1(6,:)),'Color', 'g', 'AutoScaleFactor', 0.2);
      quiver(X2(1,:), X2(2,:), cos(X2(6,:)), sin(X2(6,:)),'Color', 'r', 'AutoScaleFactor', 0.2);
      axis equal
    end

    function steps_rel = relative_step_offsets(obj)
      % Compute the relative displacements of the footsteps (for checking collocation results from footstepNLP)
      steps = obj.step_matrix();
      nsteps = length(obj.footsteps);
      steps_rel = zeros(6, nsteps-1);
      for j = 2:nsteps
        R = rotmat(-steps(6,j-1));
        steps_rel(:,j-1) = [R * (steps(1:2,j) - steps(1:2,j-1));
                    steps(3:6,j) - steps(3:6,j-1)];
      end
    end

    function steps = step_matrix(obj)
      % Return the footstep positions as a 6xnsteps matrix in x y z roll
      % pitch yaw
      steps = [obj.footsteps.pos];
    end

    function obj = trim_duplicates(obj, trim_threshold)
      % Remove steps from the beginning which are identical to the initial
      % poses and remove steps at the end which are identical to the final poses,
      % returning a modified copy of the plan.
      if nargin < 2
        trim_threshold = [0.02; 0.02; 0.02; inf; inf; pi/32];
      end
      poses = [obj.footsteps.pos];
      nsteps = length(obj.footsteps);

      trim_init = false(1, nsteps);
      trim_init(1:2) = 1;
      trim_final = false(1, nsteps);
      trim_final(end-1:end) = 1;
      % initial trim
      for j = 3:nsteps
        if sum(~trim_init) + 2 < obj.params.min_num_steps
          break
        end
        if mod(j, 2)
          trim_init(j) = all(abs(poses(:,j) - poses(:,1)) <= trim_threshold);
        else
          trim_init(j) = all(abs(poses(:,j) - poses(:,2)) <= trim_threshold);
        end
      end
      trim_init(find(trim_init, 2, 'last')) = false;

      % final trim
      for j = 3:nsteps-2
        if sum(~trim_final) + 2 < obj.params.min_num_steps
          break
        end
        if mod(nsteps-j, 2)
          trim_final(j) = all(abs(poses(:,j) - poses(:,end-1)) <= trim_threshold);
        else
          trim_final(j) = all(abs(poses(:,j) - poses(:,end)) <= trim_threshold);
        end
      end
      trim_final(find(trim_final, 2, 'first')) = false;

      obj = obj.slice(~(trim_init | trim_final));
    end

    function obj = applyTransform(obj, T)
      % Apply a transform to every footstep and safe region, returning a new FootstepPlan
      % @param T a homogeneous transformation matrix (4x4)

      % Convert each existing footstep using the homogeneous transform
      for j = 1:length(obj.footsteps)
        obj.footsteps(j).pos = tform2poseRPY(T * poseRPY2tform(obj.footsteps(j).pos));
      end

      % Now convert the safe regions. Since these regions are expressed as a polytope in 
      % x,y,yaw and a point and normal in x,y,z, the conversion is a bit more complicated.

      % To find the xyz delta, we have to remove the rotation component of T. We do this by
      % pre-multiplying by the inverse of its rotation:
      R = T(1:3,1:3);
      Ri = inv(R);
      Ti = [Ri, [0;0;0]; [0,0,0,1]];
      T2 = Ti * T;
      delta_xyz = T2(1:3,4);

      rpy = rotmat2rpy(T(1:3,1:3));
      assert(all(abs(rpy(1:2) <= 1e-3)), 'Safe region transformation has not been implemented for rotations which are not pure yaw');
      delta_yaw = rpy(3);

      % we have Ax <= b
      % we want A2 * x' <= b2
      % where x' = [R(1:2,1:2) * (x(1:2) + delta_xyz(1:2)); x(3) + delta_yaw]
      % H = [R(1:2,1:2), [0;0], [0,0,1]];
      % d = [delta_xyz(1:2); delta_yaw];
      % x' = H * (x + d)
      % A2 * (H*(x + d)) <= b2
      % A2*H*x <= b2 - A2*H*d
      % A2*H = A
      % b2 - A2*H*d = b;
      % A2 = A * Hi
      % b2 = b + A2*H*d = b + A*d

      H = [T(1:2,1:2), [0;0]; [0,0,1]];
      d = [delta_xyz(1:2); delta_yaw];
      for j = 1:length(obj.safe_regions)
        A = obj.safe_regions(j).A;
        b = obj.safe_regions(j).b;
        A2 = A / H;
        b2 = b + A * d;
        obj.safe_regions(j).A = A2;
        obj.safe_regions(j).b = b2;
        p = T * [obj.safe_regions(j).point; 1];
        n = T * [obj.safe_regions(j).normal; 0];
        obj.safe_regions(j).point = p(1:3);
        obj.safe_regions(j).normal = n(1:3);
      end
    end
  end

  methods(Static=true)
    function plan = from_footstep_plan_t(msg, biped)
      footsteps = Footstep.empty();
      for j = 1:msg.num_steps
        footsteps(j) = Footstep.from_footstep_t(msg.footsteps(j), biped);
      end
      plan = FootstepPlan(footsteps, biped, msg.params, [], []);
    end

    function plan = blank_plan(biped, nsteps, ordered_frame_id, params, safe_regions)
      % Construct a FootstepPlan with all footstep poses set to NaN, but with the individual
      % step frame_id fields filled out appropriately. This is useful because all of the
      % existing footstep optimizations require that the maximum number of footsteps and the
      % sequence of the feet be assigned beforehand.
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
      plan = FootstepPlan(footsteps, biped, params, safe_regions, region_order);
    end
  end
end
