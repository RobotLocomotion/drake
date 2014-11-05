classdef MixedIntegerFootstepPlanningProblem < MixedIntegerConvexProgram
  properties
    biped;
    nsteps;
    seed_plan;
    weights;
    max_distance = 30;
    pose_indices = [1,2,3,6];
  end

  methods
    function obj = MixedIntegerFootstepPlanningProblem(biped, seed_plan, using_symbolic)
      obj = obj@MixedIntegerConvexProgram(using_symbolic);

      obj.biped = biped;
      obj.seed_plan = seed_plan;
      obj.nsteps = length(obj.seed_plan.footsteps);
      obj.weights = obj.biped.getFootstepOptimizationWeights();

      seed_steps = [seed_plan.footsteps.pos];
      min_yaw = pi * floor(seed_steps(6,1) / pi - 1);
      max_yaw = pi * ceil(seed_steps(6,1) / pi + 1);

      lb = [repmat(seed_steps(1:3,1) - obj.max_distance, 1, obj.nsteps);
            min_yaw + zeros(1, obj.nsteps)];
      ub = [repmat(seed_steps(1:3,1) + obj.max_distance, 1, obj.nsteps);
            max_yaw + zeros(1, obj.nsteps)];
      lb(:,1) = seed_steps(obj.pose_indices, 1);
      ub(:,1) = seed_steps(obj.pose_indices, 1);
      lb(:,2) = seed_steps(obj.pose_indices, 2);
      ub(:,2) = seed_steps(obj.pose_indices, 2);
      obj = obj.addVariable('footsteps', 'C', [4, obj.nsteps], lb, ub);
    end

    function obj = addQuadraticGoalObjective(obj, goal_pose, step_indices, relative_weights, use_symbolic)
      w_goal = diag(obj.weights.goal(obj.pose_indices));

      if use_symbolic
        assert(obj.using_symbolic);
        for i = 1:length(step_indices)
          j = step_indices(i);
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.right
            xg = goal_pose.right(obj.pose_indices);
          else
            xg = goal_pose.left(obj.pose_indices);
          end
          err = obj.vars.footsteps.symb(:,j) - xg;
          obj.symbolic_objective = obj.symbolic_objective + relative_weights(i) * err' * w_goal * err; 
        end
      else
        for j = step_indices
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.right
            xg = goal_pose.right(obj.pose_indices);
          else
            xg = goal_pose.left(obj.pose_indices);
          end
          Qi = sparse([], [], [], obj.nv, obj.nv, 4);
          ci = zeros(obj.nv, 1);
          Qi(obj.vars.footsteps.i(:,j), obj.vars.footsteps.i(:,j)) = w_goal;
          ci(obj.vars.footsteps.i(:,j)) = -2 * w_goal * xg;
          objcon_i = xg' * w_goal * xg;

          obj = obj.addCost(Qi, ci, objcon_i);
        end
      end
    end

    function obj = addOuterUnitCircleCone(obj, use_symbolic)
      obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
      if use_symbolic
        assert(obj.using_symbolic);
        for j = 1:obj.nsteps
          obj.symbolic_constraints = [obj.symbolic_constraints,...
            cone([obj.vars.cos_yaw.symb(j); obj.vars.sin_yaw.symb(j)], 1)];
        end
      else
        obj = obj.addVariable('unit_circle_radius', 'C', [1,1], 1, 1);
        obj = obj.addConesByIndex([repmat(obj.vars.unit_circle_radius.i, 1, obj.nsteps); obj.vars.cos_yaw.i; obj.vars.sin_yaw.i]);
      end
    end

    function obj = addOuterUnitCircleEquality(obj, num_slices, use_symbolic)
      sector_width = 2*pi / num_slices;
      yaw0 = obj.seed_plan.footsteps(1).pos(6);
      angle_boundaries = (yaw0 - pi - sector_width/2):sector_width:(yaw0 + pi - sector_width/2);

      obj = obj.addVariable('sector', 'B', [length(angle_boundaries)-1, obj.nsteps], 0, 1);
      obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addInitialSinCosConstraints();

      if use_symbolic
        yaw = obj.vars.footsteps.symb(4,:);
        sector = obj.vars.sector.symb;
        cos_yaw = obj.vars.cos_yaw.symb;
        sin_yaw = obj.vars.sin_yaw.symb;
        assert(obj.using_symbolic)
        obj.symbolic_constraints = [obj.symbolic_constraints,...
          sum(sector, 1) == 1,...
          yaw0 - pi <= yaw <= yaw0 + pi,...
          -1 <= sin_yaw <= 1,...
          -1 <= cos_yaw <= 1,...
          polycone([cos_yaw; sin_yaw], 1, num_slices),...
          ];
        for s = 1:length(angle_boundaries)-1
          th0 = angle_boundaries(s);
          th1 = angle_boundaries(s+1);
          th = (th0 + th1) / 2;
          ct = cos(th);
          st = sin(th);
          k = tan((th1 - th0)/2) / ((th1 - th0) / 2);
          for j = 3:obj.nsteps
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              implies(sector(s,j), th0 <= yaw(j) <= th1),...
              implies(sector(s,j), [cos_yaw(j); sin_yaw(j)] == [ct; st] + (yaw(j) - th) * k * [-st; ct])];
          end
        end
      else
        error('not implemented');
      end

      obj = obj.addSectorTransitionConstraints(use_symbolic);
    end

    function obj = addInnerUnitCircleInequality(obj, num_slices, use_symbolic)
      sector_width = 2*pi / num_slices;
      yaw0 = obj.seed_plan.footsteps(1).pos(6);
      angle_boundaries = (yaw0 - pi - sector_width/2):sector_width:(yaw0 + pi - sector_width/2);

      obj = obj.addVariable('sector', 'B', [length(angle_boundaries)-1, obj.nsteps], 0, 1);
      obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addInitialSinCosConstraints();

      if use_symbolic
        yaw = obj.vars.footsteps.symb(4,:);
        sector = obj.vars.sector.symb;
        cos_yaw = obj.vars.cos_yaw.symb;
        sin_yaw = obj.vars.sin_yaw.symb;
        assert(obj.using_symbolic)
        obj.symbolic_constraints = [obj.symbolic_constraints,...
          sum(sector, 1) == 1,...
          yaw0 - pi <= yaw <= yaw0 + pi,...
          polycone([cos_yaw; sin_yaw], 1, num_slices),...
          ];
        for s = 1:length(angle_boundaries)-1
          th0 = angle_boundaries(s);
          th1 = angle_boundaries(s+1);
          c0 = cos(th0);
          s0 = sin(th0);
          c1 = cos(th1);
          s1 = sin(th1);
          v = [c1; s1] - [c0; s0];
          d0 = v' * [c0; s0];
          d1 = v' * [c1; s1];
          u = [0, 1; -1, 0] * v;
          u = u / norm(u);

          for j = 3:obj.nsteps
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              implies(sector(s,j), th0 <= yaw(j) <= th1),...
              implies(sector(s,j), u' * [cos_yaw(j); sin_yaw(j)] >= u' * [c0; s0]),...
              implies(sector(s,j), (v' * [cos_yaw(j); sin_yaw(j)] - d0) / (d1 - d0) == (yaw(j) - th0) / (th1 - th0)),...
              ];
          end
        end
      else
        error('not implemented');
      end

      obj = obj.addSectorTransitionConstraints(use_symbolic);
    end

    function obj = addSinCosLinearEquality(obj, use_symbolic)
      yaw0 = obj.seed_plan.footsteps(1).pos(6);
      min_yaw = pi * floor(yaw0 / pi - 1);
      max_yaw = pi * ceil(yaw0 / pi + 1);
      cos_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-(pi/2-1); (pi/2-1)]), 1, []);
      sin_boundaries = reshape(bsxfun(@plus, [min_yaw:pi:max_yaw; min_yaw:pi:max_yaw], [-1; 1]), 1, []);

      obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addVariable('cos_sector', 'B', [length(cos_boundaries)-1, obj.nsteps], 0, 1);
      obj = obj.addVariable('sin_sector', 'B', [length(sin_boundaries)-1, obj.nsteps], 0, 1);
      obj = obj.addInitialSinCosConstraints();

      if use_symbolic
        assert(obj.using_symbolic);
        cos_sector = obj.vars.cos_sector.symb;
        sin_sector = obj.vars.sin_sector.symb;
        cos_yaw = obj.vars.cos_yaw.symb;
        sin_yaw = obj.vars.sin_yaw.symb;
        yaw = obj.vars.footsteps.symb(4,:);

        obj.symbolic_constraints = [obj.symbolic_constraints,...
          sum(cos_sector, 1) == 1,...
          sum(sin_sector, 1) == 1,...
          min_yaw <= yaw <= max_yaw,...
          polycone([cos_yaw; sin_yaw], norm([pi/4;pi/4]), 8),...
          ];

        for s = 1:length(cos_boundaries)-1
          th0 = cos_boundaries(s);
          th1 = cos_boundaries(s+1);

          th = (th0 + th1)/2;
          cos_slope = -sin(th);
          cos_intercept = cos(th) - (cos_slope * th);

          for j = 3:obj.nsteps
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              implies(cos_sector(s,j), th0 <= yaw(j) <= th1),...
              implies(cos_sector(s,j), cos_yaw(j) == cos_slope * yaw(j) + cos_intercept)];
          end
        end
        for s = 1:length(sin_boundaries)-1
          th0 = sin_boundaries(s);
          th1 = sin_boundaries(s+1);

          th = (th0 + th1)/2;
          sin_slope = cos(th);
          sin_intercept = sin(th) - (sin_slope * th);

          for j = 3:obj.nsteps
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              implies(sin_sector(s,j), th0 <= yaw(j) <= th1),...
              implies(sin_sector(s,j), sin_yaw(j) == sin_slope * yaw(j) + sin_intercept)];
          end
        end
        % Consistency between sin and cos sectors
        for k = 1:size(sin_sector, 1)
          for j = 3:obj.nsteps
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              sum(sin_sector(max(1,k-1):min(k+1,size(sin_sector,1)),j)) >= cos_sector(k,j),...
              sum(cos_sector(max(1,k-1):min(k+1,size(cos_sector,1)),j)) >= sin_sector(k,j)];
          end
        end

        % Transitions between sectors
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            for k = 1:size(cos_sector, 1) - 1
              obj.symbolic_constraints = [obj.symbolic_constraints,...
               sum(cos_sector(k:k+1,j)) >= cos_sector(k,j-1),...
               sum(sin_sector(k:k+1,j)) >= sin_sector(k,j-1),...
               ];
            end
          else
            for k = 2:size(cos_sector, 1)
              obj.symbolic_constraints = [obj.symbolic_constraints,...
                sum(cos_sector(k-1:k,j)) >= cos_sector(k,j-1),...
                sum(sin_sector(k-1:k,j)) >= sin_sector(k,j-1),...
                ];
            end
          end
        end
      else
        error('not implemented');
      end
    end

    function obj = addInitialSinCosConstraints(obj)
      obj.vars.cos_yaw.lb(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.ub(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.lb(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.cos_yaw.ub(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.lb(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.ub(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.lb(2) = sin(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.ub(2) = sin(obj.seed_plan.footsteps(2).pos(6));
    end

    function obj = addSectorTransitionConstraints(obj, use_symbolic)
      % Restrict the set of sector transitions based on the reachability of Atlas
      if use_symbolic
        assert(obj.using_symbolic);
        sector = obj.vars.sector.symb;
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            for k = 1:size(sector, 1) - 1
              obj.symbolic_constraints = [obj.symbolic_constraints, sum(sector(k:k+1,j)) >= sector(k,j-1)];
            end
          else
            for k = 2:size(sector, 1)
              obj.symbolic_constraints = [obj.symbolic_constraints, sum(sector(k-1:k,j)) >= sector(k,j-1)];
            end
          end
        end
      else
        error('not implemented');
      end
    end

    function obj = addZAndYawReachability(obj, use_symbolic)
      if use_symbolic
        assert(obj.using_symbolic);
        x = obj.vars.footsteps.symb;
        yaw = x(4,:);
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              -obj.seed_plan.params.max_inward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_outward_angle];
          else
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              -obj.seed_plan.params.max_outward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_inward_angle];
          end
          obj.symbolic_constraints = [obj.symbolic_constraints,...
            -obj.seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= obj.seed_plan.params.nom_upward_step];
        end
      else
        error('not implemented');
      end
    end

    function obj = addXYReachabilityCircles(obj, use_symbolic)
      if use_symbolic
        assert(obj.using_symbolic);
        x = obj.vars.footsteps.symb;
        cos_yaw = obj.vars.cos_yaw.symb;
        sin_yaw = obj.vars.sin_yaw.symb;
        for j = 3:obj.nsteps
          [rel_foci, radii] = obj.biped.getReachabilityCircles(obj.seed_plan.params, obj.seed_plan.footsteps(j-1).frame_id);
          for k = 1:size(rel_foci, 2)
            obj.symbolic_constraints = [obj.symbolic_constraints, ...
              cone(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * rel_foci(:,k) - x(1:2,j), radii(k))];
          end
        end
      else
        error('not implemented')
      end
    end

    function obj = addXYReachabilityEllipse(obj, use_symbolic)
      if use_symbolic
        assert(obj.using_symbolic);
        x = obj.vars.footsteps.symb;
        cos_yaw = obj.vars.cos_yaw.symb;
        sin_yaw = obj.vars.sin_yaw.symb;
        for j = 3:obj.nsteps
          [rel_foci, l] = obj.biped.getReachabilityEllipse(obj.seed_plan.params, obj.seed_plan.footsteps(j-1).frame_id);
          expr = 0;
          for k = 1:size(rel_foci, 2)
            expr = expr + norm(x(1:2,j) - (x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * rel_foci(:,k)));
          end
          obj.symbolic_constraints = [obj.symbolic_constraints,...
            expr <= l];
        end
      else
        error('not implemented');
      end
    end

    function obj = addTrimToFinalPoses(obj, use_symbolic)
      obj = obj.addVariable('trim', 'B', [1, obj.nsteps], 0, 1);
      w_trim = obj.weights.relative(1) * (obj.seed_plan.params.nom_forward_step^2);
      min_num_steps = max([obj.seed_plan.params.min_num_steps + 2, 3]);

      if use_symbolic
        assert(obj.using_symbolic);
        trim = obj.vars.trim.symb;
        x = obj.vars.footsteps.symb;
        obj.symbolic_constraints = [obj.symbolic_constraints,...
          trim(end-1:end) == 1,...
          trim(1:2) == 0,...
          trim(2:end) >= trim(1:end-1),...
          sum(trim) <= obj.nsteps - (min_num_steps - 2)];
        for j = 3:obj.nsteps-2
          if mod(obj.nsteps-j, 2)
            obj.symbolic_constraints = [obj.symbolic_constraints, implies(trim(j), x(:,j) == x(:,end-1))];
          else
            obj.symbolic_constraints = [obj.symbolic_constraints, implies(trim(j), x(:,j) == x(:,end))];
          end
        end
        obj.symbolic_objective = obj.symbolic_objective - w_trim * sum(trim);
      else
        error('not implemented');
      end
    end

    function obj = addQuadraticRelativeObjective(obj, use_symbolic)
      if use_symbolic
        assert(obj.using_symbolic);
        x = obj.vars.footsteps.symb;
        for j = 3:obj.nsteps
          R = [obj.vars.cos_yaw.symb(j-1), obj.vars.sin_yaw.symb(j-1); 
               obj.vars.sin_yaw.symb(j-1), obj.vars.cos_yaw.symb(j-1)];
          if j == obj.nsteps
            w_rel = diag(obj.weights.relative_final(obj.pose_indices));
          else
            w_rel = diag(obj.weights.relative(obj.pose_indices));
          end
          if obj.seed_plan.footsteps(j-1).frame_id == obj.biped.foot_frame_id.right
            nom = [0; obj.seed_plan.params.nom_step_width];
          else
            nom = [0; -obj.seed_plan.params.nom_step_width];
          end
          err = x(:,j) - [x(1:2,j-1) + R * nom; 
                          x(3:4,j-1)];
          obj.symbolic_objective = obj.symbolic_objective + err' * w_rel * err;
        end
      else
        error('not implemented');
      end
    end

    function obj = addTerrainRegions(obj, safe_regions, use_symbolic)
      if isempty(safe_regions)
        safe_regions = obj.seed_plan.safe_regions;
      end
      nr = length(safe_regions);
      obj = obj.addVariable('region', 'B', [nr, obj.nsteps], 0, 1);
      obj.vars.region.lb(1,1:2) = 1;
      obj.vars.region.ub(1,1:2) = 1;
      obj.vars.region.lb(2:end,1:2) = 0;
      obj.vars.region.ub(2:end,1:2) = 0;

      if use_symbolic
        assert(obj.using_symbolic)
        region = obj.vars.region.symb;
        x = obj.vars.footsteps.symb;
        obj.symbolic_constraints = [obj.symbolic_constraints,...
          sum(region, 1) == 1];
        for r = 1:nr
          A = safe_regions(r).A;
          b = safe_regions(r).b;
          Ar_ineq = [A(:,1:2), zeros(size(A, 1), 1), A(:,3)];
          br_ineq = b;
          Ar_eq = [safe_regions(r).normal', 0];
          br_eq = safe_regions(r).normal' * safe_regions(r).point;

          for j = 3:obj.nsteps
            obj.symbolic_constraints = [obj.symbolic_constraints,...
              implies(region(r,j), [Ar_ineq * x(:,j) <= br_ineq,...
                                    Ar_eq * x(:,j) == br_eq])];
          end
        end
      else
        Ai = zeros((obj.nsteps-2) * sum(cellfun(@(x) size(x, 1) + 2, {safe_regions.A})), obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset_ineq = 0;
        Aeq = zeros(obj.nsteps-2, obj.nv);
        beq = ones(obj.nsteps-2, 1);
        offset_eq = 0;

        for r = 1:nr
          A = safe_regions(r).A;
          b = safe_regions(r).b;
          Ar = [A(:,1:2), zeros(size(A, 1), 1), A(:,3);
                safe_regions(r).normal', 0;
                -safe_regions(r).normal', 0];
          br = [b;
                safe_regions(r).normal' * safe_regions(r).point;
                -safe_regions(r).normal' * safe_regions(r).point];
          s = size(Ar, 1);
          M = obj.max_distance;
          for j = 3:obj.nsteps
            Ai(offset_ineq + (1:s), obj.vars.footsteps.i(:,j)) = Ar;
            Ai(offset_ineq + (1:s), obj.vars.region.i(r,j)) = M;
            bi(offset_ineq + (1:s)) = br + M;
            offset_ineq = offset_ineq + s;
          end
        end
        assert(offset_ineq == size(Ai, 1));
        for j = 3:obj.nsteps
          Aeq(offset_eq + 1, obj.vars.region.i(:,j)) = 1;
          offset_eq = offset_eq + 1;
        end
        assert(offset_eq == size(Aeq, 1));
        obj = obj.addLinearConstraints(Ai, bi, Aeq, beq);
      end
    end

    function plan = getFootstepPlan(obj)
      if ~isfield(obj.vars.footsteps, 'value')
        [obj, ok, ~] = obj.solve();
        if ~ok
          error('Drake:FootstepPlanner:InfeasibleProblem', 'The footstep planning problem is infeasible');
        end
      end
      steps = zeros(6, obj.nsteps);
      steps(obj.pose_indices, :) = obj.vars.footsteps.value;
      plan = obj.seed_plan;
      for j = 1:obj.nsteps
        plan.footsteps(j).pos = steps(:,j);
      end
      plan = plan.trim_duplicates();
    end
  end
end

