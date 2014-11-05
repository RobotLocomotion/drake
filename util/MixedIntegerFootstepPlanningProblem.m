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
      if use_symbolic
        assert(obj.using_symbolic);
        w_goal = diag(obj.weights.goal(obj.pose_indices));
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
        error('not implemented');
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
        error('not implemented');
      end
    end

    function obj = addOuterUnitCircleEquality(obj, num_slices, use_symbolic)
      sector_width = 2*pi / num_slices;
      yaw0 = obj.seed_plan.footsteps(1).pos(6);
      angle_boundaries = (yaw0 - pi - sector_width/2):sector_width:(yaw0 + pi - sector_width/2);

      obj = obj.addVariable('sector', 'B', [length(angle_boundaries)-1, obj.nsteps], 0, 1);
      obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj.vars.cos_yaw.lb(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.ub(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.lb(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.cos_yaw.ub(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.lb(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.ub(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.lb(2) = sin(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.ub(2) = sin(obj.seed_plan.footsteps(2).pos(6));

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
      obj.vars.cos_yaw.lb(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.ub(1) = cos(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.cos_yaw.lb(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.cos_yaw.ub(2) = cos(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.lb(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.ub(1) = sin(obj.seed_plan.footsteps(1).pos(6));
      obj.vars.sin_yaw.lb(2) = sin(obj.seed_plan.footsteps(2).pos(6));
      obj.vars.sin_yaw.ub(2) = sin(obj.seed_plan.footsteps(2).pos(6));

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
  end
end

