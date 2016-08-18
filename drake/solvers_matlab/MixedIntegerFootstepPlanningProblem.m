classdef MixedIntegerFootstepPlanningProblem < MixedIntegerConvexProgram
% A general structure for various footstep planning approaches. For example implementations,
% see footstepPlanner.humanoids2014, footstepPlanner.linearUnitCircle, and footstepPlanner.fixedRotation
  properties
    biped;
    nsteps;
    seed_plan;
    weights;
    max_distance = 30;
    pose_indices = [1,2,3,6];
  end

  methods
    function obj = MixedIntegerFootstepPlanningProblem(biped, seed_plan, has_symbolic)
      % Construct a new problem, optionally with internal symbolic representations of
      % all variables. For more info on the symbolic vars, see MixedIntegerConvexProgram
      % @param biped a Biped.
      % @param seed_plan a blank footstep plan, provinding the structure of the
      %                  desired plan. Probably generated with
      %                  FootstepPlan.blank_plan()
      % @param has_symbolic whether to keep symbolic variables

      typecheck(biped, 'Biped');
      typecheck(seed_plan, 'FootstepPlan');

      obj = obj@MixedIntegerConvexProgram(has_symbolic);

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
      % For each index j in step_indices, add a cost of the form:
      % relative_weights(j) * (footsteps(:,j) - xgoal)' * w_goal * (footsteps(:,j) - xgoal)
      w_goal = diag(obj.weights.goal(obj.pose_indices));

      if use_symbolic
        assert(obj.has_symbolic);
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
      % Add a convex conic constraint that sin^2 + cos^2 <= 1
      obj = obj.addVariableIfNotPresent('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
      obj = obj.addVariableIfNotPresent('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
      if use_symbolic
        assert(obj.has_symbolic);
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
      % Add mixed-integer linear constraints requiring that sin and cos live on the 
      % piecewise linear outer approximation of the unit circle with num_slices sides.
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
        assert(obj.has_symbolic)
        obj.symbolic_constraints = [obj.symbolic_constraints,...
          sum(sector, 1) == 1,...
          yaw0 - pi <= yaw <= yaw0 + pi,...
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
        obj.vars.footsteps.lb(4,3:end) = yaw0 - pi;
        obj.vars.footsteps.ub(4,3:end) = yaw0 + pi;
        obj = obj.addVariable('unit_circle_radius', 'C', [1,1], 1, 1);
        obj = obj.addPolyConesByIndex([repmat(obj.vars.unit_circle_radius.i, 1, obj.nsteps); obj.vars.cos_yaw.i; obj.vars.sin_yaw.i], num_slices);
        Aeq = zeros(obj.nsteps, obj.nv);
        beq = zeros(size(Aeq, 1), 1);
        for j = 1:obj.nsteps
          Aeq(j, obj.vars.sector.i(:,j)) = 1;
          beq(j) = 1;
        end

        Ai = zeros((length(angle_boundaries)-1)*(obj.nsteps-2)*6, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        M = 2*pi;
        for s = 1:length(angle_boundaries)-1
          th0 = angle_boundaries(s);
          th1 = angle_boundaries(s+1);
          th = (th0 + th1) / 2;
          ct = cos(th);
          st = sin(th);
          k = tan((th1 - th0)/2) / ((th1 - th0) / 2);
          for j = 3:obj.nsteps
            % -yaw(j) <= -th0 + M(1-sector(s,j))
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.sector.i(s,j)) = M;
            bi(offset+1) = -th0 + M;
            offset = offset + 1;

            % yaw(j) <= th1 + M(1-sector(s,j))
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+1, obj.vars.sector.i(s,j)) = M;
            bi(offset+1) = th1 + M;
            offset = offset + 1;

            % cos_yaw(j) <= ct + (yaw(j) - th) * k * -st + M(1-sector(s,j))
            Ai(offset+1, obj.vars.cos_yaw.i(j)) = 1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = st * k;
            Ai(offset+1, obj.vars.sector.i(s,j)) = M;
            bi(offset+1) = ct - th * k * -st + M;
            offset = offset + 1;

            % cos_yaw(j) >= ct + (yaw(j) - th) * k * -st - M(1-sector(s,j))
            Ai(offset+1, obj.vars.cos_yaw.i(j)) = -1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = k * -st;
            Ai(offset+1, obj.vars.sector.i(s,j)) = M;
            bi(offset+1) = -ct + th * k * -st + M;
            offset = offset + 1;

            % sin_yaw(j) <= st + (yaw(j) - th) * k * ct + M(1-sector(s,j))
            Ai(offset+1, obj.vars.sin_yaw.i(j)) = 1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -k * ct;
            Ai(offset+1, obj.vars.sector.i(s,j)) = M;
            bi(offset+1) = st + -th * k * ct + M;
            offset = offset + 1;

            % sin_yaw(j) >= st + (yaw(j) - th) * k * ct - M(1-sector(s,j))
            Ai(offset+1, obj.vars.sin_yaw.i(j)) = -1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = k * ct;
            Ai(offset+1, obj.vars.sector.i(s,j)) = M;
            bi(offset+1) = -st + th * k * ct + M;
            offset = offset + 1;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, Aeq, beq);
      end

      obj = obj.addSectorTransitionConstraints(use_symbolic);
    end

    function obj = addInnerUnitCircleInequality(obj, num_slices, use_symbolic)
      % Add mixed-integer linear constraints requiring that sin and cos live outside
      % the piecewise linear inner approximation of the unit circle with num_slices sides.
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
        assert(obj.has_symbolic)
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
      % Add mixed-integer linear constraints implementing the piecewise linear relationship
      % between yaw and cos(yaw), sin(yaw) described in "Footstep Planning on
      % Uneven Terrain with Mixed-Integer Convex Optimization" by Robin Deits and
      % Russ Tedrake (Humanoids 2014)
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

      obj.vars.footsteps.lb(4,3:end) = min_yaw;
      obj.vars.footsteps.ub(4,3:end) = max_yaw;

      if use_symbolic
        assert(obj.has_symbolic);
        cos_sector = obj.vars.cos_sector.symb;
        sin_sector = obj.vars.sin_sector.symb;
        cos_yaw = obj.vars.cos_yaw.symb;
        sin_yaw = obj.vars.sin_yaw.symb;
        yaw = obj.vars.footsteps.symb(4,:);

        obj.symbolic_constraints = [obj.symbolic_constraints,...
          sum(cos_sector, 1) == 1,...
          sum(sin_sector, 1) == 1,...
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
        obj = obj.addVariable('unit_circle_slack', 'C', [1,1], norm([pi/4;pi/4]), norm([pi/4;pi/4]));
        Aeq_s = zeros(obj.nsteps, obj.nv);
        Aeq_c = zeros(obj.nsteps, obj.nv);
        beq = ones(size(Aeq_s, 1), 1);
        for j = 1:obj.nsteps
          Aeq_c(j, obj.vars.cos_sector.i(:,j)) = 1;
          Aeq_s(j, obj.vars.sin_sector.i(:,j)) = 1;
        end
        obj = obj.addLinearConstraints([], [], [Aeq_s; Aeq_c], [beq; beq]);
        obj = obj.addPolyConesByIndex([repmat(obj.vars.unit_circle_slack.i, 1, obj.nsteps-2); obj.vars.cos_yaw.i(3:end); obj.vars.sin_yaw.i(3:end)], 8);

        M = 2*pi;
        Ai = zeros((obj.nsteps-2) * (length(cos_boundaries)-1 + length(sin_boundaries)-1) * 4, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for s = 1:length(cos_boundaries)-1
          th0 = cos_boundaries(s);
          th1 = cos_boundaries(s+1);

          th = (th0 + th1)/2;
          cos_slope = -sin(th);
          cos_intercept = cos(th) - (cos_slope * th);

          for j = 3:obj.nsteps
            % -yaw(j) <= -th0 + M(1-cos_sector(s,j))
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+1) = -th0 + M;
            % yaw(j) <= th1 + M(1-cos_sector(s,j))
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+2) = th1 + M;
            offset = offset + 2;

            % cos_yaw(j) <= cos_slope * yaw(j) + cos_intercept + M(1-cos_sector(s,j))
            Ai(offset+1, obj.vars.cos_yaw.i(j)) = 1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -cos_slope;
            Ai(offset+1, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+1) = cos_intercept + M;
            % cos_yaw(j) >= cos_slope * yaw(j) + cos_intercept - M(1-cos_sector(s,j))
            Ai(offset+2, obj.vars.cos_yaw.i(j)) = -1;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = cos_slope;
            Ai(offset+2, obj.vars.cos_sector.i(s,j)) = M;
            bi(offset+2) = -cos_intercept + M;
            offset = offset + 2;
          end
        end
        for s = 1:length(sin_boundaries)-1
          th0 = sin_boundaries(s);
          th1 = sin_boundaries(s+1);

          th = (th0 + th1)/2;
          sin_slope = cos(th);
          sin_intercept = sin(th) - (sin_slope * th);

          for j = 3:obj.nsteps
            % -yaw(j) <= -th0 + M(1-sin_sector(s,j))
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+1) = -th0 + M;
            % yaw(j) <= th1 + M(1-sin_sector(s,j))
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+2) = th1 + M;
            offset = offset + 2;

            % sin_yaw(j) <= sin_slope * yaw(j) + sin_intercept + M(1-sin_sector(s,j))
            Ai(offset+1, obj.vars.sin_yaw.i(j)) = 1;
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -sin_slope;
            Ai(offset+1, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+1) = sin_intercept + M;
            % sin_yaw(j) >= sin_slope * yaw(j) + sin_intercept - M(1-sin_sector(s,j))
            Ai(offset+2, obj.vars.sin_yaw.i(j)) = -1;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = sin_slope;
            Ai(offset+2, obj.vars.sin_sector.i(s,j)) = M;
            bi(offset+2) = -sin_intercept + M;
            offset = offset + 2;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);

        % Consistency between sin and cos sectors
        Ai = zeros((obj.nsteps-2) * obj.vars.sin_sector.size(1) * 2, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        nsectors = obj.vars.sin_sector.size(1);
        for k = 1:nsectors
          for j = 3:obj.nsteps
            Ai(offset+1, obj.vars.cos_sector.i(k,j)) = 1;
            Ai(offset+1, obj.vars.sin_sector.i(max(1,k-1):min(k+1,nsectors),j)) = -1;
            Ai(offset+2, obj.vars.sin_sector.i(k,j)) = 1;
            Ai(offset+2, obj.vars.cos_sector.i(max(1,k-1):min(k+1,nsectors),j)) = -1;
            offset = offset + 2;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);

        % Transitions between sectors
        Ai = zeros((obj.nsteps-2) * (nsectors-1) * 2, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            for k = 1:nsectors - 1
              Ai(offset+1, obj.vars.cos_sector.i(k,j-1)) = 1;
              Ai(offset+1, obj.vars.cos_sector.i(k:k+1,j)) = -1;
              Ai(offset+2, obj.vars.sin_sector.i(k,j-1)) = 1;
              Ai(offset+2, obj.vars.sin_sector.i(k:k+1,j)) = -1;
              offset = offset + 2;
            end
          else
            for k = 2:nsectors
              Ai(offset+1, obj.vars.cos_sector.i(k,j-1)) = 1;
              Ai(offset+1, obj.vars.cos_sector.i(k-1:k,j)) = -1;
              Ai(offset+2, obj.vars.sin_sector.i(k,j-1)) = 1;
              Ai(offset+2, obj.vars.sin_sector.i(k-1:k,j)) = -1;
              offset = offset + 2;
            end
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
    end

    function obj = addInitialSinCosConstraints(obj)
      % Constrain the values of sin and cos for the current poses of the feet
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
        assert(obj.has_symbolic);
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
        Ai = zeros((obj.nsteps-2)*(obj.vars.sector.size(1)-1), obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            for k = 1:obj.vars.sector.size(1) - 1
              % obj.symbolic_constraints = [obj.symbolic_constraints, sum(sector(k:k+1,j)) >= sector(k,j-1)];
              Ai(offset+1, obj.vars.sector.i(k,j-1)) = 1;
              Ai(offset+1, obj.vars.sector.i(k:k+1,j)) = -1;
              offset = offset + 1;
            end
          else
            for k = 2:obj.vars.sector.size(1)
              % obj.symbolic_constraints = [obj.symbolic_constraints, sum(sector(k-1:k,j)) >= sector(k,j-1)];
              Ai(offset+1, obj.vars.sector.i(k,j-1)) = 1;
              Ai(offset+1, obj.vars.sector.i(k-1:k,j)) = -1;
              offset = offset + 1;
            end
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
    end

    function obj = addZAndYawReachability(obj, use_symbolic)
      % Add basic limits on delta z and delta yaw between footsteps
      if use_symbolic
        assert(obj.has_symbolic);
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
        Ai = zeros((obj.nsteps-2)*4, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for j = 3:obj.nsteps
          if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.left
            % obj.symbolic_constraints = [obj.symbolic_constraints,...
            %   -obj.seed_plan.params.max_inward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_outward_angle];
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.footsteps.i(4,j-1)) = 1;
            bi(offset+1) = obj.seed_plan.params.max_inward_angle;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.footsteps.i(4,j-1)) = -1;
            bi(offset+2) = obj.seed_plan.params.max_outward_angle;
            offset = offset + 2;
          else
            % obj.symbolic_constraints = [obj.symbolic_constraints,...
            %   -obj.seed_plan.params.max_outward_angle <= yaw(j) - yaw(j-1) <= obj.seed_plan.params.max_inward_angle];
            Ai(offset+1, obj.vars.footsteps.i(4,j)) = -1;
            Ai(offset+1, obj.vars.footsteps.i(4,j-1)) = 1;
            bi(offset+1) = obj.seed_plan.params.max_outward_angle;
            Ai(offset+2, obj.vars.footsteps.i(4,j)) = 1;
            Ai(offset+2, obj.vars.footsteps.i(4,j-1)) = -1;
            bi(offset+2) = obj.seed_plan.params.max_inward_angle;
            offset = offset + 2;
          end
          % obj.symbolic_constraints = [obj.symbolic_constraints,...
          %   -obj.seed_plan.params.nom_downward_step <= x(3,j) - x(3,j-1) <= obj.seed_plan.params.nom_upward_step];
          Ai(offset+1, obj.vars.footsteps.i(3,j)) = -1;
          Ai(offset+1, obj.vars.footsteps.i(3,j-1)) = 1;
          bi(offset+1) = obj.seed_plan.params.nom_downward_step;
          Ai(offset+2, obj.vars.footsteps.i(3,j)) = 1;
          Ai(offset+2, obj.vars.footsteps.i(3,j-1)) = -1;
          bi(offset+2) = obj.seed_plan.params.nom_upward_step;
          offset = offset + 2;
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
    end

    function obj = addXYReachabilityCircles(obj, use_symbolic)
      % Add quadratic constraints to restrict the relative foot displacements in X and Y. This
      % is the reachability method described in "Footstep Planning on
      % Uneven Terrain with Mixed-Integer Convex Optimization" by Robin Deits and
      % Russ Tedrake
      if use_symbolic
        assert(obj.has_symbolic);
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
        ncons = 2 * (obj.nsteps-2);
        quadcon = struct('Qc', repmat({sparse(obj.nv, obj.nv)}, 1, ncons), 'q', repmat({zeros(obj.nv, 1)}, 1, ncons), 'rhs', repmat({0}, 1, ncons));
        offset = 0;
        expected_offset = length(quadcon);
        for j = 2:obj.nsteps-1
          [rel_foci, radii] = obj.biped.getReachabilityCircles(obj.seed_plan.params, obj.seed_plan.footsteps(j).frame_id);
          assert(size(rel_foci, 2) == 2, 'I have hard-coded the number of reachability circles in this code. You can set use_symbolic=true if you need more than two');

          for k = 1:size(rel_foci, 2)
            quadcon(offset+1).rhs = radii(k)^2;

            Qc_elementwise = [...
              obj.vars.footsteps.i(1:2,j+1), obj.vars.footsteps.i(1:2,j+1), [1;1];
              obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j), [1;1];
              obj.vars.footsteps.i(1:2,j+1), obj.vars.footsteps.i(1:2,j), [-1; -1];
              obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j+1), [-1; -1];
              obj.vars.footsteps.i(1,j+1), obj.vars.cos_yaw.i(j), -1 * rel_foci(1,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(1,j+1), -1 * rel_foci(1,k);
              obj.vars.footsteps.i(1,j+1), obj.vars.sin_yaw.i(j), 1 * rel_foci(2,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(1,j+1), 1 * rel_foci(2,k);
              obj.vars.footsteps.i(2,j+1), obj.vars.sin_yaw.i(j), -1 * rel_foci(1,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(2,j+1), -1 * rel_foci(1,k);
              obj.vars.footsteps.i(2,j+1), obj.vars.cos_yaw.i(j) -1 * rel_foci(2,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(2,j+1), -1 * rel_foci(2,k);
              obj.vars.footsteps.i(1,j), obj.vars.cos_yaw.i(j), 1 * rel_foci(1,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(1,j), 1 * rel_foci(1,k);
              obj.vars.footsteps.i(1,j), obj.vars.sin_yaw.i(j), -1 * rel_foci(2,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(1,j), -1 * rel_foci(2,k);
              obj.vars.footsteps.i(2,j), obj.vars.sin_yaw.i(j), 1 * rel_foci(1,k);
              obj.vars.sin_yaw.i(j), obj.vars.footsteps.i(2,j), 1 * rel_foci(1,k);
              obj.vars.footsteps.i(2,j) obj.vars.cos_yaw.i(j), 1 * rel_foci(2,k);
              obj.vars.cos_yaw.i(j), obj.vars.footsteps.i(2,j), 1 * rel_foci(2,k);
              obj.vars.cos_yaw.i(j), obj.vars.cos_yaw.i(j), rel_foci(1,k)^2 + rel_foci(2,k)^2;
              obj.vars.sin_yaw.i(j), obj.vars.sin_yaw.i(j), rel_foci(1,k)^2 + rel_foci(2,k)^2;
              ];
            quadcon(offset+1).Qc = sparse(Qc_elementwise(:,1), Qc_elementwise(:,2), Qc_elementwise(:,3), obj.nv, obj.nv);

            quadcon(offset+1).q = zeros(obj.nv, 1);
            offset = offset + 1;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addQuadcon(quadcon);
      end
    end

    function obj = addXYReachabilityEllipse(obj, use_symbolic)
      % Add second-order conic constraints to restrict the reachable set of foot poses with an ellipse
      % instead of a set of circles (as in addXYReachabilityCircles). 
      if use_symbolic
        assert(obj.has_symbolic);
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
      % Add a binary variable for each footstep which, if true, forces that footstep to the
      % final pose in the footstep plan. This allows us to trim it out of the footstep plan later.
      % A linear objective placed on those trim variables lets us tune the number of footsteps
      % in the plan.
      if obj.nsteps <= 3
        % Only one step to take, no point in trimming
        return
      end
      obj = obj.addVariable('trim', 'B', [1, obj.nsteps], 0, 1);
      w_trim = obj.weights.relative(1) * (obj.seed_plan.params.nom_forward_step^2);
      min_num_steps = max([obj.seed_plan.params.min_num_steps + 2, 3]);

      if use_symbolic
        assert(obj.has_symbolic);
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
        obj.symbolic_objective = obj.symbolic_objective - w_trim * sum(trim) + w_trim * obj.nsteps;
      else
        obj.vars.trim.lb(end-1:end) = 1;
        obj.vars.trim.ub(end-1:end) = 1;
        obj.vars.trim.lb(1:2) = 0;
        obj.vars.trim.ub(1:2) = 0;

        Ai = zeros(obj.nsteps-1 + 1 + max(obj.nsteps-4, 0) * 8, obj.nv);
        bi = zeros(size(Ai, 1), 1);
        offset = 0;
        expected_offset = size(Ai, 1);
        for j = 2:obj.nsteps
          % trim(j) >= trim(j-1)
          Ai(offset+1, obj.vars.trim.i(j)) = -1;
          Ai(offset+1, obj.vars.trim.i(j-1)) = 1;
          offset = offset + 1;
        end
        % sum(trim) <= obj.nsteps - (min_num_steps - 2)
        Ai(offset+1, obj.vars.trim.i) = 1;
        bi(offset+1) = obj.nsteps - (min_num_steps - 2);
        offset = offset + 1;
        M = obj.max_distance;
        for j = 3:obj.nsteps-2
          if mod(obj.nsteps-j, 2)
            % obj.symbolic_constraints = [obj.symbolic_constraints, implies(trim(j), x(:,j) == x(:,end-1))];
            k = obj.nsteps-1;
          else
            k = obj.nsteps;
          end
          % x(:,j) - x(:,k) <= M(1-trim(j))
          Ai(offset+(1:4), obj.vars.footsteps.i(:,j)) = speye(4);
          Ai(offset+(1:4), obj.vars.footsteps.i(:,k)) = -speye(4);
          Ai(offset+(1:4), obj.vars.trim.i(j)) = M;
          bi(offset+(1:4)) = M;
          offset = offset + 4;

          % x(:,j) - x(:,k) >= -M(1-trim(j))
          Ai(offset+(1:4), obj.vars.footsteps.i(:,j)) = -speye(4);
          Ai(offset+(1:4), obj.vars.footsteps.i(:,k)) = speye(4);
          Ai(offset+(1:4), obj.vars.trim.i(j)) = M;
          bi(offset+(1:4)) = M;
          offset = offset + 4;
        end
        obj = obj.addLinearConstraints(Ai, bi, [], []);
        assert(offset == expected_offset);

        c = zeros(obj.nv, 1);
        c(obj.vars.trim.i) = -w_trim;
        obj = obj.addCost([], c, w_trim * obj.nsteps);
      end
    end

    function obj = addQuadraticRelativeObjective(obj, use_symbolic)
      % Add a quadratic cost on the relative displacement between footsteps
      if use_symbolic
        assert(obj.has_symbolic);
        x = obj.vars.footsteps.symb;
        for j = 3:obj.nsteps
          R = [obj.vars.cos_yaw.symb(j-1), -obj.vars.sin_yaw.symb(j-1); 
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
        Qi = sparse(obj.nv, obj.nv);
        for j = 3:obj.nsteps
          if j == obj.nsteps
            w_rel = obj.weights.relative_final(obj.pose_indices);
          else
            w_rel = obj.weights.relative(obj.pose_indices);
          end
          if obj.seed_plan.footsteps(j-1).frame_id == obj.biped.foot_frame_id.right
            nom = [0; obj.seed_plan.params.nom_step_width];
          else
            nom = [0; -obj.seed_plan.params.nom_step_width];
          end
          assert(nom(1) == 0, 'I have hard-coded the assumption that nom(1) == 0. You can set use_symbolic=true if you want a non-zero value');
          Qnew = [...
            obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j), w_rel(1:2);
            obj.vars.footsteps.i(1:2,j-1), obj.vars.footsteps.i(1:2,j-1), w_rel(1:2);
            obj.vars.footsteps.i(1:2,j), obj.vars.footsteps.i(1:2,j-1), -w_rel(1:2);
            obj.vars.footsteps.i(1:2,j-1), obj.vars.footsteps.i(1:2,j), -w_rel(1:2);
            obj.vars.sin_yaw.i(j-1), obj.vars.sin_yaw.i(j-1), w_rel(1) * nom(2)^2;
            obj.vars.cos_yaw.i(j-1), obj.vars.cos_yaw.i(j-1), w_rel(2) * nom(2)^2;
            obj.vars.footsteps.i(1,j), obj.vars.sin_yaw.i(j-1), w_rel(1) * nom(2);
            obj.vars.sin_yaw.i(j-1), obj.vars.footsteps.i(1,j), w_rel(1) * nom(2);
            obj.vars.footsteps.i(2,j), obj.vars.cos_yaw.i(j-1), -w_rel(2) * nom(2);
            obj.vars.cos_yaw.i(j-1), obj.vars.footsteps.i(2,j), -w_rel(2) * nom(2);
            obj.vars.footsteps.i(1,j-1), obj.vars.sin_yaw.i(j-1), -w_rel(1) * nom(2);
            obj.vars.sin_yaw.i(j-1), obj.vars.footsteps.i(1,j-1), -w_rel(1) * nom(2);
            obj.vars.footsteps.i(2,j-1), obj.vars.cos_yaw.i(j-1), w_rel(2) * nom(2);
            obj.vars.cos_yaw.i(j-1), obj.vars.footsteps.i(2,j-1), w_rel(2) * nom(2);
            obj.vars.footsteps.i(3:4,j), obj.vars.footsteps.i(3:4,j), w_rel(3:4);
            obj.vars.footsteps.i(3:4,j-1), obj.vars.footsteps.i(3:4,j), -w_rel(3:4);
            obj.vars.footsteps.i(3:4,j), obj.vars.footsteps.i(3:4,j-1), -w_rel(3:4);
            obj.vars.footsteps.i(3:4,j-1), obj.vars.footsteps.i(3:4,j-1), w_rel(3:4);
            ];

          Qnew = sparse(Qnew(:,1), Qnew(:,2), Qnew(:,3), obj.nv, obj.nv);
          for k = 1:10
            x1 = rand(4,1);
            x2 = rand(4,1);
            sy = rand();
            cy = rand();
            X = zeros(obj.nv, 1);
            X(obj.vars.footsteps.i(:,j-1)) = x1;
            X(obj.vars.footsteps.i(:,j)) = x2;
            X(obj.vars.sin_yaw.i(j-1)) = sy;
            X(obj.vars.cos_yaw.i(j-1)) = cy;
            err = (x2 - [x1(1:2) + [cy, -sy; sy, cy] * nom; x1(3:4)]);
            O_des = err' * diag(w_rel) * err;
            O_actual = X' * Qnew * X;
            valuecheck(O_des, O_actual);
          end


          Qi = Qi + Qnew;
        end
        obj = obj.addCost(Qi, [], []);
      end
    end

    function obj = addTerrainRegions(obj, safe_regions, use_symbolic)
      % Add regions of safe terrain and mixed-integer constraints which require that 
      % each footstep lie within one of those safe regions.
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
        assert(obj.has_symbolic)
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

    function obj = fixRotation(obj)
      % Fix the rotations of every step (see footstepPlanner.fixedRotation())
      for j = 3:obj.nsteps
        if isnan(obj.seed_plan.footsteps(j).pos(6))
          if mod(j, 2)
            yaw = obj.seed_plan.footsteps(1).pos(6);
          else
            yaw = obj.seed_plan.footsteps(2).pos(6);
          end
        else
          yaw = obj.seed_plan.footsteps(j).pos(6);
        end

        obj.seed_plan.footsteps(j).pos(6) = yaw;
        obj.vars.footsteps.lb(4,j) = yaw - 0.01;
        obj.vars.footsteps.ub(4,j) = yaw + 0.01;
      end

      seed_steps = [obj.seed_plan.footsteps.pos];
      yaw = seed_steps(6,:);
      obj = obj.addVariable('cos_yaw', 'C', [1, obj.nsteps], cos(yaw), cos(yaw));
      obj = obj.addVariable('sin_yaw', 'C', [1, obj.nsteps], sin(yaw), sin(yaw));
    end

    function obj = addReachabilityLinearConstraints(obj)
      % Add linear constraints describing the reachable polytope (see footstepPlanner.fixedRotation())
      seed_steps = [obj.seed_plan.footsteps.pos];
      yaw = seed_steps(6,:);
      if any(isnan(yaw))
        error('Drake:MixedIntegerFootstepPlanningProblem:NaNsInYaw', 'Cannot handle NaNs in seed step yaws. You can call fixRotation() to set these yaw values correctly');
      end
      for j = 3:obj.nsteps
        R = [rotmat(-yaw(j-1)), zeros(2,2); zeros(2,2), eye(2)];
        [A_reach, b_reach] = obj.biped.getReachabilityPolytope(obj.seed_plan.footsteps(j-1).frame_id, obj.seed_plan.footsteps(j).frame_id, obj.seed_plan.params);
        A_reach = A_reach(:,obj.pose_indices);
        Ai = zeros(size(A_reach, 1), obj.nv);
        rA_reach = A_reach * R;
        Ai(:,obj.vars.footsteps.i(:,j)) = rA_reach;
        Ai(:,obj.vars.footsteps.i(:,j-1)) = -rA_reach;
        bi = b_reach;
        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
    end

    % function obj = addSmallRelativeAngleRotation(obj, use_symbolic)
    %   error('these constraints turned out to be backwards, so this approach doesn''t work yet -rdeits');
    %   obj = obj.addVariable('sin_is_positive', 'B', [1, obj.nsteps], 0, 1);
    %   obj = obj.addVariable('cos_is_positive', 'B', [1, obj.nsteps], 0, 1);
    %   obj = obj.addVariable('abs_delta_cos', 'C', [1, obj.nsteps-1], 0, 1);
    %   obj = obj.addVariable('abs_delta_sin', 'C', [1, obj.nsteps-1], 0, 1);
    %   obj = obj.addVariable('abs_delta_theta', 'C', [1, obj.nsteps-1], 0, pi);
    %   obj = obj.addVariable('abs_sin', 'C', [1, obj.nsteps-1], 0, 1);
    %   obj = obj.addVariable('abs_cos', 'C', [1, obj.nsteps-1], 0, 1);
    %   obj = obj.addVariable('cos_yaw', 'C', [1, obj.nsteps], -1, 1);
    %   obj = obj.addVariable('sin_yaw', 'C', [1, obj.nsteps], -1, 1);
    %   obj = obj.addVariable('abs_delta_cos_slack', 'C', [1, obj.nsteps-1], 0, 1);
    %   obj = obj.addVariable('abs_delta_sin_slack', 'C', [1, obj.nsteps-1], 0, 1);
    %   obj = obj.addInitialSinCosConstraints();

    %   if use_symbolic
    %     assert(obj.has_symbolic);
    %     abs_delta_cos = obj.vars.abs_delta_cos.symb;
    %     abs_delta_sin = obj.vars.abs_delta_sin.symb;
    %     abs_delta_cos_slack = obj.vars.abs_delta_cos_slack.symb;
    %     abs_delta_sin_slack = obj.vars.abs_delta_sin_slack.symb;
    %     abs_delta_theta = obj.vars.abs_delta_theta.symb;
    %     sin_is_positive = obj.vars.sin_is_positive.symb;
    %     cos_is_positive = obj.vars.cos_is_positive.symb;
    %     yaw = obj.vars.footsteps.symb(4,:);
    %     cos_yaw = obj.vars.cos_yaw.symb;
    %     sin_yaw = obj.vars.sin_yaw.symb;
    %     abs_sin = obj.vars.abs_sin.symb;
    %     abs_cos = obj.vars.abs_cos.symb;
    %     for j = 2:obj.nsteps
    %       obj.symbolic_constraints = [obj.symbolic_constraints,...
    %         implies(sin_is_positive(j-1), sin_yaw(j-1) == abs_sin(j-1)),...
    %         implies(~sin_is_positive(j-1), sin_yaw(j-1) == -abs_sin(j-1)),...
    %         implies(cos_is_positive(j-1), cos_yaw(j-1) == abs_cos(j-1)),...
    %         implies(~cos_is_positive(j-1), cos_yaw(j-1) == -abs_cos(j-1)),...
    %         ];
    %       if obj.seed_plan.footsteps(j).frame_id == obj.biped.foot_frame_id.right
    %         obj.symbolic_constraints = [obj.symbolic_constraints, ...
    %           abs_delta_theta(j-1) == -(yaw(j) - yaw(j-1)),...
    %           implies(sin_is_positive(j-1), abs_delta_cos(j-1) == cos_yaw(j) - cos_yaw(j-1)),...
    %           implies(~sin_is_positive(j-1), abs_delta_cos(j-1) == -(cos_yaw(j) - cos_yaw(j-1))),...
    %           implies(cos_is_positive(j-1), abs_delta_sin(j-1) == -(sin_yaw(j) - sin_yaw(j-1))),...
    %           implies(~cos_is_positive(j-1), abs_delta_sin(j-1) == sin_yaw(j) - sin_yaw(j-1)),...
    %           ];
    %       else
    %         obj.symbolic_constraints = [obj.symbolic_constraints,...
    %           abs_delta_theta(j-1) == (yaw(j) - yaw(j-1)),...
    %           implies(sin_is_positive(j-1), abs_delta_cos(j-1) == -(cos_yaw(j) - cos_yaw(j-1))),...
    %           implies(~sin_is_positive(j-1), abs_delta_cos(j-1) == (cos_yaw(j) - cos_yaw(j-1))),...
    %           implies(cos_is_positive(j-1), abs_delta_sin(j-1) == (sin_yaw(j) - sin_yaw(j-1))),...
    %           implies(~cos_is_positive(j-1), abs_delta_sin(j-1) == -(sin_yaw(j) - sin_yaw(j-1))),...
    %           ];
    %       end
    %       obj.symbolic_constraints = [obj.symbolic_constraints,...
    %         abs_delta_cos(j-1) >= abs_delta_cos_slack(j-1) * abs_delta_cos_slack(j-1);
    %         abs_delta_sin(j-1) >= abs_delta_sin_slack(j-1) * abs_delta_sin_slack(j-1);
    %         rcone(abs_delta_cos_slack(j-1), abs_delta_theta(j-1)/2, abs_sin(j-1)),...
    %         rcone(abs_delta_sin_slack(j-1), abs_delta_theta(j-1)/2, abs_cos(j-1)),...
    %         cone([cos_yaw(j) - cos_yaw(j-1); sin_yaw(j) - sin_yaw(j-1)], abs_delta_theta(j-1)),...
    %         cone([cos_yaw(j); sin_yaw(j)], 1),...
    %         ];
    %     end
    %   else
    %     error('not implemented');
    %   end
    % end

    function plan = getFootstepPlan(obj)
      % Solve the problem if needed and retrieve a footstep plan with the corresponding solution.
      if ~isfield(obj.vars.footsteps, 'value')
        obj = obj.solve();
      end
      steps = zeros(6, obj.nsteps);
      steps(obj.pose_indices, :) = obj.vars.footsteps.value;
      plan = obj.seed_plan;
      for j = 1:obj.nsteps
        plan.footsteps(j).pos = steps(:,j);
      end

      for j = 1:obj.nsteps
        region_ndx = find(obj.vars.region.value(:,j));
        assert(length(region_ndx) == 1, 'Got no (or multiple) region assignments for this footstep. This indicates an infeasibility or bad setup in the mixed-integer program');
        plan.region_order(j) = region_ndx;
      end
      plan = plan.trim_duplicates();

    end
  end
end

