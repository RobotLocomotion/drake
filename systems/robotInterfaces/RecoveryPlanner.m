classdef RecoveryPlanner < MixedIntegerConvexProgram
	properties
    has_setup = false;
    start
    omega
    nsteps = 15;
    dt = 0.05;
    max_foot_velocity = 2; % m / s
    STANCE_UPPER_BOUND = 2; % m, an upper bound on the width of the robot's stance, for mixed-integer constraint formulation
    weights = struct('foot_motion', 0.01,...
                     'final_posture', 0.01);
    nom_stance_width = 0.26;
  end

  methods
    function obj = RecoveryPlanner(nsteps, dt, use_symbolic)
      if nargin < 3
        use_symbolic = false;
      end
      if use_symbolic
        checkDependency('yalmip');
        yalmip('clear');
      end
      obj = obj@MixedIntegerConvexProgram(use_symbolic);

      if ~isempty(nsteps)
        obj.nsteps = nsteps;
      end
      if ~isempty(dt)
        obj.dt = dt;
      end

      obj = obj.addVariable('xcom', 'C', [4, obj.nsteps], -1, 1);
      obj = obj.addVariable('qcop', 'C', [2, obj.nsteps], -1, 1);
      obj = obj.addVariable('qr', 'C', [2, obj.nsteps], -1, 1);
      obj = obj.addVariable('ql', 'C', [2, obj.nsteps], -1, 1);
      obj = obj.addVariable('contained', 'B', [2, obj.nsteps], 0, 1);
      obj = obj.addVariable('capture_pt_obj', 'C', [1, 1], -inf, inf);
      obj = obj.addVariable('foot_motion_slack', 'C', [1, obj.nsteps-1], -inf, inf);
      obj = obj.addVariable('foot_motion_obj', 'C', [1, 1], -inf, inf);
      obj = obj.addVariable('posture_obj', 'C', [1, 1], -inf, inf);
      obj = obj.addVariable('posture_slack', 'C', [2, 1], -inf, inf);


    end

    function sol = solveBipedProblem(obj, biped, x0, zmp0, use_symbolic)
      typecheck(biped, 'Biped')
      nq = biped.getNumPositions();
      q0 = x0(1:nq);
      feet_position = biped.feetPosition(q0);

      warning('Assuming ground is at 0 here')
      contact = [feet_position.right(3) < 0.01; feet_position.left(3) < 0.01];

      cop0 = mean([feet_position.right(1:2), feet_position.left(1:2)], 2);
      if isempty(zmp0)
        feet = [feet_position.right(1:2), feet_position.left(1:2)];
        cop0 = mean(feet(:,contact'), 2);
      else
        cop0 = zmp0(:,end);
      end

      kinsol = biped.doKinematics(q0);
      [qcom, J] = biped.getCOM(kinsol);
      qcomdot = J * x0((biped.getNumPositions()+1):end);

      start = struct('xcom', [qcom(1:2); qcomdot(1:2)],...
                     'qcop', cop0,...
                     'qr', feet_position.right(1:2),...
                     'ql', feet_position.left(1:2),...
                     'contact', contact);

      warning('another z = 0 assumption here');
      omega = sqrt(9.81 / qcom(3));
      sol = obj.solveBaseProblem(start, omega, use_symbolic);
    end

    function sol = solveBaseProblem(obj, start, omega, use_symbolic)
      if nargin < 4
        use_symbolic = false;
      end
      if obj.has_setup
        error('Drake:RecoveryPlanner:DuplicateSetup', 'Cannot call setup twice on the same recovery planner instance');
      end
      obj.has_setup = true;
      obj.start = start;
      obj.omega = omega;

      x_lb = [repmat(obj.start.xcom(1:2) - 30, 1, obj.nsteps);
              -10 + zeros(2, obj.nsteps)];
      x_ub = [repmat(obj.start.xcom(1:2) + 30, 1, obj.nsteps);
              10 + zeros(2, obj.nsteps)];
      obj.vars.xcom.lb = x_lb;
      obj.vars.xcom.ub = x_ub;
      obj.vars.qcop.lb = x_lb(1:2,:);
      obj.vars.qcop.ub = x_ub(1:2,:);
      obj.vars.qr.lb = x_lb(1:2,:);
      obj.vars.qr.ub = x_ub(1:2,:);
      obj.vars.ql.lb = x_lb(1:2,:);
      obj.vars.ql.ub = x_ub(1:2,:);

      for v = {'xcom', 'qr', 'ql', 'qcop'}
        name = v{1};
        obj.vars.(name).lb(:,1) = obj.start.(name)(1:obj.vars.(name).size(1));
        obj.vars.(name).ub(:,1) = obj.start.(name)(1:obj.vars.(name).size(1));
      end

      obj_base = obj;

      obj = obj.addInitialContactConstraints(start.contact, use_symbolic);
      obj = obj.addDiscreteLinearDynamics(use_symbolic);
      obj = obj.addReachability(use_symbolic);
      obj = obj.addContactConstraints(use_symbolic);
      obj = obj.addFootVelocityLimits(use_symbolic);
      obj = obj.addFinalFootVelocity(use_symbolic);

      % obj = obj.addFinalCOMObjective();
      obj = obj.addCapturePointObjective(use_symbolic);
      obj = obj.addFootMotionObjective(use_symbolic);
      obj = obj.addFinalPostureObjective(use_symbolic);

      % obj = obj.addSymbolicObjective(-.05 * sum(sum(obj.vars.contact.symb)));

      if use_symbolic
        [obj, solvertime, objval] = obj.solveYalmip(sdpsettings('solver', 'gurobi', 'verbose', 1));
      else
        [obj, solvertime, objval] = obj.solveGurobi(struct('outputflag', 1));
      end
      solvertime
      objval


      if use_symbolic == 2
        obj_check = obj_base;
        obj_check = obj_check.addInitialContactConstraints(start.contact, false);
        obj_check = obj_check.addDiscreteLinearDynamics(false);
        obj_check = obj_check.addReachability(false);
        obj_check = obj_check.addContactConstraints(false);
        obj_check = obj_check.addFootVelocityLimits(false);
        obj_check = obj_check.addFinalFootVelocity(false);

        % obj_check = obj_check.addFinalCOMObjective();
        obj_check = obj_check.addCapturePointObjective(false);
        obj_check = obj_check.addFootMotionObjective(false);
        obj_check = obj_check.addFinalPostureObjective(false);

        [obj_check, solvertime_check, objval_check] = obj_check.solveGurobi(struct('outputflag', 1));
        solvertime_check
        objval_check
        valuecheck(objval_check, objval, 1e-3);
        obj = obj_check;
      end

      sol = PointMassBipedPlan();
      sol.ts = obj.dt * (0:(obj.nsteps-1));
      sol.xcom = obj.vars.xcom.value;
      sol.qr = obj.vars.qr.value;
      sol.ql = obj.vars.ql.value;
      sol.qcop = obj.vars.qcop.value;
      motion = [any(abs(diff(obj.vars.qr.value, 1, 2)) >= 0.005), false;
                any(abs(diff(obj.vars.ql.value, 1, 2)) >= 0.005), false];
      support = ~(motion | [[false; false], motion(:,1:end-1)] | [motion(:,2:end), [false; false]]);
      support(:,1) = support(:,1) & obj.start.contact;
      sol.support = support;
      sol.omega = obj.omega;
    end

    function obj = addInitialContactConstraints(obj, contact, use_symbolic)
      if use_symbolic
        if contact(1)
          obj = obj.addSymbolicConstraints([...
            obj.vars.qr.symb(:,2) == obj.vars.qr.symb(:,1),...
            ]);
        end
        if contact(2)
          obj = obj.addSymbolicConstraints([...
            obj.vars.ql.symb(:,2) == obj.vars.ql.symb(:,1),...
            ]);
        end
      else
        Aeq = zeros(4, obj.nv);
        beq = zeros(4,1);
        offset = 0;
        if contact(1)
          Aeq(offset+(1:2),obj.vars.qr.i(:,2)) = eye(2);
          Aeq(offset+(1:2),obj.vars.qr.i(:,1)) = -eye(2);
          offset = offset + 2;
        end
        if contact(2)
          Aeq(offset+(1:2),obj.vars.ql.i(:,2)) = eye(2);
          Aeq(offset+(1:2),obj.vars.ql.i(:,1)) = -eye(2);
          offset = offset + 2;
        end
        Aeq = Aeq(1:offset,:);
        beq = beq(1:offset);
        obj = obj.addLinearConstraints([], [], Aeq, beq);
      end
    end

    function obj = addDiscreteLinearDynamics(obj, use_symbolic)
      A = [0, 0, 1, 0;
           0, 0, 0, 1;
           obj.omega^2, 0, 0, 0;
           0, obj.omega^2, 0, 0];
      B = [0 0;
           0 0;
           -obj.omega^2, 0;
           0, -obj.omega^2];
      Ai = inv(A);
      exAdt = expm(A * obj.dt);
      if use_symbolic
        xcom = obj.vars.xcom.symb;
        qcop = obj.vars.qcop.symb;
        for j = 1:obj.nsteps-1
          beta = qcop(:,j);
          alpha = (qcop(:,j+1) - qcop(:,j)) / obj.dt;
          T = -Ai * B * beta - Ai*Ai*B*alpha;
          S = -Ai * B * alpha;
          Q = xcom(:,j) + Ai * B * beta + Ai * Ai * B * alpha;
          obj = obj.addSymbolicConstraints(xcom(:,j+1) == exAdt * Q + S * obj.dt + T);
        end

        obj = obj.addSymbolicConstraints([...
          qcop(:,end) == mean([obj.vars.qr.symb(:,end), obj.vars.ql.symb(:,end)], 2),...
          % obj.vars.contact.symb(:,end) == [1;1],...
          ]);
      else
        Aeq = zeros(4 * (obj.nsteps-1) + 2, obj.nv);
        beq = zeros(size(Aeq, 1), 1);
        offset = 0;
        for j = 1:obj.nsteps-1
          % xcom(:,j) = rand(4,1);
          % qcop(:,j) = rand(2,1);
          % qcop(:,j+1) = rand(2,1);
          % beta = qcop(:,j);
          % alpha = (qcop(:,j+1) - qcop(:,j)) / obj.dt;
          % T = -Ai * B * beta - Ai*Ai*B*alpha;
          % S = -Ai * B * alpha;
          % Q = xcom(:,j) + Ai * B * beta + Ai * Ai * B * alpha;
          % xcom(:,j+1) = exAdt * Q + S * obj.dt + T;

          % valuecheck(xcom(:,j+1), exAdt * Q + S * obj.dt + T, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * beta + Ai * Ai * B * alpha) + S * obj.dt + T, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * qcop(:,j) + Ai * Ai * B * alpha) + S * obj.dt + T, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * qcop(:,j) + Ai * Ai * B * ((qcop(:,j+1) - qcop(:,j)) / obj.dt)) + S * obj.dt + T, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * qcop(:,j) + Ai * Ai * B * ((qcop(:,j+1) - qcop(:,j)) / obj.dt)) + (-Ai * B * alpha) * obj.dt + T, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * qcop(:,j) + Ai * Ai * B * ((qcop(:,j+1) - qcop(:,j)) / obj.dt)) + (-Ai * B * (qcop(:,j+1) - qcop(:,j)) / obj.dt) * obj.dt + T, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * qcop(:,j) + Ai * Ai * B * ((qcop(:,j+1) - qcop(:,j)) / obj.dt)) + (-Ai * B * (qcop(:,j+1) - qcop(:,j))) + T, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * qcop(:,j) + Ai * Ai * B * ((qcop(:,j+1) - qcop(:,j)) / obj.dt)) + (-Ai * B * (qcop(:,j+1) - qcop(:,j))) + -Ai * B * beta - Ai*Ai*B*alpha, 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * (xcom(:,j) + Ai * B * qcop(:,j) + Ai * Ai * B * ((qcop(:,j+1) - qcop(:,j)) / obj.dt)) + (-Ai * B * (qcop(:,j+1) - qcop(:,j))) + -Ai * B * qcop(:,j) - Ai*Ai*B*((qcop(:,j+1) - qcop(:,j)) / obj.dt), 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * xcom(:,j) + ...
          %                (exAdt * Ai * B + exAdt * Ai * Ai * B * -1/obj.dt + Ai * B - Ai * B  + Ai * Ai * B * 1/obj.dt) * qcop(:,j) + ...
          %                (exAdt * Ai * Ai * B * 1/obj.dt + -Ai * B + -Ai * Ai * B * 1/obj.dt) * qcop(:,j+1), 1e-8);
          % valuecheck(xcom(:,j+1), exAdt * xcom(:,j) + ...
          %                (exAdt * Ai * B + exAdt * Ai * Ai * B * -1/obj.dt + Ai * Ai * B * 1/obj.dt) * qcop(:,j) + ...
          %                (exAdt * Ai * Ai * B * 1/obj.dt + -Ai * B + -Ai * Ai * B * 1/obj.dt) * qcop(:,j+1), 1e-8);

          ci = offset+(1:4);
          Aeq(ci, obj.vars.xcom.i(:,j+1)) = -eye(4);
          Aeq(ci, obj.vars.xcom.i(:,j)) = exAdt;
          Aeq(ci, obj.vars.qcop.i(:,j)) = exAdt * Ai * B + exAdt * Ai * Ai * B * -1/obj.dt + Ai * Ai * B * 1/obj.dt;
          Aeq(ci, obj.vars.qcop.i(:,j+1)) = exAdt * Ai * Ai * B * 1/obj.dt + -Ai * B + -Ai * Ai * B * 1/obj.dt;
          offset = offset + 4;
        end
        Aeq(offset+(1:2), obj.vars.qcop.i(:,end)) = -eye(2);
        Aeq(offset+(1:2), obj.vars.qr.i(:,end)) = 0.5 * eye(2);
        Aeq(offset+(1:2), obj.vars.ql.i(:,end)) = 0.5 * eye(2);
        obj = obj.addLinearConstraints([], [], Aeq, beq);
      end
    end

    function obj = addReachability(obj, use_symbolic)
      initial_deltas = [obj.start.qr - obj.start.xcom(1:2), obj.start.ql - obj.start.xcom(1:2)];
      max_forward_step = max([0.2, initial_deltas(1,:)]);
      min_backward_step = min([-0.15, initial_deltas(1,:)]);
      [Ar, br] = poly2lincon([min_backward_step, max_forward_step, max_forward_step, min_backward_step],...
                             [0.1, 0.1, -0.25, -0.25]);
      [Al, bl] = poly2lincon([min_backward_step, max_forward_step, max_forward_step, min_backward_step],...
                             [-0.1, -0.1, 0.25, 0.25]);
      A_l_minus_r = [0, 1; 
                     0, -1];
      b_l_minus_r = [0.45;
                     -0.16];
      warning('not handling orientation')

      if use_symbolic
        for j = 2:obj.nsteps
          obj = obj.addSymbolicConstraints([...
            Ar * (obj.vars.qr.symb(1:2,j) - obj.vars.xcom.symb(1:2,j)) <= br,...
            Al * (obj.vars.ql.symb(1:2,j) - obj.vars.xcom.symb(1:2,j)) <= bl,...
            A_l_minus_r * (obj.vars.ql.symb(1:2,j) - obj.vars.qr.symb(1:2,j)) <= b_l_minus_r,...
            ]);
            % obj.vars.ql.symb(2,j) - obj.vars.qr.symb(2,j) >= 0.15,...
        end
      else
        A = zeros((size(Ar, 1) + size(Al, 1) + size(A_l_minus_r, 1)) * (obj.nsteps-1), obj.nv);
        b = zeros(size(A, 1), 1);
        offset = 0;
        for j = 2:obj.nsteps
          ci = offset + (1:size(Ar, 1));
          A(ci, obj.vars.qr.i(1:2,j)) = Ar;
          A(ci, obj.vars.xcom.i(1:2,j)) = -Ar;
          b(ci) = br;
          offset = ci(end);

          ci = offset + (1:size(Al, 1));
          A(ci, obj.vars.ql.i(1:2,j)) = Al;
          A(ci, obj.vars.xcom.i(1:2,j)) = -Al;
          b(ci) = bl;
          offset = ci(end);

          ci = offset + (1:size(A_l_minus_r, 1));
          A(ci, obj.vars.ql.i(1:2,j)) = A_l_minus_r;
          A(ci, obj.vars.qr.i(1:2,j)) = -A_l_minus_r;
          b(ci) = b_l_minus_r;
          offset = ci(end);
        end
        obj = obj.addLinearConstraints(A, b, [], []);
      end
    end

    function obj = addContactConstraints(obj, use_symbolic)
      foot_bounds = struct('x', [-0.05, 0.05],...
                           'y', [-0.02, 0.02]);
      if use_symbolic
        obj = obj.addSymbolicConstraints([...
          sum(obj.vars.contained.symb, 1) == 1,...
          ]);

        for j = 2:obj.nsteps-1
          obj = obj.addSymbolicConstraints([...
            foot_bounds.x(1) - (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(1,j) - obj.vars.qr.symb(1,j)) <= foot_bounds.x(2) + (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
            foot_bounds.y(1) - (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(2,j) - obj.vars.qr.symb(2,j)) <= foot_bounds.y(2) + (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
            foot_bounds.x(1) - (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(1,j) - obj.vars.ql.symb(1,j)) <= foot_bounds.x(2) + (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
            foot_bounds.y(1) - (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(2,j) - obj.vars.ql.symb(2,j)) <= foot_bounds.y(2) + (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
            ]);
        end
        for j = 1:obj.nsteps-1
          contained = obj.vars.contained.symb;
          obj = obj.addSymbolicConstraints([...
            abs(obj.vars.qr.symb(:,j+1) - obj.vars.qr.symb(:,j)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
            abs(obj.vars.ql.symb(:,j+1) - obj.vars.ql.symb(:,j)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
            ]);
          if j > 1
            obj = obj.addSymbolicConstraints([...
              abs(obj.vars.qr.symb(:,j) - obj.vars.qr.symb(:,j-1)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
              abs(obj.vars.ql.symb(:,j) - obj.vars.ql.symb(:,j-1)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
              ]);
          end
          if j > 2
            obj = obj.addSymbolicConstraints([...
              abs(obj.vars.qr.symb(:,j-1) - obj.vars.qr.symb(:,j-2)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
              abs(obj.vars.ql.symb(:,j-1) - obj.vars.ql.symb(:,j-2)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
              ]);
          end
          if j < obj.nsteps-1
            obj = obj.addSymbolicConstraints([...
              abs(obj.vars.qr.symb(:,j+2) - obj.vars.qr.symb(:,j+1)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
              abs(obj.vars.ql.symb(:,j+2) - obj.vars.ql.symb(:,j+1)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
              ]);
          end
        end
      else
        Aeq = zeros(obj.vars.contained.size(2), obj.nv);
        beq = ones(size(Aeq, 1), 1);
        Aeq(:,obj.vars.contained.i(1,:)) = eye(obj.vars.contained.size(2));
        Aeq(:,obj.vars.contained.i(2,:)) = eye(obj.vars.contained.size(2));
        obj = obj.addLinearConstraints([], [], Aeq, beq);

        A = zeros((obj.nsteps-2) * 8, obj.nv);
        b = zeros(size(A, 1), 1);
        ci = 1;
        for j = 2:obj.nsteps-1

          A(ci, obj.vars.qcop.i(1,j)) = -1;
          A(ci, obj.vars.qr.i(1,j)) = 1;
          A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = -1 * (foot_bounds.x(1) - obj.STANCE_UPPER_BOUND);
          ci = ci + 1;

          A(ci, obj.vars.qcop.i(1,j)) = 1;
          A(ci, obj.vars.qr.i(1,j)) = -1;
          A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = foot_bounds.x(2) + obj.STANCE_UPPER_BOUND;
          ci = ci + 1;

          A(ci, obj.vars.qcop.i(2,j)) = -1;
          A(ci, obj.vars.qr.i(2,j)) = 1;
          A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = -1 * (foot_bounds.y(1) - obj.STANCE_UPPER_BOUND);
          ci = ci + 1;

          A(ci, obj.vars.qcop.i(2,j)) = 1;
          A(ci, obj.vars.qr.i(2,j)) = -1;
          A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = foot_bounds.y(2) + obj.STANCE_UPPER_BOUND;
          ci = ci + 1;

          A(ci, obj.vars.qcop.i(1,j)) = -1;
          A(ci, obj.vars.ql.i(1,j)) = 1;
          A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = -1 * (foot_bounds.x(1) - obj.STANCE_UPPER_BOUND);
          ci = ci + 1;

          A(ci, obj.vars.qcop.i(1,j)) = 1;
          A(ci, obj.vars.ql.i(1,j)) = -1;
          A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = foot_bounds.x(2) + obj.STANCE_UPPER_BOUND;
          ci = ci + 1;

          A(ci, obj.vars.qcop.i(2,j)) = -1;
          A(ci, obj.vars.ql.i(2,j)) = 1;
          A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = -1 * (foot_bounds.y(1) - obj.STANCE_UPPER_BOUND);
          ci = ci + 1;

          A(ci, obj.vars.qcop.i(2,j)) = 1;
          A(ci, obj.vars.ql.i(2,j)) = -1;
          A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = foot_bounds.y(2) + obj.STANCE_UPPER_BOUND;
          ci = ci + 1;
        end
        obj = obj.addLinearConstraints(A, b, [], []);

        A = zeros(2 * 2 * 2 * (obj.nsteps-1 + obj.nsteps-2 + obj.nsteps-3 + obj.nsteps-2), obj.nv);
        b = zeros(size(A, 1), 1);
        offset = 0;
        expected_offset = size(A, 1);

        for j = 1:obj.nsteps-1
          % obj = obj.addSymbolicConstraints([...
          %   abs(obj.vars.qr.symb(:,j+1) - obj.vars.qr.symb(:,j)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
          %   abs(obj.vars.ql.symb(:,j+1) - obj.vars.ql.symb(:,j)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
          %   ]);
          ci = offset+(1:2);
          A(ci, obj.vars.qr.i(:,j+1)) = eye(2);
          A(ci, obj.vars.qr.i(:,j)) = -eye(2);
          A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = obj.STANCE_UPPER_BOUND;
          ci = offset+(3:4);
          A(ci, obj.vars.qr.i(:,j)) = eye(2);
          A(ci, obj.vars.qr.i(:,j+1)) = -eye(2);
          A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = obj.STANCE_UPPER_BOUND;
          offset = offset+4;

          ci = offset+(1:2);
          A(ci, obj.vars.ql.i(:,j+1)) = eye(2);
          A(ci, obj.vars.ql.i(:,j)) = -eye(2);
          A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = obj.STANCE_UPPER_BOUND;
          ci = offset+(3:4);
          A(ci, obj.vars.ql.i(:,j)) = eye(2);
          A(ci, obj.vars.ql.i(:,j+1)) = -eye(2);
          A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
          b(ci) = obj.STANCE_UPPER_BOUND;
          offset = offset+4;

          if j > 1
            % obj = obj.addSymbolicConstraints([...
            %   abs(obj.vars.qr.symb(:,j) - obj.vars.qr.symb(:,j-1)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
            %   abs(obj.vars.ql.symb(:,j) - obj.vars.ql.symb(:,j-1)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
            %   ]);
            ci = offset+(1:2);
            A(ci, obj.vars.qr.i(:,j-1)) = eye(2);
            A(ci, obj.vars.qr.i(:,j)) = -eye(2);
            A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            ci = offset+(3:4);
            A(ci, obj.vars.qr.i(:,j)) = eye(2);
            A(ci, obj.vars.qr.i(:,j-1)) = -eye(2);
            A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            offset = offset+4;

            ci = offset+(1:2);
            A(ci, obj.vars.ql.i(:,j-1)) = eye(2);
            A(ci, obj.vars.ql.i(:,j)) = -eye(2);
            A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            ci = offset+(3:4);
            A(ci, obj.vars.ql.i(:,j)) = eye(2);
            A(ci, obj.vars.ql.i(:,j-1)) = -eye(2);
            A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            offset = offset+4;
          end
          if j > 2
            % obj = obj.addSymbolicConstraints([...
            %   abs(obj.vars.qr.symb(:,j-1) - obj.vars.qr.symb(:,j-2)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
            %   abs(obj.vars.ql.symb(:,j-1) - obj.vars.ql.symb(:,j-2)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
            %   ]);
            ci = offset+(1:2);
            A(ci, obj.vars.qr.i(:,j-1)) = eye(2);
            A(ci, obj.vars.qr.i(:,j-2)) = -eye(2);
            A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            ci = offset+(3:4);
            A(ci, obj.vars.qr.i(:,j-2)) = eye(2);
            A(ci, obj.vars.qr.i(:,j-1)) = -eye(2);
            A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            offset = offset+4;

            ci = offset+(1:2);
            A(ci, obj.vars.ql.i(:,j-1)) = eye(2);
            A(ci, obj.vars.ql.i(:,j-2)) = -eye(2);
            A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            ci = offset+(3:4);
            A(ci, obj.vars.ql.i(:,j-2)) = eye(2);
            A(ci, obj.vars.ql.i(:,j-1)) = -eye(2);
            A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            offset = offset+4;
          end
          if j < obj.nsteps-1
            % obj = obj.addSymbolicConstraints([...
            %   abs(obj.vars.qr.symb(:,j+2) - obj.vars.qr.symb(:,j+1)) <= (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
            %   abs(obj.vars.ql.symb(:,j+2) - obj.vars.ql.symb(:,j+1)) <= (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
            %   ]);
            ci = offset+(1:2);
            A(ci, obj.vars.qr.i(:,j+1)) = eye(2);
            A(ci, obj.vars.qr.i(:,j+2)) = -eye(2);
            A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            ci = offset+(3:4);
            A(ci, obj.vars.qr.i(:,j+2)) = eye(2);
            A(ci, obj.vars.qr.i(:,j+1)) = -eye(2);
            A(ci, obj.vars.contained.i(1,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            offset = offset+4;

            ci = offset+(1:2);
            A(ci, obj.vars.ql.i(:,j+1)) = eye(2);
            A(ci, obj.vars.ql.i(:,j+2)) = -eye(2);
            A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            ci = offset+(3:4);
            A(ci, obj.vars.ql.i(:,j+2)) = eye(2);
            A(ci, obj.vars.ql.i(:,j+1)) = -eye(2);
            A(ci, obj.vars.contained.i(2,j)) = obj.STANCE_UPPER_BOUND;
            b(ci) = obj.STANCE_UPPER_BOUND;
            offset = offset+4;
          end
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(A, b, [], []);
      end
    end

    function obj = addCapturePointObjective(obj, use_symbolic)
      % r'ic = r' + rd'
      % r' = r / z
      % rd' = rd / (omega * z)
      % ric = r'ic * z = z * (r/z + rd / (omega*z)) = r + rd/omega
      if use_symbolic
        obj = obj.addSymbolicConstraints([...
          norm((obj.vars.xcom.symb(1:2,end) + 1/obj.omega * obj.vars.xcom.symb(3:4,end)) - mean([obj.vars.qr.symb(:,end), obj.vars.ql.symb(:,end)], 2), 1) <= obj.vars.capture_pt_obj.symb]);
        obj = obj.addSymbolicObjective(obj.vars.capture_pt_obj.symb);                               
      else
        A = zeros(4, obj.nv);
        b = zeros(size(A, 1), 1);
        ci = 1;

        A(ci, obj.vars.xcom.i(1,end)) = 1;
        A(ci, obj.vars.xcom.i(3,end)) = 1/obj.omega;
        A(ci, obj.vars.qr.i(1,end)) = -0.5;
        A(ci, obj.vars.ql.i(1,end)) = -0.5;
        A(ci, obj.vars.xcom.i(2,end)) = 1;
        A(ci, obj.vars.xcom.i(4,end)) = 1/obj.omega;
        A(ci, obj.vars.qr.i(2,end)) = -0.5;
        A(ci, obj.vars.ql.i(2,end)) = -0.5;
        A(ci, obj.vars.capture_pt_obj.i) = -1;
        ci = ci + 1;

        A(ci, obj.vars.xcom.i(1,end)) = -1;
        A(ci, obj.vars.xcom.i(3,end)) = -1/obj.omega;
        A(ci, obj.vars.qr.i(1,end)) = 0.5;
        A(ci, obj.vars.ql.i(1,end)) = 0.5;
        A(ci, obj.vars.xcom.i(2,end)) = 1;
        A(ci, obj.vars.xcom.i(4,end)) = 1/obj.omega;
        A(ci, obj.vars.qr.i(2,end)) = -0.5;
        A(ci, obj.vars.ql.i(2,end)) = -0.5;
        A(ci, obj.vars.capture_pt_obj.i) = -1;
        ci = ci + 1;

        A(ci, obj.vars.xcom.i(1,end)) = 1;
        A(ci, obj.vars.xcom.i(3,end)) = 1/obj.omega;
        A(ci, obj.vars.qr.i(1,end)) = -0.5;
        A(ci, obj.vars.ql.i(1,end)) = -0.5;
        A(ci, obj.vars.xcom.i(2,end)) = -1;
        A(ci, obj.vars.xcom.i(4,end)) = -1/obj.omega;
        A(ci, obj.vars.qr.i(2,end)) = 0.5;
        A(ci, obj.vars.ql.i(2,end)) = 0.5;
        A(ci, obj.vars.capture_pt_obj.i) = -1;
        ci = ci + 1;

        A(ci, obj.vars.xcom.i(1,end)) = -1;
        A(ci, obj.vars.xcom.i(3,end)) = -1/obj.omega;
        A(ci, obj.vars.qr.i(1,end)) = 0.5;
        A(ci, obj.vars.ql.i(1,end)) = 0.5;
        A(ci, obj.vars.xcom.i(2,end)) = -1;
        A(ci, obj.vars.xcom.i(4,end)) = -1/obj.omega;
        A(ci, obj.vars.qr.i(2,end)) = 0.5;
        A(ci, obj.vars.ql.i(2,end)) = 0.5;
        A(ci, obj.vars.capture_pt_obj.i) = -1;
        ci = ci + 1;

        obj = obj.addLinearConstraints(A, b, [], []);

        c = zeros(obj.nv, 1);
        c(obj.vars.capture_pt_obj.i) = 1;
        obj = obj.addCost([], c, []);
      end
    end

    function obj = addFinalCOMObjective(obj, xdotf)
      if nargin < 2
        xdotf = [0;0];
      end
      xf = mean([obj.vars.qr.symb(:,end), obj.vars.ql.symb(:,end)], 2);

      obj = obj.addSymbolicObjective(...
        norm(obj.vars.xcom.symb(1:2,end) - xf)^2 ...
        + norm(obj.vars.xcom.symb(3:4,end) - xdotf)^2);
    end

    function obj = addFootMotionObjective(obj, use_symbolic)
      if use_symbolic
        for j = 1:obj.nsteps-1
          obj = obj.addSymbolicObjective(...
            obj.weights.foot_motion * sum(abs(obj.vars.qr.symb(:,j) - obj.vars.qr.symb(:,j+1))) + ...
            obj.weights.foot_motion * sum(abs(obj.vars.ql.symb(:,j) - obj.vars.ql.symb(:,j+1))));
        end
      else
        A = zeros(4 * 2 * (obj.nsteps-1) + 1, obj.nv);
        b = zeros(size(A, 1), 1);
        offset = 0;
        expected_offset = size(A, 1);

        for j = 1:obj.nsteps-1
          ci = offset+1;
          A(ci, obj.vars.qr.i(1,j)) = 1;
          A(ci, obj.vars.qr.i(1,j+1)) = -1;
          A(ci, obj.vars.qr.i(2,j)) = 1;
          A(ci, obj.vars.qr.i(2,j+1)) = -1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;

          ci = offset+1;
          A(ci, obj.vars.qr.i(1,j)) = -1;
          A(ci, obj.vars.qr.i(1,j+1)) = 1;
          A(ci, obj.vars.qr.i(2,j)) = 1;
          A(ci, obj.vars.qr.i(2,j+1)) = -1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;

          ci = offset+1;
          A(ci, obj.vars.qr.i(1,j)) = 1;
          A(ci, obj.vars.qr.i(1,j+1)) = -1;
          A(ci, obj.vars.qr.i(2,j)) = -1;
          A(ci, obj.vars.qr.i(2,j+1)) = 1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;

          ci = offset+1;
          A(ci, obj.vars.qr.i(1,j)) = -1;
          A(ci, obj.vars.qr.i(1,j+1)) = 1;
          A(ci, obj.vars.qr.i(2,j)) = -1;
          A(ci, obj.vars.qr.i(2,j+1)) = 1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;

          ci = offset+1;
          A(ci, obj.vars.ql.i(1,j)) = 1;
          A(ci, obj.vars.ql.i(1,j+1)) = -1;
          A(ci, obj.vars.ql.i(2,j)) = 1;
          A(ci, obj.vars.ql.i(2,j+1)) = -1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;

          ci = offset+1;
          A(ci, obj.vars.ql.i(1,j)) = -1;
          A(ci, obj.vars.ql.i(1,j+1)) = 1;
          A(ci, obj.vars.ql.i(2,j)) = 1;
          A(ci, obj.vars.ql.i(2,j+1)) = -1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;

          ci = offset+1;
          A(ci, obj.vars.ql.i(1,j)) = 1;
          A(ci, obj.vars.ql.i(1,j+1)) = -1;
          A(ci, obj.vars.ql.i(2,j)) = -1;
          A(ci, obj.vars.ql.i(2,j+1)) = 1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;

          ci = offset+1;
          A(ci, obj.vars.ql.i(1,j)) = -1;
          A(ci, obj.vars.ql.i(1,j+1)) = 1;
          A(ci, obj.vars.ql.i(2,j)) = -1;
          A(ci, obj.vars.ql.i(2,j+1)) = 1;
          A(ci, obj.vars.foot_motion_slack.i(1,j)) = -1;
          offset = offset + 1;
        end
        ci = offset+1;
        A(ci, obj.vars.foot_motion_slack.i) = 1;
        A(ci, obj.vars.foot_motion_obj.i) = -1;
        offset = offset + 1;
        assert(offset == expected_offset);

        obj = obj.addLinearConstraints(A, b, [], []);
        c = zeros(obj.nv, 1);
        c(obj.vars.foot_motion_obj.i) = obj.weights.foot_motion;
        obj = obj.addCost([], c, []);
      end
    end

    function obj = addFinalPostureObjective(obj, use_symbolic)
      if use_symbolic
        obj = obj.addSymbolicObjective(...
          obj.weights.final_posture * sum(abs(obj.vars.qr.symb(:,end) - (obj.vars.xcom.symb(1:2,end) + [0; -obj.nom_stance_width/2]))) + ...
          obj.weights.final_posture * sum(abs(obj.vars.ql.symb(:,end) - (obj.vars.xcom.symb(1:2,end) + [0; obj.nom_stance_width/2]))));
      else
        A = zeros(4 * 2 + 1, obj.nv);
        b = zeros(size(A, 1), 1);
        ci = 1;

        nom_pose = [0; -obj.nom_stance_width/2];

        A(ci, obj.vars.qr.i(1,end)) = 1;
        A(ci, obj.vars.xcom.i(1,end)) = -1;
        b(ci) = nom_pose(1);
        A(ci, obj.vars.qr.i(2,end)) = 1;
        A(ci, obj.vars.xcom.i(2,end)) = -1;
        b(ci) = b(ci) + nom_pose(2);
        A(ci, obj.vars.posture_slack.i(1)) = -1;
        ci = ci + 1;

        A(ci, obj.vars.qr.i(1,end)) = -1;
        A(ci, obj.vars.xcom.i(1,end)) = 1;
        b(ci) = -nom_pose(1);
        A(ci, obj.vars.qr.i(2,end)) = 1;
        A(ci, obj.vars.xcom.i(2,end)) = -1;
        b(ci) = b(ci) + nom_pose(2);
        A(ci, obj.vars.posture_slack.i(1)) = -1;
        ci = ci + 1;

        A(ci, obj.vars.qr.i(1,end)) = 1;
        A(ci, obj.vars.xcom.i(1,end)) = -1;
        b(ci) = nom_pose(1);
        A(ci, obj.vars.qr.i(2,end)) = -1;
        A(ci, obj.vars.xcom.i(2,end)) = 1;
        b(ci) = b(ci) - nom_pose(2);
        A(ci, obj.vars.posture_slack.i(1)) = -1;
        ci = ci + 1;

        A(ci, obj.vars.qr.i(1,end)) = -1;
        A(ci, obj.vars.xcom.i(1,end)) = 1;
        b(ci) = -nom_pose(1);
        A(ci, obj.vars.qr.i(2,end)) = -1;
        A(ci, obj.vars.xcom.i(2,end)) = 1;
        b(ci) = b(ci) - nom_pose(2);
        A(ci, obj.vars.posture_slack.i(1)) = -1;
        ci = ci + 1;

        nom_pose = [0; obj.nom_stance_width/2];

        A(ci, obj.vars.ql.i(1,end)) = 1;
        A(ci, obj.vars.xcom.i(1,end)) = -1;
        b(ci) = nom_pose(1);
        A(ci, obj.vars.ql.i(2,end)) = 1;
        A(ci, obj.vars.xcom.i(2,end)) = -1;
        b(ci) = b(ci) + nom_pose(2);
        A(ci, obj.vars.posture_slack.i(2)) = -1;
        ci = ci + 1;

        A(ci, obj.vars.ql.i(1,end)) = -1;
        A(ci, obj.vars.xcom.i(1,end)) = 1;
        b(ci) = -nom_pose(1);
        A(ci, obj.vars.ql.i(2,end)) = 1;
        A(ci, obj.vars.xcom.i(2,end)) = -1;
        b(ci) = b(ci) + nom_pose(2);
        A(ci, obj.vars.posture_slack.i(2)) = -1;
        ci = ci + 1;

        A(ci, obj.vars.ql.i(1,end)) = 1;
        A(ci, obj.vars.xcom.i(1,end)) = -1;
        b(ci) = nom_pose(1);
        A(ci, obj.vars.ql.i(2,end)) = -1;
        A(ci, obj.vars.xcom.i(2,end)) = 1;
        b(ci) = b(ci) - nom_pose(2);
        A(ci, obj.vars.posture_slack.i(2)) = -1;
        ci = ci + 1;

        A(ci, obj.vars.ql.i(1,end)) = -1;
        A(ci, obj.vars.xcom.i(1,end)) = 1;
        b(ci) = -nom_pose(1);
        A(ci, obj.vars.ql.i(2,end)) = -1;
        A(ci, obj.vars.xcom.i(2,end)) = 1;
        b(ci) = b(ci) - nom_pose(2);
        A(ci, obj.vars.posture_slack.i(2)) = -1;
        ci = ci + 1;

        A(ci, obj.vars.posture_slack.i) = 1;
        A(ci, obj.vars.posture_obj.i) = -1;
        obj = obj.addLinearConstraints(A, b, [], []);

        c = zeros(obj.nv, 1);
        c(obj.vars.posture_obj.i) = obj.weights.final_posture;
        obj = obj.addCost([], c, []);
      end
    end

    function obj = addFootVelocityLimits(obj, use_symbolic)
      if use_symbolic
        for j = 1:obj.nsteps-1
          obj = obj.addSymbolicConstraints([...
            abs(obj.vars.qr.symb(:,j+1) - obj.vars.qr.symb(:,j)) <= obj.max_foot_velocity * obj.dt,...
            abs(obj.vars.ql.symb(:,j+1) - obj.vars.ql.symb(:,j)) <= obj.max_foot_velocity * obj.dt,...
            ]);
        end
      else
        A = zeros(2 * 2 * 2 * (obj.nsteps-1), obj.nv);
        b = zeros(size(A, 1), 1);
        offset = 0;
        expected_offset = size(A, 1);
        for j = 1:obj.nsteps-1
          ci = offset+(1:2);
          A(ci, obj.vars.qr.i(:,j+1)) = eye(2);
          A(ci, obj.vars.qr.i(:,j)) = -eye(2);
          b(ci) = obj.max_foot_velocity * obj.dt;
          ci = offset+(3:4);
          A(ci, obj.vars.qr.i(:,j)) = eye(2);
          A(ci, obj.vars.qr.i(:,j+1)) = -eye(2);
          b(ci) = obj.max_foot_velocity * obj.dt;
          offset = offset + 4;

          ci = offset+(1:2);
          A(ci, obj.vars.ql.i(:,j+1)) = eye(2);
          A(ci, obj.vars.ql.i(:,j)) = -eye(2);
          b(ci) = obj.max_foot_velocity * obj.dt;
          ci = offset+(3:4);
          A(ci, obj.vars.ql.i(:,j)) = eye(2);
          A(ci, obj.vars.ql.i(:,j+1)) = -eye(2);
          b(ci) = obj.max_foot_velocity * obj.dt;
          offset = offset + 4;
        end
        assert(offset == expected_offset);
        obj = obj.addLinearConstraints(A, b, [], []);
      end
    end

    function obj = addFinalFootVelocity(obj, use_symbolic)
      if use_symbolic
        obj = obj.addSymbolicConstraints([...
          obj.vars.qr.symb(:,end-1) == obj.vars.qr.symb(:,end),...
          obj.vars.ql.symb(:,end-1) == obj.vars.ql.symb(:,end),...
          ]);
      else
        Aeq = zeros(4, obj.nv);
        beq = zeros(size(Aeq, 1), 1);

        ci = 1:2;
        Aeq(ci, obj.vars.qr.i(:,end-1)) = eye(2);
        Aeq(ci, obj.vars.qr.i(:,end)) = -eye(2);

        ci = 3:4;
        Aeq(ci, obj.vars.ql.i(:,end-1)) = eye(2);
        Aeq(ci, obj.vars.ql.i(:,end)) = -eye(2);

        obj = obj.addLinearConstraints([], [], Aeq, beq);
      end
    end
  end
end