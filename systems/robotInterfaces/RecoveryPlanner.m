classdef RecoveryPlanner < MixedIntegerConvexProgram
	properties
    has_setup = false;
    start
    omega
    nsteps = 15;
    dt = 0.05;
    max_foot_velocity = 2; % m / s
    STANCE_UPPER_BOUND = 2; % m, an upper bound on the width of the robot's stance, for mixed-integer constraint formulation
  end

  methods
    function obj = RecoveryPlanner(nsteps, dt)
      checkDependency('yalmip');
      yalmip('clear');
      obj = obj@MixedIntegerConvexProgram(true);

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


      obj = obj.addSymbolicConstraints([...
        obj.vars.qr.symb(:,end-1) == obj.vars.qr.symb(:,end),...
        obj.vars.ql.symb(:,end-1) == obj.vars.ql.symb(:,end),...
        ]);

    end

    function sol = solveBipedProblem(obj, biped, x0, zmp0)
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
      sol = obj.solveBaseProblem(start, omega);
    end

    function sol = solveBaseProblem(obj, start, omega)
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

      if start.contact(1)
        obj = obj.addSymbolicConstraints([...
          obj.vars.qr.symb(:,2) == obj.vars.qr.symb(:,1),...
          ]);
      end
      if start.contact(2)
        obj = obj.addSymbolicConstraints([...
          obj.vars.ql.symb(:,2) == obj.vars.ql.symb(:,1),...
          ]);
      end

      obj = obj.addDiscreteLinearDynamics();
      obj = obj.addReachability();
      obj = obj.addContactConstraints();
      obj = obj.addFootVelocityLimits();

      % obj = obj.addFinalCOMObjective();
      obj = obj.addCapturePointObjective();
      obj = obj.addFootMotionObjective();
      obj = obj.addFinalPostureObjective();

      % obj = obj.addSymbolicObjective(-.05 * sum(sum(obj.vars.contact.symb)));

      obj = obj.solveYalmip(sdpsettings('solver', 'gurobi', 'verbose', 1));

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

    function obj = addDiscreteLinearDynamics(obj)
      xcom = obj.vars.xcom.symb;
      qcop = obj.vars.qcop.symb;
      A = [0, 0, 1, 0;
           0, 0, 0, 1;
           obj.omega^2, 0, 0, 0;
           0, obj.omega^2, 0, 0];
      B = [0 0;
           0 0;
           -obj.omega^2, 0;
           0, -obj.omega^2];
      Ai = inv(A);
      for j = 1:obj.nsteps-1
        beta = qcop(:,j);
        alpha = (qcop(:,j+1) - qcop(:,j)) / obj.dt;
        T = -Ai * B * beta - Ai*Ai*B*alpha;
        S = -Ai * B * alpha;
        Q = xcom(:,j) + Ai * B * beta + Ai * Ai * B * alpha;
        obj = obj.addSymbolicConstraints(xcom(:,j+1) == expm(A*obj.dt) * Q + S * obj.dt + T);
      end

      obj = obj.addSymbolicConstraints([...
        qcop(:,end) == mean([obj.vars.qr.symb(:,end), obj.vars.ql.symb(:,end)], 2),...
        % obj.vars.contact.symb(:,end) == [1;1],...
        ]);
    end

    function obj = addReachability(obj)
      initial_deltas = [obj.start.qr - obj.start.xcom(1:2), obj.start.ql - obj.start.xcom(1:2)];
      max_forward_step = max([0.2, initial_deltas(1,:)]);
      min_backward_step = min([-0.15, initial_deltas(1,:)]);

      [Ar, br] = poly2lincon([min_backward_step, max_forward_step, max_forward_step, min_backward_step],...
                             [0.1, 0.1, -0.25, -0.25]);
      [Al, bl] = poly2lincon([min_backward_step, max_forward_step, max_forward_step, min_backward_step],...
                             [-0.1, -0.1, 0.25, 0.25]);
      A_l_minus_r = [0, 1; 0, -1];
      b_l_minus_r = [0.45; -0.16];
      warning('not handling orientation')
      for j = 2:obj.nsteps
        obj = obj.addSymbolicConstraints([...
          Ar * (obj.vars.qr.symb(1:2,j) - obj.vars.xcom.symb(1:2,j)) <= br,...
          Al * (obj.vars.ql.symb(1:2,j) - obj.vars.xcom.symb(1:2,j)) <= bl,...
          obj.vars.ql.symb(2,j) - obj.vars.qr.symb(2,j) >= 0.15,...
          A_l_minus_r * (obj.vars.ql.symb(1:2,j) - obj.vars.qr.symb(1:2,j)) <= b_l_minus_r,...
          ]);
      end
    end

    function obj = addContactConstraints(obj)
      obj = obj.addSymbolicConstraints([...
        sum(obj.vars.contained.symb(:,1:end), 1) == 1,...
        ]);

      for j = 2:obj.nsteps-1
        obj = obj.addSymbolicConstraints([...
          -0.05 - (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(1,j) - obj.vars.qr.symb(1,j)) <= 0.05 + (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
          -0.02 - (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(2,j) - obj.vars.qr.symb(2,j)) <= 0.02 + (1-obj.vars.contained.symb(1,j)) * obj.STANCE_UPPER_BOUND,...
          -0.05 - (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(1,j) - obj.vars.ql.symb(1,j)) <= 0.05 + (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
          -0.02 - (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND <= (obj.vars.qcop.symb(2,j) - obj.vars.ql.symb(2,j)) <= 0.02 + (1-obj.vars.contained.symb(2,j)) * obj.STANCE_UPPER_BOUND,...
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
    end

    function obj = addCapturePointObjective(obj)
      % r'ic = r' + rd'
      % r' = r / z
      % rd' = rd / (omega * z)
      % ric = r'ic * z = z * (r/z + rd / (omega*z)) = r + rd/omega
      obj = obj.addSymbolicObjective(...
        norm((obj.vars.xcom.symb(1:2,end) + 1/obj.omega * obj.vars.xcom.symb(3:4,end)) - mean([obj.vars.qr.symb(:,end), obj.vars.ql.symb(:,end)], 2), 1));
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

    function obj = addFootMotionObjective(obj)
      for j = 1:obj.nsteps-1
        obj = obj.addSymbolicObjective(...
          0.01 * sum(abs(obj.vars.qr.symb(:,j) - obj.vars.qr.symb(:,j+1))) + ...
          0.01 * sum(abs(obj.vars.ql.symb(:,j) - obj.vars.ql.symb(:,j+1))));
      end
    end

    function obj = addFinalPostureObjective(obj)
      obj = obj.addSymbolicObjective(...
        0.01 * sum(abs(obj.vars.qr.symb(:,end) - (obj.vars.xcom.symb(1:2,end) + [0; -0.13]))) + ...
        0.01 * sum(abs(obj.vars.ql.symb(:,end) - (obj.vars.xcom.symb(1:2,end) + [0; 0.13]))));
    end

    function obj = addFootVelocityLimits(obj)
      for j = 1:obj.nsteps-1
        obj = obj.addSymbolicConstraints([...
          abs(obj.vars.qr.symb(:,j+1) - obj.vars.qr.symb(:,j)) <= obj.max_foot_velocity * obj.dt,...
          abs(obj.vars.ql.symb(:,j+1) - obj.vars.ql.symb(:,j)) <= obj.max_foot_velocity * obj.dt,...
          ]);
      end
    end


  end
end