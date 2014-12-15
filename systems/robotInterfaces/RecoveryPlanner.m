classdef RecoveryPlanner < MixedIntegerConvexProgram
	properties
    start
    omega
    nsteps
    dt
    max_foot_velocity = 2; % m / s
    STANCE_UPPER_BOUND = 2; % m, an upper bound on the width of the robot's stance, for mixed-integer constraint formulation
  end

  methods(Static)
    function obj = run(x0, xd0, qcop, qr, ql, contact)
      if nargin == 0
        x0 = [0;0;1];
        xd0 = [0.4;0.2;0];
        qr = [0;-0.1];
        ql = [0; 0.1];
        contact = [1;1];
      end

      checkDependency('gurobi');
      x0
      xd0
      qcop
      qr
      ql
      % qr = [0; qr(2)]
      % ql = [0; ql(2)]
      contact

      warning('assuming ground at z=0')

      start = struct('xcom', [x0(1:2); xd0(1:2)],...
                     'qcop', qcop,...
                     'qr', qr,...
                     'ql', ql,...
                     'contact', contact);

      omega = sqrt(9.81 / x0(3));
      obj = RecoveryPlanner(start, omega, 15, 0.05);
      obj.max_foot_velocity = 2;
      obj = obj.setup();
      obj = obj.solveYalmip(sdpsettings('solver', 'gurobi', 'verbose', 1));

      utraj = obj.getUtraj();

      breaks = utraj.getBreaks();

      ts = linspace(breaks(1), breaks(end));

      r = PointMassBiped(omega);
      v = r.constructVisualizer();
      sys = cascade(utraj, r);
      ytraj = sys.simulate([breaks(1), breaks(end)], start.xcom);
      v.playback(ytraj, struct('slider', true));

      ys_sim = ytraj.eval(ts);
      xcom_sim = ys_sim(1:2,:);

      us = utraj.eval(ts);
      xcom_des = us(end-1:end,:);
    end
  end

  methods
    function obj = RecoveryPlanner(start, omega, nsteps, dt)
      yalmip('clear');
      obj = obj@MixedIntegerConvexProgram(true);
      obj.start = start;
      obj.omega = omega;
      obj.nsteps = nsteps;
      obj.dt = dt;

      x_lb = [repmat(obj.start.xcom(1:2) - 30, 1, obj.nsteps);
              -10 + zeros(2, obj.nsteps)];
      x_ub = [repmat(obj.start.xcom(1:2) + 30, 1, obj.nsteps);
              10 + zeros(2, obj.nsteps)];
      obj = obj.addVariable('xcom', 'C', [4, obj.nsteps], x_lb, x_ub);
      obj = obj.addVariable('qcop', 'C', [2, obj.nsteps], x_lb(1:2,:), x_ub(1:2,:));
      obj = obj.addVariable('qr', 'C', [2, obj.nsteps], x_lb(1:2,:), x_ub(1:2,:));
      obj = obj.addVariable('ql', 'C', [2, obj.nsteps], x_lb(1:2,:), x_ub(1:2,:));
      obj = obj.addVariable('contained', 'B', [2, obj.nsteps], 0, 1);

      for v = {'xcom', 'qr', 'ql', 'qcop'}
        name = v{1};
        obj.vars.(name).lb(:,1) = obj.start.(name)(1:obj.vars.(name).size(1));
        obj.vars.(name).ub(:,1) = obj.start.(name)(1:obj.vars.(name).size(1));
      end

      % obj.vars.contained.lb(:,1) = obj.start.contact;
      % obj.vars.contained.ub(:,1) = obj.start.contact;

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
      obj = obj.addSymbolicConstraints([...
        obj.vars.qr.symb(:,end-1) == obj.vars.qr.symb(:,end),...
        obj.vars.ql.symb(:,end-1) == obj.vars.ql.symb(:,end),...
        ]);

    end

    function obj = setup(obj)
      obj = obj.addDiscreteLinearDynamics();
      obj = obj.addReachability();
      obj = obj.addContactConstraints();
      obj = obj.addFootVelocityLimits();

      % obj = obj.addFinalCOMObjective();
      obj = obj.addCapturePointObjective();
      obj = obj.addFootMotionObjective();
      obj = obj.addFinalPostureObjective();

      % obj = obj.addSymbolicObjective(-.05 * sum(sum(obj.vars.contact.symb)));

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
      for j = 1:obj.nsteps-1
        beta = qcop(:,j);
        alpha = (qcop(:,j+1) - qcop(:,j)) / obj.dt;
        Ai = inv(A);
        T = -Ai * B * beta - Ai*Ai*B*alpha;
        S = -Ai * B * alpha;
        Q = xcom(:,j) + Ai * B * beta + Ai * Ai * B * alpha;
        obj = obj.addSymbolicConstraints(xcom(:,j+1) == expm(A*obj.dt) * Q + S * obj.dt + T);
%         obj = obj.addSymbolicConstraints(xcom(:,j+1) - [qcop(:,j); 0; 0] == expm(obj.dt * A) * (xcom(:,j) - [qcop(:,j); 0; 0]));
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
                             [0, 0, -0.25, -0.25]);
      [Al, bl] = poly2lincon([min_backward_step, max_forward_step, max_forward_step, min_backward_step],...
                             [0, 0, 0.25, 0.25]);
      warning('not handling orientation')
      for j = 2:obj.nsteps
        obj = obj.addSymbolicConstraints([...
          Ar * (obj.vars.qr.symb(1:2,j) - obj.vars.xcom.symb(1:2,j)) <= br,...
          Al * (obj.vars.ql.symb(1:2,j) - obj.vars.xcom.symb(1:2,j)) <= bl,...
          obj.vars.ql.symb(2,j) - obj.vars.qr.symb(2,j) >= 0.15,...
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

    function utraj = getUtraj(obj)
      r = PointMassBiped(obj.omega);

      breaks = (0:(obj.nsteps-1)) * obj.dt;
      traj = PPTrajectory(foh(breaks, obj.vars.qcop.value));
      traj = traj.vertcat(PPTrajectory(foh(breaks, obj.vars.qr.value)));
      traj = traj.vertcat(PPTrajectory(foh(breaks, obj.vars.ql.value)));
      contact = obj.getContactSequence();
      traj = traj.vertcat(PPTrajectory(zoh(breaks, contact)));
      traj = traj.vertcat(PPTrajectory(pchipDeriv(breaks, obj.vars.xcom.value(1:2,:), obj.vars.xcom.value(3:4,:))));

      utraj = traj.setOutputFrame(r.getInputFrame());

    end
    
    function contact = getContactSequence(obj)
      motion = [any(abs(diff(obj.vars.qr.value, 1, 2)) >= 0.005), false;
                any(abs(diff(obj.vars.ql.value, 1, 2)) >= 0.005), false];
      % contact = ~(motion | [[false; false], motion(:,1:end-1)] | [motion(:,2:end), [false; false]]);
      contact = ~motion;
    end

    function dynamic_footstep_plan = getDynamicFootstepPlan(obj, biped, foot_start)
      typecheck(biped, 'Biped');
      motion = [any(abs(diff(obj.vars.qr.value, 1, 2)) >= 0.005), false;
                any(abs(diff(obj.vars.ql.value, 1, 2)) >= 0.005), false];
      support = ~(motion | [[false; false], motion(:,1:end-1)] | [motion(:,2:end), [false; false]]);
      support(:,1) = support(:,1) & obj.start.contact;
      % contact = obj.getContactSequence();
      % contained = obj.vars.contained.value;
      body_ind = struct('right', biped.getFrame(biped.foot_frame_id.right).body_ind,...
                        'left', biped.getFrame(biped.foot_frame_id.left).body_ind);
      body_ind_list = [body_ind.right, body_ind.left];
      initial_supports = RigidBodySupportState(biped, body_ind_list(support(:,1)));
      zmp_knots = struct('t', 0, 'zmp', obj.vars.qcop.value(:,1), 'supp', initial_supports);

      offset = [-0.048; 0; 0.0811; 0;0;0];
      foot_origin_knots = struct('t', 0,...
                                 'right', foot_start.right + offset,...
                                 'left', foot_start.left + offset,...
                                 'is_liftoff', false,...
                                 'is_landing', false,...
                                 'toe_off_allowed', struct('right', false, 'left', false));
      warning('ignoring roll and pitch')
      for j = 2:obj.nsteps
        foot_origin_knots(end+1).t = foot_origin_knots(end).t + obj.dt;
        if motion(1,j) || motion(1,j-1)
          zr = 0.025;
        else
          zr = 0;
        end
        if motion(2,j) || motion(2, j-1)
          zl = 0.025;
        else
          zl = 0;
        end
        foot_origin_knots(end).right = [obj.vars.qr.value(:,j); zr; 0; 0; foot_start.right(6)] + offset;
        foot_origin_knots(end).left = [obj.vars.ql.value(:,j); zl; 0; 0; foot_start.left(6)] + offset;
        foot_origin_knots(end).is_liftoff = any(support(:,j) < support(:,j-1));
        if j > 2
          foot_origin_knots(end).is_landing = any(support(:,j) > support(:,j-1));
        else
          foot_origin_knots(end).is_landing = false;
        end
        foot_origin_knots(end).toe_off_allowed = struct('right', false, 'left', false);

        zmp_knots(end+1).t = zmp_knots(end).t + obj.dt;
        zmp_knots(end).zmp = obj.vars.qcop.value(:,j);
        zmp_knots(end).supp = RigidBodySupportState(biped, body_ind_list(support(:,j)));
      end

      foot_origin_knots(end+1) = foot_origin_knots(end);
      foot_origin_knots(end).t = foot_origin_knots(end-1).t + obj.dt

      zmp_knots(end+1) = zmp_knots(end);
      zmp_knots(end).t = zmp_knots(end).t + obj.dt;

      dynamic_footstep_plan = DynamicFootstepPlan(biped, zmp_knots, foot_origin_knots);
    end
  end
end