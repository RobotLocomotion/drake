classdef HolonomicDrivePlant < SecondOrderSystem
  properties (SetAccess = private, GetAccess = public)
    m
    I
    wheels
  end

  methods
    function obj = HolonomicDrivePlant(wheels, I, m)
      % wheels: struct of
      %   pos:      offset from center of mass
      %   r:        wheel radius
      %   driveDir: drive direction, a unit vector
      %   slipDir:  slip direction. For mecanum wheels, this is not
      %             orthogonal to the drive direction
      %   omegaMax: motor free-running speed
      %   tauMax:   motor stall-torque
      % I: moment of inertia of entire robot about vertical axis
      % m: total mass of robot
      assert(isrow(wheels), 'Wheels must be a row vector');
      n = length(wheels);

      obj = obj@SecondOrderSystem(3, n, true);

      obj.I = I;
      obj.m = m;
      obj.wheels = wheels;

      obj = setInputLimits(obj,-1,1);

      obj = setInputFrame(obj,CoordinateFrame('HolonomicInput',n,'u',...
        arrayfun(@(i) sprintf('u%d', i), 1:n, 'Unif', false)));
      obj = setStateFrame(obj,CoordinateFrame('HolonomicState',6,'x',...
        {'x','y','theta', 'x_dot','y_dot', 'theta_dot'}));
      obj = setOutputFrame(obj,obj.getStateFrame);
    end

    function speeds = rotorSpeeds(obj, vel, omega)
      % get the motor rotor speeds needed to give the robot a certain
      % velocity in body space
      i = 1;
      speeds = zeros(length(obj.wheels), 1)*vel(1); % to help TaylorVar
      for wheel = obj.wheels
        % r cross omega
        rotv = omega * [-wheel.pos(2); wheel.pos(1)];
        totalv = vel + rotv;

        % decompose into powered and unpowered direction
        vwheel = [wheel.driveDir wheel.slipDir] \ totalv;

        speeds(i) = vwheel(1) / wheel.r;
        i = i + 1;
      end
    end

    function [qdd, qdddx] = sodynamics(obj,t,q,qd,u)
      n = length(obj.wheels);

      if nargout > 1
        qdddx = zeros(3, 7 + n);
        % dynamics are linear in each wheel.
        for i = 1:n
          wheel = obj.wheels(i);
          qdddxi = oneWheelGradients(obj.m, obj.I, wheel, t, q, qd, u(i));
          qdddx(:,1:7) = qdddx(:,1:7) + qdddxi(:,1:7);

          % qdd / du_i
          qdddx(:,7+i) = qdddxi(:,8);
        end
      end

      theta = q(3);
      rotation = rotmat(theta);

      bodyvel = rotation' * qd(1:2);

      speeds = obj.rotorSpeeds(bodyvel, qd(3));


      total_force = [0; 0];
      total_moment = 0;
      for i = 1:n
        wheel = obj.wheels(i);
        % simplified motor dynamics
        tau = wheel.tauMax * (u(i) - speeds(i) / wheel.omegaMax);

        % assign direction to force
        forceDir = [-wheel.slipDir(2); wheel.slipDir(1)];
        f = tau / wheel.r * forceDir / (forceDir' * wheel.driveDir);

        % combine forces
        total_force = total_force + f;
        total_moment = total_moment...
                   + wheel.pos(1) * f(2)...
                   - wheel.pos(2) * f(1);
      end

      % convert force to world space
      total_force = rotation * total_force;

      qdd = [total_force / obj.m; total_moment / obj.I];
    end

    function [utraj,xtraj] = optimalTrajectory(p,x0,xf, tf0)
      N = 51;
      prog = DircolTrajectoryOptimization(p,N,[tf0/2 tf0*2]);
      prog = prog.addInputConstraint(BoundingBoxConstraint(p.umin, p.umax), 1:N);
      prog = prog.addStateConstraint(ConstantConstraint(x0), 1);
      prog = prog.addStateConstraint(ConstantConstraint(xf), N);
      prog = prog.addFinalCost(@finalCost);

      function [h,dh] = finalCost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
      end

      traj_init.x = PPTrajectory(foh([0,tf0],[x0,xf]));
      info = 0;
      while info~=1
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0, traj_init);
        toc
      end
    end

    function v = constructVisualizer(obj)
      v = HolonomicDriveVisualizer(obj);
    end

    function overall_traj = appendWheelStates(obj, xtraj)
      % given a trajectory, augments it with wheel angle information, useful for rendering
      % Only a convenience method - use HolonomicDriveWheelSensors directly for when this
      % information is used numerically.
      sensors = HolonomicDriveWheelSensors(obj);
      wheeltraj = simulate(cascade(xtraj, sensors), xtraj.tspan, zeros(length(obj.wheels), 1));
      overall_traj = [xtraj; wheeltraj];
    end
  end

  methods(Static)
    function obj = unitModel(n)
      % Construct a drive where all params are equal to 1, just like
      % in the real world. n is the number of wheels, and defaults to 3
      r = 1;
      if ~exist('n', 'var')
        n = 3;
      end

      wheels = [];
      for i = 1:n
        theta = 2*pi * (2*i-1)/(2*n);
        wheels(i).pos = [r*cos(theta); r*sin(theta)];

        % first column - active direction
        % second column - passive direction
        wheels(i).driveDir = [-sin(theta); cos(theta)];
        wheels(i).slipDir = [cos(theta); sin(theta)];
        wheels(i).omegaMax = 1;
        wheels(i).tauMax = 1;
        wheels(i).r = 1;
      end

      obj = HolonomicDrivePlant(wheels, 1, 1);
    end

    function runOpenLoop()
      % Try and move in a certain direction using open-loop control
      % Plots robot speed over time
      plant = HolonomicDrivePlant.unitModel(3);

      u0 = plant.rotorSpeeds([2; 1], 0);
      u0 = u0 / max(u0);
      x0 = Point(plant.getStateFrame, zeros(plant.getNumOutputs, 1));

      z = zeros(plant.getNumInputs, 1);
      u = zoh([0 15 20], [u0,z,z]);
      utraj = PPTrajectory(u);
      utraj = setOutputFrame(utraj, plant.getInputFrame);
      sys = cascade(utraj, plant);

      xtraj = simulate(sys, [0 20], x0);

      % plot the speed
      ts = 0:1/30:20;
      xs = xtraj.eval(ts);
      figure();
      xlabel('t / s');
      ylabel('speed / ms^-1');
      plot(ts, sqrt(xs(4,:).*xs(4,:) + xs(5,:).*xs(5,:)));

      % visualize the motion
      v = HolonomicDriveVisualizer(plant);
      function draw(t, x)
        v.draw(t, x);
        xs = xtraj.eval(xtraj.tspan(1):1/30:t);
        plot(xs(1,:), xs(2,:));
      end
      v2 = FunctionHandleVisualizer(plant.getOutputFrame, @draw);
      v2.playback(xtraj, struct('slider', true));
    end

    function runTrajectory()
      % Perform a simple trajectory optimization, to get to a location
      % in the minimum time.
      plant = HolonomicDrivePlant.unitModel(3);

      x0 = zeros(6, 1);
      xf = [0; 5; 0; 0; 0; 0];

      [utraj, xtraj] = plant.optimalTrajectory(x0, xf, 5);

      real_xtraj = simulate(cascade(utraj, plant), xtraj.tspan, x0);

      figure;
      % plot inputs
      ts = linspace(utraj.tspan(1), utraj.tspan(2), 200);
      subplot(1, 2, 1);
      ylabel('u');
      xlabel('t / s');
      plot(ts, utraj.eval(ts));

      % plot trajectory
      subplot(1, 2, 2);
      xs = xtraj.eval(ts);
      plot(xs(1,:), xs(2,:));

      % plot orientations
      ts_sparse = linspace(utraj.tspan(1), utraj.tspan(2), 20);
      xs_sparse = xtraj.eval(ts_sparse);
      quiver(xs_sparse(1,:), xs_sparse(2,:), cos(xs_sparse(3,:)), sin(xs_sparse(3,:)))

      x2s = real_xtraj.eval(ts);

      v1 = HolonomicDriveVisualizer(plant);
      function draw(t, x)
        xs = xtraj.eval(ts(ts < t));
        plot(x2s(1,:), x2s(2,:), 'b:');
        plot(xs(1,:), xs(2,:), 'b');

        v1.draw(t, x)
      end
      v2 = FunctionHandleVisualizer(plant.getOutputFrame, @draw);

      v2.playback(xtraj, struct('slider', true));
    end
  end
end