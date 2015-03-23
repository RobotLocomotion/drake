classdef QPReactiveRecoveryPlan < QPControllerPlan
  properties
    robot
    qtraj
    mu = 0.5;
    g = 9.81;
    LIP_height;
  end

  methods
    function obj = QPReactiveRecoveryPlan(robot, qtraj, LIP_height)
      obj.robot = robot;
      obj.qtraj = qtraj;
      obj.LIP_height = LIP_height;
    end

    function next_plan = getSuccessor(obj, t, x)
      next_plan = QPLocomotionPlan.from_standing_state(x, obj.robot);
    end

    function qp_input = getQPControllerInput(obj, t_global, x, rpc, contact_force_detected)

      q = x(1:rpc.nq);
      qd = x(rpc.nq + (1:rpc.nv));
      kinsol = doKinematics(obj.robot, q);

      [com, J] = obj.robot.getCOM(kinsol);
      comd = J * qd;

      omega = sqrt(obj.g / obj.LIP_height);

      r_ic = com(1:2) + comd(1:2) / omega;

      % r_ic(t) = (r_ic(0) - r_cop) e^(t*omega) + r_cop
      % 
    end
  end

  methods(Static)
    function p = expTaylor(a, b, c, n)
      % Taylor expansion of a*exp(b*x) + c about x=0 up to degree n
      p = zeros(n+1, length(a));
      for j = 1:n+1
        d = (n+1) - j;
        p(j,:) = a.*b.^d;
        if j == 0
          p(j,:) = p(j,:) + c;
        end
        p(j,:) = p(j,:) / factorial(d);
      end
    end

    function t_int = exp_intercept(a, b, c, l0, ld0, u, n)
      % Find the t >= 0 solutions to a*e^(b*t) + c == l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
      % using a taylor expansion up to power n
      p = expTaylor(a, b, c, n);
      p_spline = [zeros(N-2, 1);
                  0.25 * u;
                  0.5 * ld0;
                  l0 - 0.25 * ld0.^2 / u];


    function icp_intercept(r_ic, r_cop, omega, r_foot, rd_foot, a)
    end
  end
end


