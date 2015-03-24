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

      % 
    end
  end

  methods(Static)
    function pp = swingTraj(intercept_plan, foot_state, r_foot_new)
      % Plan a one- or two-piece polynomial spline to get to r_foot_new with final velocity 0. 
      

    function best_plan = chooseBestIntercept(intercept_plans)
      [min_error, idx] = min([intercept_plans.error])
      best_plan = intercept_plans(idx);
    end

    function intercept_plans = enumerateCaptureOptions(foot_contact, foot_vertices, foot_state, r_ic, u)
      reachable_l_minus_r = [-0.3, 0.35, 0.35, -0.3;
                             0.15, 0.15, 0.4, 0.4];


      if foot_contact.right && foot_contact.left
        available_feet = struct('stance', {'right', 'left'},...
                                'swing', {'left', 'right'});
      elseif ~foot_contact.right
        available_feet = struct('stance', {'left'},...
                                'swing', {'right'});
      else
        available_feet = struct('stance', {'right'},...
                                'swing', {'left'});
      end
      offset = 0.13;

      intercept_plans = struct('foot', {}, 'r_cop', {}, 'u', {}, 't_intercept', {}, 't_switch', {}, 'error', {});
      for j = 1:length(available_feet)
        stance_vertices = bsxfun(@plus,...
                                 foot_state.(available_feet(j).stance).position(1:2),...
                                 foot_vertices.(available_feet(j).stance);
        if size(vertices, 2) > 4
          error('too long for custom solver');
        end
        r_cop = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                      stance_vertices, r_ic)) + r_ic;
        r_foot = foot_state.(available_feet(j).swing).position;
        rd_foot = foot_state.(available_feet(j).swing).velocity;

        reach_vertices = reachable_l_minus_r;
        if strcmp(available_feet(j).swing, 'right')
          reach_vertices(2,:) = -reach_vertices(2,:);
        end
        reach_vertices = bsxfun(@plus,...
                                foot_state.(available_feet(j).stance).position(1:2),...
                                reach_vertices);

        available_u = [-u, u];
        for k = 1:length(available_u)
          [t_int, t_switch, r_foot_new, r_ic_new] = icpIntercept(r_ic, r_cop, omega, r_foot, rd_foot, available_u(k), offset);

          v_reach = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                          reach_vertices,...
                                                          r_foot_new));
          if norm(v_reach) > 1e-3
            % Desired pose is not reachable, so find the closest point in reachable region
            r_foot_new = v_reach + r_foot_new;
          end

          swing_vertices = bsxfun(@plus,...
                                  r_foot_new,...
                                  foot_vertices.(available_feet(j).stance));

          v_double_support = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                                   [stance_vertices, swing_vertices],...
                                                                   r_ic_new));
          if norm(v_double_support) < 1e-3
            % We've brought the ICP into the support polygon
            r_cop_new = r_ic_new;
          else
            % We're going to have to take another step
            r_cop_new = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                              swing_vertices,...
                                                              r_ic_new)) + r_ic_new;
          end

          intercept_plans(end+1) = struct('foot', available_feet(j).swing, 'r_cop', r_cop, 'u', available_u(k), 't_intercept', t_int, 't_switch', t_switch, 'error', norm(r_cop_new - r_ic_new));
        end
      end
    end

    function p = expTaylor(a, b, c, n)
      % Taylor expansion of a*exp(b*x) + c about x=0 up to degree n
      p = zeros(n+1, length(a));
      for j = 1:n+1
        d = (n+1) - j;
        p(j,:) = a.*b.^d;
        if d == 0
          p(j,:) = p(j,:) + c;
        end
        p(j,:) = p(j,:) / factorial(d);
      end
    end

    function [t_int, l_int] = expIntercept(a, b, c, l0, ld0, u, n)
      % Find the t >= 0 solutions to a*e^(b*t) + c == l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
      % using a taylor expansion up to power n
      p = QPReactiveRecoveryPlan.expTaylor(a, b, c, n);
      p_spline = [zeros(n-2, 1);
                  0.25 * u;
                  0.5 * ld0;
                  l0 - 0.25 * ld0.^2 / u];
      t_int = roots(p - p_spline);
      mask = false(size(t_int));
      for j = 1:size(t_int)
        mask(j) = isreal(t_int(j)) && t_int(j) > 0;
      end
      t_int = t_int(mask);
      l_int = polyval(p_spline, t_int);

      tt = linspace(0, max([t_int; 5]));

      figure(1)
      clf
      hold on
      plot(tt, polyval(p, tt), 'g--');
      plot(tt, a.*exp(b.*tt) + c, 'g-');
      plot(tt, polyval(p_spline, tt), 'r-');
      plot(t_int, polyval(p_spline, t_int), 'ro');
    end

    function [t_int, t_switch, r_foot_new, r_ic_new] = icpIntercept(r_ic, r_cop, omega, r_foot, rd_foot, u, offset)
      nhat = r_ic - r_cop;
      nhat = nhat / norm(nhat);

      l_ic = r_ic' * nhat;
      l_cop = r_cop' * nhat;
      l_foot = r_foot' * nhat;
      ld_foot = rd_foot' * nhat;

      % u_signed = [u, -u];
      % intercepts = struct('u', {u_signed}, 'ts', {{}, {}});
      % for j = 1:2
        % r_ic(t) = (r_ic(0) - r_cop) e^(t*omega) + r_cop
      [t_int, l_int] = QPReactiveRecoveryPlan.expIntercept(l_ic - l_cop, omega, l_cop + offset, l_foot, ld_foot, u, 5);
      % t_int = t_int(t_int >= abs(ld_foot / u));
      t_switch = 0.5 * (t_int - ld_foot / u);

      r_foot_new = l_int * nhat + r_cop;
      r_ic_new = (r_ic - r_cop) * exp(t_int * omega) + r_cop;

      % end
    end
  end
end


