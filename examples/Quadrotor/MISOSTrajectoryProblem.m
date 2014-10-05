classdef MISOSTrajectoryProblem
% Compute a smooth polynomial trajectory through an n-dimensional space while remaining within convex safe regions.
% We build a piecewise polynomial trajectory of the specified degree from the start pose to the goal pose, while 
% ensuring that each polynomial is fully contained within one convex region from each set of safe regions. 
% This function implements a general form of the algorithm presented in:
%    Robin Deits and Russ Tedrake. "Efficient Mixed-Integer Planning for UAVs in Cluttered Environments." Submitted to:
%    ICRA 2015, Seattle, WA. 
  properties
    traj_degree = 3; % option: degree of the polynomials used
    num_traj_segments = 6; % option: number of polynomial pieces in the trajectory
    bot_radius = 0; % option: if nonzero, then treat the robot as a sphere with the given radius (and thus require that it lie
                    %         at least this distance within the polytopes defined by the safe regions).
    basis = 'legendre'; % option: 'monomials' or 'legendre'. The polynomial basis to use.
    dt = 0.5; % Duration of each trajectory piece (seconds)
  end

  methods
    function [ytraj, diagnostics, objective, safe_region_assignments] = solveTrajectory(obj, start, goal, safe_region_sets, safe_region_assignments)
      % @param start [n x m1]: starting state. The first column of start is the initial position in n-dimensional space. 
      %              The remaining columns are (optional) constraints on the derivatives at the initial time. For example,
      %              to constrain only the initial position in 2D, set start = [0;0]. To constrain position and velocity, 
      %              set start = [0 0; 0 0]. To constrain position, velocity, and acceleration, set start = [0 0 0; 0 0 0];
      % @param goal [n x m2]: final state. Same format as start, but can have a different number of columns if, for example,
      %             you want to constrain initial position and velocity but only the final position.
      % @param safe_region_sets struct array with fields 'A', and 'b'; or cell array of struct arrays. The safe regions are 
      %                         the convex areas of obstacle-free space in which the trajectories must be contained. If 
      %                         a struct array, then each trajectory piece P(t) must satisfy:
      %                            safe_region_sets(j).A P(t) <= safe_region_sets(j).b for all 0 <= t <= 1
      %                         If safe_region_sets is a cell array of struct arrays, then for every i there must be a j
      %                            such that:
      %                            safe_region_sets{i}(j).A P(t) <= safe_region_sets{i}(j).b for all 0 <= t <= 1
      % @option safe_region_assignments logical array or cell array of logicals. If you would like to fix the assignment
      %                                 of trajectories to safe regions from a previous run, then you can pass in the
      %                                 safe_region_assignments directly from the prior output. 
      % @retval ytraj a PPTrajectory 

    checkDependency('yalmip');
    if nargin < 5
      safe_region_assignments = [];
    end

    if size(start, 2) > obj.traj_degree + 1
      warning('For a degree d polynomial, we can only constrain at most the first d derivatives. Additional derivatives ignored.');
      start = start(:,1:(obj.traj_degree+1));
    end
    if size(goal, 2) > obj.traj_degree + 1
      warning('For a degree d polynomial, we can only constrain at most the first d derivatives. Additional derivatives ignored.');
      goal = goal(:,1:(obj.traj_degree+1));
    end

    if ~iscell(safe_region_sets)
      safe_region_sets = {safe_region_sets};
    end
    if ~iscell(safe_region_assignments)
      safe_region_assignments = {safe_region_assignments};
    end

    dim = size(start, 1);
    sizecheck(goal, [dim, nan]);


    C_BOUND = 10;
    SIGMA_BOUND = 1000;

    t = sdpvar(1,1);

    if strcmp(obj.basis, 'monomials')
      basis = monolist(t,obj.obj.traj_degree);
    elseif strcmp(obj.basis, 'legendre')
      shifted_legendre = [1;
                          2*t - 1;
                          6*t^2 - 6*t + 1;
                          20*t^3 - 30*t^2 + 12*t - 1;
                          70*t^4 - 140*t^3 + 90*t^2 - 20*t + 1;
                          252*t^5 - 630*t^4 + 560*t^3 - 210*t^2 + 30*t - 1;
                          924*t^6 - 2772*t^5 + 3150*t^4 - 1680*t^3 + 420*t^2 - 42*t + 1;
                          3432*t^7 - 12012*t^6 + 16632*t^5 - 11550*t^4 + 4200*t^3 - 756*t^2 + 56*t - 1;
                          ];
      basis = shifted_legendre(1:obj.traj_degree+1);
    else
      error('Invalid basis name: %s', obj.basis);
    end

    assign(t, 0);
    basis_t0 = value(basis);
    assign(t, 1);
    basis_t1 = value(basis);


    % Set up our array of coefficients C and our array of derivative coefficients Cd
    % Each element of C, C{j} is the vector of coefficients of polynomial piece j.
    % Each element of Cd, Cd{j}{k} is the vector of coefficients of the kth derivative of the jth polynomial piece.
    C = {};
    for j = 1:obj.num_traj_segments
      C{j} = sdpvar(obj.traj_degree+1,dim,'full');
      X{j} = C{j}'*basis;
      Xd{j} = {jacobian(X{j}, t)};
      Cd{j} = {[coefficients(Xd{j}{1}(1), t); 0]};
      for d = 2:dim
        Cd{j}{1}(:,d) = [coefficients(Xd{j}{1}(d), t); 0];
      end
      for k = 2:obj.traj_degree
        Xd{j}{k} = jacobian(Xd{j}{k-1}, t);
        Cd{j}{k} = [coefficients(Xd{j}{k}(1), t); zeros(k,1)];
        for d = 2:dim
          Cd{j}{k}(:,d) = [coefficients(Xd{j}{k}(d), t); zeros(k,1)];
        end
      end
    end

    x0 = start(:,1);
    xf = goal(:,1);

    constraints = [];

    % Constrain the initial position and derivatives, as requested
    for j = 1:size(start, 2)
      if j == 1
        constraints = [constraints, C{1}' * basis_t0 == start(:,j)];
      else
        constraints = [constraints, Cd{1}{j-1}' * [1; zeros(obj.traj_degree, 1)] == start(:,j)];
      end
    end

    % Constrain the final position and derivatives, as requested
    for j = 1:size(goal, 2)
      if j == 1
        constraints = [constraints, C{obj.num_traj_segments}' * basis_t1 == goal(:,j)];
      else
        constraints = [constraints, Cd{obj.num_traj_segments}{j-1}' * [1; ones(obj.traj_degree, 1)] == goal(:,j)];
      end
    end

    % Region is a cell array. Each element region{l} is a matrix of binary variables denoting the assignment
    % of each trajectory piece to a convex safe region. 
    region = {};
    for j = 1:length(safe_region_sets)
      if length(safe_region_sets{j}) > 1
        if isempty(cell2mat(safe_region_assignments))
          region{j} = binvar(length(safe_region_sets{j}), obj.num_traj_segments, 'full');
          constraints = [constraints, sum(region{j}, 1) == 1];
        else
          region{j} = safe_region_assignments{j};
        end
      else
        region{j} = true;
      end
    end

    % Enforce continuity of the derivatives
    for j = 1:obj.num_traj_segments-1
      constraints = [constraints,...
                     C{j}' * basis_t1 == C{j+1}' * basis_t0,...
                     ];
      for d = 1:obj.traj_degree-1;
        constraints = [constraints,...
                       Cd{j}{d}' * [1; ones(obj.traj_degree, 1)] == Cd{j+1}{d}' * [1; zeros(obj.traj_degree, 1)],...
                       ];
      end
    end

    objective = 0;

    sigma = {};
    for j = 1:obj.num_traj_segments
      sigma{j} = {};
      if isempty(cell2mat(safe_region_assignments))
        constraints = [constraints, ...
                       -C_BOUND <= C{j} <= C_BOUND,...
                       ];
      end
      if obj.traj_degree <= 3
        for d = 1:dim
          objective = objective + Cd{j}{obj.traj_degree}(:,d)' * Cd{j}{obj.traj_degree}(:,d);
        end
      else
        snap_coeffs = Cd{j}{4};
        objective = objective + sum(sum(snap_coeffs' .* snap_coeffs', 1) ./ (1:obj.traj_degree+1));
      end

      % Set up the SOS constraints to keep each polynomial in its assigned region
      for rs = 1:length(region)
        sigma{j}{rs} = {};
        nr = size(region{rs},1);
        for r = 1:nr
          sigma{j}{rs}{r} = {};
          A = safe_region_sets{rs}(r).A;
          b = safe_region_sets{rs}(r).b;
          for k = 1:size(A,1)
            ai = A(k,:);
            bi = b(k);
            n = norm(ai);
            ai = ai / n;
            bi = bi / n;
            [coeff, ~] = coefficients(bi - obj.bot_radius - (ai*(C{j}') * basis), t, monolist(t, obj.traj_degree));
            sigma{j}{rs}{r}{k} = {sdpvar(1, obj.traj_degree), sdpvar(1, obj.traj_degree)};
            % objective = objective + 0.01 * (sum(abs(sigma{j}{rs}{r}{k}{1})) + sum(abs(sigma{j}{rs}{r}{k}{2})));
            sigma1 = sigma{j}{rs}{r}{k}{1}*monolist(t, obj.traj_degree-1);
            sigma2 = sigma{j}{rs}{r}{k}{2}*monolist(t, obj.traj_degree-1);
            Q1 = sdpvar((obj.traj_degree-1)/2 + 1);
            Q2 = sdpvar((obj.traj_degree-1)/2 + 1);
            m = monolist(t, (obj.traj_degree-1)/2);
            if isa(region{rs}(r,j), 'sdpvar')
              constraints = [constraints,...
                             implies(region{rs}(r,j), coeff == coefficients((sigma1)*(t) + (sigma2)*(1-t), t)),...
                             -SIGMA_BOUND <= sigma{j}{rs}{r}{k}{1} <= SIGMA_BOUND,...
                             -SIGMA_BOUND <= sigma{j}{rs}{r}{k}{2} <= SIGMA_BOUND,...
                            ];
            elseif region{rs}(r,j)
              constraints = [constraints,...
                             coeff == coefficients((sigma1)*(t) + (sigma2)*(1-t), t)];
            end
            if obj.traj_degree == 1
              constraints = [constraints, sigma1 >= 0, sigma2 >= 0];
            elseif obj.traj_degree == 3
              constraints = [constraints, ...
                             rcone(sigma{j}{rs}{r}{k}{1}(2), 2*sigma{j}{rs}{r}{k}{1}(1), sigma{j}{rs}{r}{k}{1}(3)),...
                             rcone(sigma{j}{rs}{r}{k}{2}(2), 2*sigma{j}{rs}{r}{k}{2}(1), sigma{j}{rs}{r}{k}{2}(3)),...
                             ];
            else
              constraints = [constraints,...
                             coefficients(sigma1, t) == coefficients(m'*Q1*m, t),...
                             coefficients(sigma2, t) == coefficients(m'*Q2*m, t),...
                             Q1 >= 0,...
                             Q2 >= 0,...
                           ];
              objective = objective + trace(Q1) + trace(Q2);
            end
          end
        end
      end
    end

    t0 = tic;
    if obj.traj_degree > 3 && isempty(cell2mat(safe_region_assignments))
      diagnostics = optimize(constraints, objective, sdpsettings('solver', 'bnb', 'bnb.maxiter', 5000, 'verbose', 0, 'debug', true));
    else
      if checkDependency('mosek');
        diagnostics = optimize(constraints, objective, sdpsettings('solver', 'mosek', 'mosek.MSK_DPAR_MIO_TOL_REL_GAP', 1e-2, 'mosek.MSK_DPAR_MIO_MAX_TIME', 600, 'verbose', 3));
      else
        warning('Mosek not found. Will fall back to gurobi');
        checkDependency('gurobi');
        diagnostics = optimize(constraints, objective, sdpsettings('solver', 'gurobi', 'gurobi.MIPGap', 1e-2));
      end
    end
    toc(t0);

    breaks = 0:obj.dt:(obj.num_traj_segments*obj.dt);
    coeffs = zeros([dim, length(breaks)-1, obj.traj_degree+1]);

    if diagnostics.problem == 0
      % Build the output trajectory
      for k = 1:length(breaks)-1
        c = value(C{k});
        for i = 1:dim
          [ct, ~] = coefficients(c(:,i)' * basis, t, monolist(t, obj.traj_degree));
          if length(ct) < obj.traj_degree + 1
            ct = [ct; 1e-6]; % stupid yalmip bug when polynomial is constant
          end
          lambda = t / obj.dt;
          z = ct'* monolist(lambda, obj.traj_degree);
          [cz, ~] = coefficients(z, t, monolist(t, obj.traj_degree));
          coeffs(i,k,:) = flip(cz');
        end
      end
    end
    
    ytraj = PPTrajectory(mkpp(breaks, coeffs, dim));

    objective = double(objective);

    if isempty(cell2mat(safe_region_assignments))
      safe_region_assignments = cell(1, length(region));
      for j = 1:length(region)
        safe_region_assignments{j} = logical(round(double(region{j})));
      end
    end

    end
  end
end