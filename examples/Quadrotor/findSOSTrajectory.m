function [ytraj, diagnostics, objective] = findSOSTrajectory(start, goal, safe_region_sets, traj_degree, num_traj_segments, bot_radius)

if nargin < 6
  bot_radius = 0;
end

dim = length(start);
assert(length(goal) == dim);
nt = num_traj_segments;
C_BOUND = 10;
V_BOUND = 1000;
dt = .5;

t = sdpvar(1,1);
shifted_legendre = [1;
                    2*t - 1;
                    6*t^2 - 6*t + 1;
                    20*t^3 - 30*t^2 + 12*t - 1;
                    70*t^4 - 140*t^3 + 90*t^2 - 20*t + 1;
                    252*t^5 - 630*t^4 + 560*t^3 - 210*t^2 + 30*t - 1;
                    924*t^6 - 2772*t^5 + 3150*t^4 - 1680*t^3 + 420*t^2 - 42*t + 1;
                    3432*t^7 - 12012*t^6 + 16632*t^5 - 11550*t^4 + 4200*t^3 - 756*t^2 + 56*t - 1;
                    ];

% basis = monolist(t,traj_degree);
basis = shifted_legendre(1:traj_degree+1);
assign(t, 0);
basis_t0 = value(basis);
assign(t, 1);
basis_t1 = value(basis);

% s = sdpvar(1);
% sbasis = monolist(s, traj_degree) .* flip(monolist(1+s, traj_degree));

C = {};
for j = 1:nt
  C{j} = sdpvar(traj_degree+1,dim,'full');
  X{j} = C{j}'*basis;
  Xd{j} = {jacobian(X{j}, t)};
  Cd{j} = {[coefficients(Xd{j}{1}(1), t); 0]};
  for d = 2:dim
    Cd{j}{1}(:,d) = [coefficients(Xd{j}{1}(d), t); 0];
  end
  for k = 2:traj_degree
    Xd{j}{k} = jacobian(Xd{j}{k-1}, t);
    Cd{j}{k} = [coefficients(Xd{j}{k}(1), t); zeros(k,1)];
    for d = 2:dim
      Cd{j}{k}(:,d) = [coefficients(Xd{j}{k}(d), t); zeros(k,1)];
    end
  end
end

x0 = start;
xf = goal;

constraints = [...
               C{1}' * basis_t0 == x0,...
               Cd{1}{1}' * [1; zeros(traj_degree,1)] == zeros(dim, 1),...
               Cd{nt}{1}' * [1; ones(traj_degree,1)] == zeros(dim, 1),...
               C{nt}' * basis_t1 == xf,...
              ];
if traj_degree > 1
  constraints = [constraints,...
   Cd{1}{2}' * [1; zeros(traj_degree,1)] == zeros(dim, 1),...
   Cd{nt}{2}' * [1; ones(traj_degree,1)] == zeros(dim, 1),...
   ];
 end

region = {};
for j = 1:length(safe_region_sets)
  if length(safe_region_sets{j}) > 1
    region{j} = binvar(length(safe_region_sets{j}), nt, 'full');
    constraints = [constraints, sum(region{j}, 1) == 1];
  else
    region{j} = true;
  end
end

for j = 1:nt-1
  constraints = [constraints,...
                 C{j}' * basis_t1 == C{j+1}' * basis_t0,...
                 ];
  for d = 1:traj_degree-1;
    constraints = [constraints,...
                   Cd{j}{d}' * [1; ones(traj_degree, 1)] == Cd{j+1}{d}' * [1; zeros(traj_degree, 1)],...
                   ];
  end
end

objective = 0;

V = {};
for j = 1:nt
  V{j} = {};
  constraints = [constraints, ...
                 -C_BOUND <= C{j} <= C_BOUND,...
                 ];
  if traj_degree <= 3
    for d = 1:dim
      objective = objective + Cd{j}{traj_degree}(:,d)' * Cd{j}{traj_degree}(:,d);
    end
  else
    snap_coeffs = Cd{j}{4};
    objective = objective + sum(sum(snap_coeffs' .* snap_coeffs', 1) ./ (1:traj_degree+1));
  end

  for rs = 1:length(region)
    V{j}{rs} = {};
    nr = size(region{rs},1);
    for r = 1:nr
      V{j}{rs}{r} = {};
      A = safe_region_sets{rs}(r).A;
      b = safe_region_sets{rs}(r).b;
      for k = 1:size(A,1)
        ai = A(k,:);
        bi = b(k);
        n = norm(ai);
        ai = ai / n;
        bi = bi / n;
        [coeff, ~] = coefficients(bi - bot_radius - (ai*(C{j}') * basis), t, monolist(t, traj_degree));
        V{j}{rs}{r}{k} = {sdpvar(1, traj_degree), sdpvar(1, traj_degree)};
        % objective = objective + 0.01 * (sum(abs(V{j}{rs}{r}{k}{1})) + sum(abs(V{j}{rs}{r}{k}{2})));
        sigma1 = V{j}{rs}{r}{k}{1}*monolist(t, traj_degree-1);
        sigma2 = V{j}{rs}{r}{k}{2}*monolist(t, traj_degree-1);
        Q1 = sdpvar((traj_degree-1)/2 + 1);
        Q2 = sdpvar((traj_degree-1)/2 + 1);
        m = monolist(t, (traj_degree-1)/2);
        if nr > 1
          constraints = [constraints,...
                         implies(region{rs}(r,j), coeff == coefficients((sigma1)*(t) + (sigma2)*(1-t), t)),...
                         -V_BOUND <= V{j}{rs}{r}{k}{1} <= V_BOUND,...
                         -V_BOUND <= V{j}{rs}{r}{k}{2} <= V_BOUND,...
                        ];
        else
          constraints = [constraints,...
                         coeff == coefficients((sigma1)*(t) + (sigma2)*(1-t), t)];
        end
        if traj_degree == 1
          constraints = [constraints, sigma1 >= 0, sigma2 >= 0];
        elseif traj_degree == 3
          constraints = [constraints, ...
                         rcone(V{j}{rs}{r}{k}{1}(2), 2*V{j}{rs}{r}{k}{1}(1), V{j}{rs}{r}{k}{1}(3)),...
                         rcone(V{j}{rs}{r}{k}{2}(2), 2*V{j}{rs}{r}{k}{2}(1), V{j}{rs}{r}{k}{2}(3)),...
                         ];
        else
          constraints = [constraints,...
                         coefficients(sigma1, t) == coefficients(m'*Q1*m, t),...
                         coefficients(sigma2, t) == coefficients(m'*Q2*m, t),...
                         Q1 >= 0,...
                         Q2 >= 0,...
                       ];
        end
      end
    end
  end
end

t0 = tic;
if traj_degree > 3
  diagnostics = optimize(constraints, objective, sdpsettings('solver', 'bnb', 'bnb.maxiter', 5000, 'verbose', 0, 'debug', true))
else
  % diagnostics = optimize(constraints, objective, sdpsettings('solver', 'gurobi', 'gurobi.MIPGap', 1e-2))
  diagnostics = optimize(constraints, objective, sdpsettings('solver', 'mosek', 'mosek.MSK_DPAR_MIO_TOL_REL_GAP', 1e-2, 'mosek.MSK_DPAR_MIO_MAX_TIME', 600, 'verbose', 0))
end
toc(t0);

breaks = 0:dt:(num_traj_segments*dt);
coeffs = zeros([dim+1, length(breaks)-1, traj_degree+1]);
for k = 1:length(breaks)-1
  c = value(C{k});
  for i = 1:dim
    [ct, ~] = coefficients(c(:,i)' * basis, t, monolist(t, traj_degree));
    if length(ct) < traj_degree + 1
      ct = [ct; 1e-6]; % stupid yalmip bug
    end
    lambda = t / dt;
    z = ct'* monolist(lambda, traj_degree);
    [cz, ~] = coefficients(z, t, monolist(t, traj_degree));
    coeffs(i,k,:) = flip(cz');
  end
end
ytraj = PPTrajectory(mkpp(breaks, coeffs, dim+1));

objective = double(objective);

% keyboard()

axis equal

% end