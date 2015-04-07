function [coefs, ts, objval] = qpSpline(ts, xs, xd0, xdf, settings, debug)
% Compute a smooth spline through four knot points with fixed velocity at the
% initial and final knots only. Solves a (very small) QP using cvxgen to
% minimize the integral of the squared norm of acceleration along the
% trajectory.
% @param ts [1x4] times
% @param xs [6x4] 6-DOF poses at each knot
% @param xd0 [6x1] initial velocity
% @param xdf [6x1] final velocity
% @param settings settings struct; options
%   - optimize_knot_times @default false
% @option debug

if ~isfield(settings, 'optimize_knot_times')
  settings.optimize_knot_times = false;
end

if nargin < 6
  debug = false;
end

assert(all(size(ts) == [1,4]));
assert(all(size(xs) == [6,4]));
assert(all(size(xd0) == [6,1]));
assert(all(size(xdf) == [6,1]));

params = struct('x0', xs(:,1),...
                'xd0', xd0,...
                'x1', xs(:,2),...
                'x2', xs(:,3),...
                'xf', xs(:,4),...
                'xdf', xdf,...
                't0', ts(1),...
                't1', ts(2),...
                't2', ts(3),...
                'tf', ts(4),...
                'Q0', [2^2 * (ts(2) - ts(1)), 6*(ts(2) - ts(1))^2;
                       6*(ts(2) - ts(1))^2, 6^2 / 3 *(ts(2) - ts(1))^3],...
                'Q1', [2^2 * (ts(3) - ts(2)), 6*(ts(3) - ts(2))^2;
                       6*(ts(3) - ts(2))^2, 6^2 / 3 *(ts(3) - ts(2))^3],...
                'Q2', [2^2 * (ts(4) - ts(3)), 6*(ts(4) - ts(3))^2;
                       6*(ts(4) - ts(3))^2, 6^2 / 3 *(ts(4) - ts(3))^3]);
cvx_settings = struct('verbose', 0);

if settings.optimize_knot_times
  tic()
  [vars, status] = qpSplineGridSearchmex(params, cvx_settings);
  toc()
  ts(2) = vars.t1;
  ts(3) = vars.t2;
else

  [vars, status] = qpSplineMex(params, cvx_settings);
end
coefs = [cat(3, vars.C0_3, vars.C0_2, vars.C0_1, vars.C0_0),...
         cat(3, vars.C1_3, vars.C1_2, vars.C1_1, vars.C1_0),...
         cat(3, vars.C2_3, vars.C2_2, vars.C2_1, vars.C2_0)];
objval = status.optval;

% objvali = 0;
% for j = 1:6
%   x0i = xs(j,1);
%   xd0i = xd0(j);
%   x1i = xs(j,2);
%   x2i = xs(j,3);
%   xfi = xs(j,4);
%   xdfi = xdf(j);

%   nv = 0;
%   v = struct();
%   v.c0.i = nv + (1:4); nv = nv + 4;
%   v.c1.i = nv + (1:4); nv = nv + 4;
%   v.c2.i = nv + (1:4); nv = nv + 4;

%   Q = zeros(nv);
%   nc = 12;
%   E = zeros(nc, nv);
%   d = zeros(nc, 1);
%   Q(v.c0.i(3:4), v.c0.i(3:4)) = [2^2 * (ts(2) - ts(1)), 6*(ts(2) - ts(1))^2;
%                                  6*(ts(2) - ts(1))^2, 6^2 / 3 *(ts(2) - ts(1))^3];
%   Q(v.c1.i(3:4), v.c1.i(3:4)) = [2^2 * (ts(3) - ts(2)), 6*(ts(3) - ts(2))^2;
%                                  6*(ts(3) - ts(2))^2, 6^2 / 3 *(ts(3) - ts(2))^3];
%   Q(v.c2.i(3:4), v.c2.i(3:4)) = [2^2 * (ts(4) - ts(3)), 6*(ts(4) - ts(3))^2;
%                                  6*(ts(4) - ts(3))^2, 6^2 / 3 *(ts(4) - ts(3))^3];
%   ci = 1;
%   E(ci, v.c0.i(1)) = 1;
%   d(ci) = x0i;
%   ci = ci + 1;

%   E(ci, v.c0.i(2)) = 1;
%   d(ci) = xd0i;
%   ci = ci + 1;

%   E(ci, v.c1.i(1)) = 1;
%   d(ci) = x1i;
%   ci = ci + 1;

%   E(ci, v.c2.i(1)) = 1;
%   d(ci) = x2i;
%   ci = ci + 1;

%   % Continuity of position
%   E(ci, v.c0.i(1)) = 1;
%   E(ci, v.c0.i(2)) = (ts(2) - ts(1));
%   E(ci, v.c0.i(3)) = (ts(2) - ts(1))^2;
%   E(ci, v.c0.i(4)) = (ts(2) - ts(1))^3;
%   E(ci, v.c1.i(1)) = -1;
%   ci = ci + 1;

%   E(ci, v.c1.i(1)) = 1;
%   E(ci, v.c1.i(2)) = (ts(3) - ts(2));
%   E(ci, v.c1.i(3)) = (ts(3) - ts(2))^2;
%   E(ci, v.c1.i(4)) = (ts(3) - ts(2))^3;
%   E(ci, v.c2.i(1)) = -1;
%   ci = ci + 1;

%   E(ci, v.c2.i(1)) = 1;
%   E(ci, v.c2.i(2)) = (ts(4) - ts(3));
%   E(ci, v.c2.i(3)) = (ts(4) - ts(3))^2;
%   E(ci, v.c2.i(4)) = (ts(4) - ts(3))^3;
%   d(ci) = xfi;
%   ci = ci + 1;

%   % Continuity of velocity
%   E(ci, v.c0.i(2)) = 1;
%   E(ci, v.c0.i(3)) = 2 * (ts(2) - ts(1));
%   E(ci, v.c0.i(4)) = 3 * (ts(2) - ts(1))^2;
%   E(ci, v.c1.i(2)) = -1;
%   ci = ci + 1;

%   E(ci, v.c1.i(2)) = 1;
%   E(ci, v.c1.i(3)) = 2 * (ts(3) - ts(2));
%   E(ci, v.c1.i(4)) = 3 * (ts(3) - ts(2))^2;
%   E(ci, v.c2.i(2)) = -1;
%   ci = ci + 1;

%   E(ci, v.c2.i(2)) = 1;
%   E(ci, v.c2.i(3)) = 2 * (ts(4) - ts(3));
%   E(ci, v.c2.i(4)) = 3 * (ts(4) - ts(3))^2;
%   d(ci) = xdfi;
%   ci = ci + 1;

%   % Continuity of acceleration
%   E(ci, v.c0.i(3)) = 2;
%   E(ci, v.c0.i(4)) = 6 * (ts(2) - ts(1));
%   E(ci, v.c1.i(3)) = -2;
%   ci = ci + 1;

%   E(ci, v.c1.i(3)) = 2;
%   E(ci, v.c1.i(4)) = 6 * (ts(3) - ts(2));
%   E(ci, v.c2.i(3)) = -2;
%   ci = ci + 1;

%   tic()
%   z = E \ d;
%   toc();

%   % M = [Q, E'; E, zeros(nc)];
%   % b = [zeros(nv, 1); d];
%   % size(M)
%   % tic(); z =  M \ b; toc();
%   % z = z(1:nv);
%   coefsi = [reshape(z(v.c0.i([4:-1:1])), [1, 1, 4]),...
%             reshape(z(v.c1.i([4:-1:1])), [1, 1, 4]),...
%             reshape(z(v.c2.i([4:-1:1])), [1, 1, 4])];
%   objvali = objvali + z' * Q * z;
%   coefs(j,:,:) = coefsi;
%   valuecheck(coefsi, coefs(j,:,:), 1e-5);
% end

% valuecheck(objvali, status.optval);


if debug
  pp = mkpp(ts, coefs, 6);

  tt = linspace(ts(1), ts(end), 1000);
  ps = ppval(pp, tt);
  figure(15);
  clf
  for j = 1:6
    subplot(6, 1, j)
    hold on
    plot(tt, ps(j,:));
    plot(ts, xs(j,:), 'ro');
  end

  atraj = fnder(pp, 2);
  as = ppval(atraj, tt);
  ns = sum(as.^2,1);
  error = abs(sum(ns) * (tt(2) - tt(1)) - status.optval) / status.optval

  figure(16);
  clf
  vs = ppval(fnder(pp, 1), tt);
  for j = 1:6
    subplot(6, 1, j)
    plot(tt, vs(j,:));
  end

  figure(17);
  clf
  for j = 1:6
    subplot(6,1,j);
    plot(tt, as(j,:));
  end

  rangecheck(error, 0, 1e-2);
end
