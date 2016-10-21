function [coefs, ts, objval] = qpSpline(ts, xs, xd0, xdf, settings)
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
if ~isfield(settings, 'use_mex')
  settings.use_mex = true;
end

assert(all(size(ts) == [1,4]));
assert(all(size(xs) == [6,4]));
assert(all(size(xd0) == [6,1]));
assert(all(size(xdf) == [6,1]));

if settings.optimize_knot_times
  [coefs, ts, objval] = nWaypointCubicSplineFreeKnotTimesmex(ts(1), ts(end), xs, xd0, xdf);
else
  if settings.use_mex
    [coefs, objval] = nWaypointCubicSplinemex(ts, xs, xd0, xdf);
  else
    objval = 0;
    for j = 1:6
      x0i = xs(j,1);
      xd0i = xd0(j);
      x1i = xs(j,2);
      x2i = xs(j,3);
      xfi = xs(j,4);
      xdfi = xdf(j);
      
      nv = 0;
      v = struct();
      v.c0.i = nv + (1:4); nv = nv + 4;
      v.c1.i = nv + (1:4); nv = nv + 4;
      v.c2.i = nv + (1:4); nv = nv + 4;
      
      Q = zeros(nv);
      nc = 12;
      E = zeros(nc, nv);
      d = zeros(nc, 1);
      Q(v.c0.i(3:4), v.c0.i(3:4)) = [2^2 * (ts(2) - ts(1)), 6*(ts(2) - ts(1))^2;
        6*(ts(2) - ts(1))^2, 6^2 / 3 *(ts(2) - ts(1))^3];
      Q(v.c1.i(3:4), v.c1.i(3:4)) = [2^2 * (ts(3) - ts(2)), 6*(ts(3) - ts(2))^2;
        6*(ts(3) - ts(2))^2, 6^2 / 3 *(ts(3) - ts(2))^3];
      Q(v.c2.i(3:4), v.c2.i(3:4)) = [2^2 * (ts(4) - ts(3)), 6*(ts(4) - ts(3))^2;
        6*(ts(4) - ts(3))^2, 6^2 / 3 *(ts(4) - ts(3))^3];
      ci = 1;
      E(ci, v.c0.i(1)) = 1;
      d(ci) = x0i;
      ci = ci + 1;
      
      E(ci, v.c0.i(2)) = 1;
      d(ci) = xd0i;
      ci = ci + 1;
      
      E(ci, v.c1.i(1)) = 1;
      d(ci) = x1i;
      ci = ci + 1;
      
      E(ci, v.c2.i(1)) = 1;
      d(ci) = x2i;
      ci = ci + 1;
      
      % Continuity of position
      E(ci, v.c0.i(1)) = 1;
      E(ci, v.c0.i(2)) = (ts(2) - ts(1));
      E(ci, v.c0.i(3)) = (ts(2) - ts(1))^2;
      E(ci, v.c0.i(4)) = (ts(2) - ts(1))^3;
      E(ci, v.c1.i(1)) = -1;
      ci = ci + 1;
      
      E(ci, v.c1.i(1)) = 1;
      E(ci, v.c1.i(2)) = (ts(3) - ts(2));
      E(ci, v.c1.i(3)) = (ts(3) - ts(2))^2;
      E(ci, v.c1.i(4)) = (ts(3) - ts(2))^3;
      E(ci, v.c2.i(1)) = -1;
      ci = ci + 1;
      
      E(ci, v.c2.i(1)) = 1;
      E(ci, v.c2.i(2)) = (ts(4) - ts(3));
      E(ci, v.c2.i(3)) = (ts(4) - ts(3))^2;
      E(ci, v.c2.i(4)) = (ts(4) - ts(3))^3;
      d(ci) = xfi;
      ci = ci + 1;
      
      % Continuity of velocity
      E(ci, v.c0.i(2)) = 1;
      E(ci, v.c0.i(3)) = 2 * (ts(2) - ts(1));
      E(ci, v.c0.i(4)) = 3 * (ts(2) - ts(1))^2;
      E(ci, v.c1.i(2)) = -1;
      ci = ci + 1;
      
      E(ci, v.c1.i(2)) = 1;
      E(ci, v.c1.i(3)) = 2 * (ts(3) - ts(2));
      E(ci, v.c1.i(4)) = 3 * (ts(3) - ts(2))^2;
      E(ci, v.c2.i(2)) = -1;
      ci = ci + 1;
      
      E(ci, v.c2.i(2)) = 1;
      E(ci, v.c2.i(3)) = 2 * (ts(4) - ts(3));
      E(ci, v.c2.i(4)) = 3 * (ts(4) - ts(3))^2;
      d(ci) = xdfi;
      ci = ci + 1;
      
      % Continuity of acceleration
      E(ci, v.c0.i(3)) = 2;
      E(ci, v.c0.i(4)) = 6 * (ts(2) - ts(1));
      E(ci, v.c1.i(3)) = -2;
      ci = ci + 1;
      
      E(ci, v.c1.i(3)) = 2;
      E(ci, v.c1.i(4)) = 6 * (ts(3) - ts(2));
      E(ci, v.c2.i(3)) = -2;
      ci = ci + 1;
      
      z = E \ d;
      
      % M = [Q, E'; E, zeros(nc)];
      % b = [zeros(nv, 1); d];
      % size(M)
      % tic(); z =  M \ b; toc();
      % z = z(1:nv);
      coefsi = [reshape(z(v.c0.i([4:-1:1])), [1, 1, 4]),...
        reshape(z(v.c1.i([4:-1:1])), [1, 1, 4]),...
        reshape(z(v.c2.i([4:-1:1])), [1, 1, 4])];
      objval = objval + z' * Q * z;
      coefs(j,:,:) = coefsi;
      %   valuecheck(coefsi, coefs(j,:,:), 1e-5);
    end
  end
end
end
