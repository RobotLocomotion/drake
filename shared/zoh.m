function ypp = zoh(t0,y0)

% ZERO-ORDER-HOLD
%   Creates a pp form (piecewise polynomial) of order 0 which implments a
%   zero-order hold.  Use ppval, ppval_safe, or fnval to evaluate it.

ypp = spline(t0,y0);
ypp.coefs = ypp.coefs(:,end);
ypp.order = 1;

% note: might be able to do this a bit more efficiently using mkpp (instead
% of fitting the spline coefs then deleting them)