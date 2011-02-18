function ypp = zoh(t0,y0)

% ZERO-ORDER-HOLD
%   Creates a pp form (piecewise polynomial) of order 0 which implments a
%   zero-order hold.  Use ppval, ppval_safe, or fnval to evaluate it.

D = size(y0); L = D(end)-1; D = D(1:(end-1));
if (length(t0)~=L+1) error('t0 and y0 do not match'); end

y0 = reshape(y0,[],L+1);
coefs(:,1) = reshape(y0(:,1:L),[],1);
ypp = mkpp(t0,coefs,D);



