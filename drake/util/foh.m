function ypp = foh(t0,y0)

% First-ORDER-HOLD
%   Creates a pp form (piecewise polynomial) of order 1 which implements a
%   first-order hold.  Use ppval, ppval_safe, or fnval to evaluate it.
%   Whatever the size of y0, the last dimension is paired up with time. 

D = size(y0); L = D(end)-1; D = D(1:(end-1));
if (length(t0)~=L+1) error('t0 and y0 do not match'); end

y0 = reshape(y0,[],L+1);
t0 = reshape(t0,1,L+1);
coefs(:,2) = reshape(y0(:,1:L),[],1);
coefs(:,1) = reshape(diff(y0,1,2)./repmat(diff(t0),[size(y0,1) 1]),[],1);
ypp = mkpp(t0,coefs,D);



