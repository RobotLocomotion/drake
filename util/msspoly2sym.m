function ps = msspoly2sdpvar(xm,xs,pm)

% SPOT to symbolic matlab converter
% flips an msspoly polynomial, pm, which depends on variables xm, and
% outputs the sym polynomial, ps, in terms of the syms xp.

typecheck(xm,'msspoly');
typecheck(xs,'sym');
typecheck(pm,'msspoly');

[x,p,M] = decomp(pm);
% decomp places the variables in a different order.
% we must sort xs to match this order.
[~,xn]  = isfree(x);  % extract ids of x.
[~,xmn] = isfree(xm); % extract ids of xm.
perm = mss_match(xmn,xn); % compute permutation.
xs = xs(perm);

% want to use this (from decomp), but yalmip doesn't have prod(x,dim):
ps=reshape(M*prod(repmat(xs',size(p,1),1).^p,2),size(pm));




