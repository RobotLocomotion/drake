function ps = msspoly2sdpvar(xm,xs,pm)

% SPOT to YALMIP converter
% flips an msspoly polynomial, pm, which depends on variables xm, and
% outputs the sdpvar polynomial, ps, in terms of the sdpvars xp.

typecheck(xm,'msspoly');
typecheck(xs,'sdpvar');
typecheck(pm,'msspoly');


[x,p,M] = decomp(pm);
% decomp places the variables in a different order.
% we must sort xs to match this order.
[~,xn]  = isfree(x);  % extract ids of x.
[~,xmn] = isfree(xm); % extract ids of xm.
perm = mss_match(xmn,xn); % compute permutation.
xs = xs(perm);

% want to use this (from decomp), but yalmip doesn't have prod(x,dim):
%ps=reshape(M*prod(repmat(xs',size(p,1),1).^p,2),size(pm));

% here is the unrolled version
ps=ones(size(p,1),1);
for i=1:size(p,2)
  ps = ps.*(repmat(xs(i,:)',size(p,1),1).^p(:,i));
end
ps = reshape(M*ps,size(pm));



