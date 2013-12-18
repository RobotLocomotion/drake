function [x,objval,exitflag] = gloptipoly(obj,options)

checkDependency('gloptipoly3');
checkDependency('sedumi');

vars = obj.decision_vars;
mpol('v',length(vars));
objective = msspoly2mpol(vars,v,obj.poly_objective);

if ~isempty(obj.poly_equality_constraints)
  error('not implemented yet, but should be easy');
end
if ~isempty(obj.poly_inequality_constraints)
  error('not implemented yet, but should be easy');
end

prog = msdp(min(objective));
[exitflag,objval] = msol(prog);
x = double(v);

end

function pmpol = msspoly2mpol(xmss,xmpol,pmss)

[x,p,M] = decomp(pmss);
% decomp places the variables in a different order.
% we must sort xs to match this order.
[~,xn]  = isfree(x);  % extract ids of x.
[~,xmn] = isfree(xmss); % extract ids of xm.
perm = mss_match(xmn,xn); % compute permutation.
xmpol = xmpol(perm);

% want to use this (from decomp), but mpol doesn't have prod(x,dim):
%pmpol=reshape(M*prod(repmat(xmpol',size(p,1),1).^p,2),size(pmss));

% here is the unrolled version
pmpol=ones(size(p,1),1);
for i=1:size(p,2)
  pmpol = pmpol.*(repmat(xmpol(i,:)',size(p,1),1).^p(:,i));
end
pmpol = reshape(M*pmpol,size(pmss));


end